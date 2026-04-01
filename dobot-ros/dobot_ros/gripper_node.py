#!/usr/bin/env python3
"""
ROS2 node for DH Robotics gripper via Dobot internal Modbus TCP.

Owns the single Modbus connection to the gripper. Publishes state on
/gripper/state topic, provides /gripper/init and /gripper/status services,
and the /gripper action for move commands with feedback.

Register map (DH Robotics AG-95):
    Write (SetHoldRegs):
        0x0100 (256)  Init        0xA5 (165) = full init
        0x0101 (257)  Force       20-100 % (also controls speed on AG-95)
        0x0102 (258)  Reserved
        0x0103 (259)  Position    0 (closed) to 1000 (open)
        0x0104 (260)  Speed       1-100 % (PGC/PGE only — AG-95 ignores this)

    Read (GetHoldRegs):
        0x0200 (512)  Init status   0=not init, 1=initialized
        0x0201 (513)  Grip state    0=moving, 1=reached, 2=caught, 3=dropped
        0x0202 (514)  Current pos   0-1000
"""

import json
import re
import threading
import time

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from std_msgs.msg import String
from std_srvs.srv import Trigger

from dobot_actions.action import Gripper
from dobot_msgs_v4.srv import (
    ModbusCreate,
    ModbusClose,
    SetHoldRegs,
    GetHoldRegs,
    SetTool485,
)

# Write registers
REG_INIT = 256       # 0x0100
REG_FORCE = 257      # 0x0101
REG_POSITION = 259   # 0x0103
REG_SPEED = 260      # 0x0104

# Read registers
REG_INIT_STATUS = 512  # 0x0200
REG_GRIP_STATE = 513   # 0x0201
REG_CURRENT_POS = 514  # 0x0202

INIT_VALUE = 165  # 0xA5
GRIP_STATE_NAMES = {0: 'moving', 1: 'reached', 2: 'caught', 3: 'dropped'}


class GripperNode(Node):
    """ROS2 node for DH Robotics gripper via Dobot Modbus gateway."""

    def __init__(self):
        super().__init__('gripper_node')

        self.declare_parameter('slave_id', 1)
        self.declare_parameter('motion_timeout', 10.0)
        self.declare_parameter('init_timeout', 15.0)

        self._cb_group = ReentrantCallbackGroup()
        self._modbus_lock = threading.Lock()
        self._modbus_index = -1
        self._initialized = False

        # Cached state
        self._position = -1
        self._grip_state = -1
        self._init_status = 0

        # Modbus service clients (to C++ driver)
        srv = '/dobot_bringup_ros2/srv'
        self._modbus_create_client = self.create_client(
            ModbusCreate, f'{srv}/ModbusCreate', callback_group=self._cb_group)
        self._modbus_close_client = self.create_client(
            ModbusClose, f'{srv}/ModbusClose', callback_group=self._cb_group)
        self._set_hold_regs_client = self.create_client(
            SetHoldRegs, f'{srv}/SetHoldRegs', callback_group=self._cb_group)
        self._get_hold_regs_client = self.create_client(
            GetHoldRegs, f'{srv}/GetHoldRegs', callback_group=self._cb_group)
        self._set_tool_485_client = self.create_client(
            SetTool485, f'{srv}/SetTool485', callback_group=self._cb_group)

        # State publisher
        self._state_pub = self.create_publisher(String, '/gripper/state', 10)
        self._state_timer = self.create_timer(
            0.2, self._publish_state, callback_group=self._cb_group)  # 5Hz

        # Services
        self.create_service(
            Trigger, '/gripper/init', self._handle_init,
            callback_group=self._cb_group)
        self.create_service(
            Trigger, '/gripper/status', self._handle_status,
            callback_group=self._cb_group)

        # Action server
        self._action_server = ActionServer(
            self, Gripper, 'gripper',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._cb_group,
        )

        self.get_logger().info('Gripper node created')
        self._init_timer = self.create_timer(
            2.0, self._auto_init, callback_group=self._cb_group)

    # ── Modbus helpers ──────────────────────────────────────────
    # SetTool485 + ModbusCreate once at connect time, keep connection open.
    # All operations serialized via _modbus_lock.

    def _call_service_sync(self, client, request, timeout=5.0):
        if not client.wait_for_service(timeout_sec=timeout):
            raise TimeoutError(f'Service {client.srv_name} not available')
        future = client.call_async(request)
        deadline = time.time() + timeout
        while not future.done() and time.time() < deadline:
            time.sleep(0.01)
        if future.result() is None:
            raise RuntimeError(f'Service call failed: {client.srv_name}')
        return future.result()

    def _ensure_connected(self):
        """SetTool485 + ModbusCreate if not already connected."""
        if self._modbus_index >= 0:
            return
        # Configure RS-485 (creates port 60000 gateway)
        req = SetTool485.Request()
        req.baudrate = 115200
        req.parity = 'N'
        req.stop = 1
        req.identify = 0
        self._call_service_sync(self._set_tool_485_client, req)
        # Open Modbus connection
        req = ModbusCreate.Request()
        req.ip = '127.0.0.1'
        req.port = 60000
        req.slave_id = self.get_parameter('slave_id').value
        req.is_rtu = 1
        result = self._call_service_sync(self._modbus_create_client, req)
        self._modbus_index = 0
        if result.robot_return:
            match = re.search(r'\{(\d+)\}', result.robot_return)
            if match:
                self._modbus_index = int(match.group(1))
        self.get_logger().info(f'Modbus connected, index={self._modbus_index}')

    def _reconnect(self):
        """Close and reopen the Modbus connection."""
        if self._modbus_index >= 0:
            try:
                req = ModbusClose.Request()
                req.index = self._modbus_index
                self._call_service_sync(self._modbus_close_client, req)
            except Exception:
                pass
            self._modbus_index = -1
        self._ensure_connected()

    def _write_reg(self, addr: int, value: int):
        """Write one register."""
        req = SetHoldRegs.Request()
        req.index = self._modbus_index
        req.addr = addr
        req.count = 1
        req.val_tab = '{' + str(value) + '}'
        req.val_type = 'U16'
        result = self._call_service_sync(self._set_hold_regs_client, req)
        if result.res != 0:
            raise RuntimeError(f'SetHoldRegs failed: addr={addr} res={result.res}')

    def _read_regs(self, addr: int, count: int = 1):
        """Read registers. Returns list of ints or None."""
        req = GetHoldRegs.Request()
        req.index = self._modbus_index
        req.addr = addr
        req.count = count
        req.val_type = 'U16'
        result = self._call_service_sync(self._get_hold_regs_client, req)
        # Response format: "0,{1,1,712},GetHoldRegs(...)" — values are comma-separated inside braces
        brace_match = re.search(r'\{([^}]+)\}', result.robot_return)
        if not brace_match:
            return None
        values = [int(v.strip()) for v in brace_match.group(1).split(',')]
        return values if len(values) == count else None

    # ── Init ────────────────────────────────────────────────────

    async def _auto_init(self):
        self._init_timer.cancel()
        try:
            with self._modbus_lock:
                self._ensure_connected()
            self._activate_gripper()
            self._initialized = True
            self.get_logger().info('Gripper ready')
        except Exception as e:
            self.get_logger().error(f'Gripper auto-init failed: {e}')
            self.get_logger().info('Gripper node running (not initialized). Use /gripper/init to retry.')

    def _activate_gripper(self):
        self.get_logger().info('Activating gripper...')
        time.sleep(0.5)
        with self._modbus_lock:
            vals = self._read_regs(REG_INIT_STATUS, 1)
        status = vals[0] if vals else None
        self.get_logger().info(f'Current init status: {status}')
        if status == 1:
            self._init_status = 1
            self._initialized = True
            self.get_logger().info('Gripper already initialized')
            return
        with self._modbus_lock:
            self._write_reg(REG_INIT, INIT_VALUE)
        timeout = self.get_parameter('init_timeout').value
        start = time.time()
        while time.time() - start < timeout:
            time.sleep(1.0)
            with self._modbus_lock:
                vals = self._read_regs(REG_INIT_STATUS, 1)
            status = vals[0] if vals else None
            if status == 1:
                self._init_status = 1
                self._initialized = True
                self.get_logger().info('Gripper initialized')
                return
        raise RuntimeError('Gripper init timed out')

    # ── State publishing ────────────────────────────────────────

    def _publish_state(self):
        # Read from Modbus only when no command is in progress
        if self._modbus_index >= 0 and self._modbus_lock.acquire(blocking=False):
            try:
                vals = self._read_regs(REG_INIT_STATUS, 3)
                if vals:
                    self._init_status = vals[0]
                    if vals[0] == 1:
                        self._initialized = True
                    self._grip_state = vals[1]
                    self._position = vals[2]
            except Exception:
                pass
            finally:
                self._modbus_lock.release()

        # Always publish cached state (action server updates these during moves)
        msg = String()
        msg.data = json.dumps({
            'initialized': self._initialized,
            'position': self._position,
            'grip_state': self._grip_state,
            'grip_state_name': GRIP_STATE_NAMES.get(self._grip_state, 'unknown'),
        })
        self._state_pub.publish(msg)

    # ── Services ────────────────────────────────────────────────

    def _handle_init(self, request, response):
        try:
            with self._modbus_lock:
                self._reconnect()
            self._activate_gripper()
            response.success = True
            response.message = 'Gripper initialized'
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def _handle_status(self, request, response):
        response.success = self._initialized
        response.message = json.dumps({
            'initialized': self._initialized,
            'position': self._position,
            'grip_state': self._grip_state,
            'grip_state_name': GRIP_STATE_NAMES.get(self._grip_state, 'unknown'),
        })
        return response

    # ── Action server ───────────────────────────────────────────

    def goal_callback(self, goal_request):
        if not self._initialized:
            self.get_logger().error('Rejecting: gripper not initialized')
            return GoalResponse.REJECT
        pos = goal_request.position
        speed = goal_request.speed
        force = goal_request.force
        if pos > 1000 or speed < 1 or speed > 100 or force < 20 or force > 100:
            self.get_logger().error(f'Rejecting: invalid params pos={pos} speed={speed} force={force}')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        position = goal_handle.request.position
        speed = goal_handle.request.speed
        force = goal_handle.request.force
        result = Gripper.Result()

        # Write force, speed, position on a fresh connection (matches plugin pattern)
        try:
            with self._modbus_lock:
                self._reconnect()
                self._write_reg(REG_FORCE, force)
                self._write_reg(REG_SPEED, speed)
                self._write_reg(REG_POSITION, position)
        except Exception as e:
            self.get_logger().error(f'Write failed: {e}')
            goal_handle.abort()
            result.success = False
            result.message = str(e)
            return result

        # Poll until done
        timeout = self.get_parameter('motion_timeout').value
        feedback = Gripper.Feedback()
        start_time = time.time()

        while time.time() - start_time < timeout:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = 'Cancelled'
                return result

            try:
                with self._modbus_lock:
                    vals = self._read_regs(REG_GRIP_STATE, 2)
            except Exception:
                time.sleep(0.05)
                continue

            if vals:
                grip_state, current_pos = vals[0], vals[1]
                feedback.current_position = current_pos
                feedback.status = grip_state
                self._position = current_pos
                self._grip_state = grip_state
                goal_handle.publish_feedback(feedback)
                # Publish state so WebSocket gets live updates during moves
                msg = String()
                msg.data = json.dumps({
                    'initialized': self._initialized,
                    'position': current_pos,
                    'grip_state': grip_state,
                    'grip_state_name': GRIP_STATE_NAMES.get(grip_state, 'unknown'),
                })
                self._state_pub.publish(msg)

                if grip_state != 0:
                    goal_handle.succeed()
                    result.success = True
                    result.final_position = current_pos
                    result.status = grip_state
                    messages = {1: 'Position reached', 2: 'Object caught', 3: 'Object dropped'}
                    result.message = messages.get(grip_state, f'Stopped ({grip_state})')
                    self.get_logger().info(f'Done: {result.message} pos={current_pos}')
                    return result

            time.sleep(0.05)

        goal_handle.abort()
        result.success = False
        result.message = 'Timeout'
        return result


def main(args=None):
    rclpy.init(args=args)
    node = GripperNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        if node._modbus_index >= 0:
            try:
                req = ModbusClose.Request()
                req.index = node._modbus_index
                node._call_service_sync(node._modbus_close_client, req)
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
