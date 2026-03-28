#!/usr/bin/env python3
"""
ROS2 Action Server for DH Robotics gripper via Dobot internal Modbus TCP.

The gripper communicates through the Dobot controller's internal Modbus TCP
gateway at 127.0.0.1:60000 (NOT ModbusRTUCreate).

Register map:
    Write (SetHoldRegs):
        0x0100 (256)  Init        0xA5 (165) = full init
        0x0101 (257)  Force       20-100 %
        0x0103 (259)  Position    0 (closed) to 1000 (open)
        0x0104 (260)  Speed       1-100 %

    Read (GetHoldRegs):
        0x0200 (512)  Init status   0=not init, 1=initialized
        0x0201 (513)  Grip state    0=moving, 1=reached, 2=caught, 3=dropped
        0x0202 (514)  Current pos   0-1000
"""

import time
import re

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from dobot_actions.action import Gripper
from dobot_msgs_v4.srv import (
    ModbusCreate,
    ModbusClose,
    SetHoldRegs,
    GetHoldRegs,
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

INIT_VALUE = 165  # 0xA5 = full initialization


class GripperServer(Node):
    """Action server for DH Robotics gripper via Dobot internal Modbus TCP."""

    def __init__(self):
        super().__init__('gripper_server')

        self.declare_parameter('slave_id', 1)
        self.declare_parameter('motion_timeout', 10.0)
        self.declare_parameter('init_timeout', 15.0)

        self._cb_group = ReentrantCallbackGroup()
        self._initialized = False
        self._modbus_index = -1

        srv = '/dobot_bringup_ros2/srv'
        self._modbus_create_client = self.create_client(
            ModbusCreate, f'{srv}/ModbusCreate', callback_group=self._cb_group)
        self._modbus_close_client = self.create_client(
            ModbusClose, f'{srv}/ModbusClose', callback_group=self._cb_group)
        self._set_hold_regs_client = self.create_client(
            SetHoldRegs, f'{srv}/SetHoldRegs', callback_group=self._cb_group)
        self._get_hold_regs_client = self.create_client(
            GetHoldRegs, f'{srv}/GetHoldRegs', callback_group=self._cb_group)

        self._action_server = ActionServer(
            self,
            Gripper,
            'gripper',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._cb_group,
        )

        self.get_logger().info('Gripper action server created')
        self._init_timer = self.create_timer(
            2.0, self._auto_init, callback_group=self._cb_group)

    async def _auto_init(self):
        """Connect and activate gripper on startup."""
        self._init_timer.cancel()
        try:
            self._create_modbus()
            self._activate_gripper()
            self._initialized = True
            self.get_logger().info('Gripper ready')
        except Exception as e:
            self.get_logger().error(f'Gripper init failed: {e}')

    def _call_service_sync(self, client, request, timeout=5.0):
        if not client.wait_for_service(timeout_sec=timeout):
            raise TimeoutError(f'Service {client.srv_name} not available')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if future.result() is None:
            raise RuntimeError(f'Service call failed: {client.srv_name}')
        return future.result()

    def _create_modbus(self):
        """Create Modbus TCP connection to internal gateway at 127.0.0.1:60000."""
        req = ModbusCreate.Request()
        req.ip = '127.0.0.1'
        req.port = 60000
        req.slave_id = self.get_parameter('slave_id').value
        req.is_rtu = 1
        result = self._call_service_sync(self._modbus_create_client, req)
        match = re.search(r'\{(\d+)\}', result.robot_return)
        if match:
            self._modbus_index = int(match.group(1))
        else:
            self._modbus_index = 0
        self.get_logger().info(f'Modbus TCP connected, index={self._modbus_index}')

    def _activate_gripper(self):
        """Send init (0xA5) and wait until initialized."""
        self.get_logger().info('Activating gripper...')
        self._write_reg(REG_INIT, INIT_VALUE)
        timeout = self.get_parameter('init_timeout').value
        start = time.time()
        while time.time() - start < timeout:
            status = self._read_reg(REG_INIT_STATUS)
            if status == 1:
                self.get_logger().info('Gripper initialized')
                return
            time.sleep(0.5)
        raise RuntimeError('Gripper init timed out')

    def _write_reg(self, addr: int, value: int):
        req = SetHoldRegs.Request()
        req.index = self._modbus_index
        req.addr = addr
        req.count = 1
        req.val_tab = '{' + str(value) + '}'
        req.val_type = 'U16'
        result = self._call_service_sync(self._set_hold_regs_client, req)
        if result.res != 0:
            raise RuntimeError(f'SetHoldRegs failed: addr={addr} res={result.res}')

    def _read_reg(self, addr: int):
        req = GetHoldRegs.Request()
        req.index = self._modbus_index
        req.addr = addr
        req.count = 1
        req.val_type = 'U16'
        result = self._call_service_sync(self._get_hold_regs_client, req)
        match = re.search(r'\{(\d+)\}', result.robot_return)
        if match:
            return int(match.group(1))
        return None

    def goal_callback(self, goal_request):
        if not self._initialized:
            self.get_logger().error('Rejecting: gripper not initialized')
            return GoalResponse.REJECT
        pos = goal_request.position
        speed = goal_request.speed
        force = goal_request.force
        if pos > 1000:
            self.get_logger().error(f'Rejecting: position {pos} > 1000')
            return GoalResponse.REJECT
        if speed < 1 or speed > 100:
            self.get_logger().error(f'Rejecting: speed {speed} not in 1-100')
            return GoalResponse.REJECT
        if force < 20 or force > 100:
            self.get_logger().error(f'Rejecting: force {force} not in 20-100')
            return GoalResponse.REJECT
        self.get_logger().info(f'Goal: pos={pos} speed={speed} force={force}')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        position = goal_handle.request.position
        speed = goal_handle.request.speed
        force = goal_handle.request.force
        result = Gripper.Result()

        try:
            self._write_reg(REG_FORCE, force)
            self._write_reg(REG_SPEED, speed)
            self._write_reg(REG_POSITION, position)
        except Exception as e:
            self.get_logger().error(f'Write failed: {e}')
            goal_handle.abort()
            result.success = False
            result.message = str(e)
            return result

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
                current_pos = self._read_reg(REG_CURRENT_POS)
                grip_state = self._read_reg(REG_GRIP_STATE)
            except Exception:
                await self._sleep(0.1)
                continue

            if current_pos is not None:
                feedback.current_position = current_pos
            if grip_state is not None:
                feedback.status = grip_state
            goal_handle.publish_feedback(feedback)

            if grip_state is not None and grip_state != 0:
                goal_handle.succeed()
                result.success = True
                result.final_position = current_pos or 0
                result.status = grip_state
                messages = {1: 'Position reached', 2: 'Object caught', 3: 'Object dropped'}
                result.message = messages.get(grip_state, f'Stopped ({grip_state})')
                self.get_logger().info(f'Done: {result.message} pos={current_pos}')
                return result

            await self._sleep(0.05)

        goal_handle.abort()
        result.success = False
        result.message = 'Timeout'
        try:
            result.final_position = self._read_reg(REG_CURRENT_POS) or 0
            result.status = self._read_reg(REG_GRIP_STATE) or 0
        except Exception:
            pass
        return result

    async def _sleep(self, duration):
        import asyncio
        await asyncio.sleep(duration)


def main(args=None):
    rclpy.init(args=args)
    node = GripperServer()
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
