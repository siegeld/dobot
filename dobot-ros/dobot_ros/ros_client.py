"""
ROS2 Service Client for Dobot CR robots.

Provides a high-level interface to dobot_bringup_v4 services.
Uses ROS2 topic subscriptions for state (joint angles, cartesian pose, robot mode)
and the gripper_node for all gripper operations.
"""

import json
import math
import re
import threading
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from dobot_actions.action import Gripper
from dobot_msgs_v4.msg import ToolVectorActual
from dobot_msgs_v4.srv import (
    EnableRobot,
    DisableRobot,
    ClearError,
    MovJ,
    RelMovLUser,
    RelMovLTool,
    SpeedFactor,
    Stop,
)


@dataclass
class Position:
    """Robot position in joint and cartesian space."""
    joint: List[float]  # 6 joint angles in degrees
    cartesian: List[float]  # [X, Y, Z, RX, RY, RZ]

    # Use uppercase to match original dobot-cr CLI
    JOINT_NAMES = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']
    CARTESIAN_NAMES = ['X', 'Y', 'Z', 'RX', 'RY', 'RZ']

    @property
    def joint_dict(self) -> dict:
        return dict(zip(self.JOINT_NAMES, self.joint))

    @property
    def cartesian_dict(self) -> dict:
        return dict(zip(self.CARTESIAN_NAMES, self.cartesian))


class DobotRosClient(Node):
    """ROS2 client for Dobot robot control via dobot_bringup_v4 services."""

    def __init__(
        self,
        namespace: str = '',
        service_timeout: float = 5.0,
        subscribe_topics: bool = False,
        managed_executor: bool = False,
    ):
        super().__init__('dobot_ros_cli')
        self._managed_executor = managed_executor

        self.namespace = namespace.rstrip('/')
        self.service_timeout = service_timeout
        self._jog_mode = 'user'

        # Build service prefix
        prefix = f'{self.namespace}/' if self.namespace else ''
        self._srv_prefix = f'{prefix}dobot_bringup_ros2/srv/'

        # ── Topic subscriptions for state data ──────────────────
        # Only enabled for long-lived clients (web dashboard).
        # CLI clients use service calls and don't need subscriptions.
        self._state_lock = threading.Lock()
        self._joint_angles = None  # degrees
        self._joint_stamp = 0.0    # header.stamp from last joint msg
        self._cartesian_pose = None  # [X, Y, Z, RX, RY, RZ]
        self._robot_mode = -1
        self._gripper_state = {
            'initialized': False, 'position': -1,
            'grip_state': -1, 'grip_state_name': 'unknown',
        }

        if subscribe_topics:
            self.create_subscription(
                JointState, '/joint_states_robot',
                self._joint_state_cb, 10)
            self.create_subscription(
                ToolVectorActual, '/dobot_msgs_v4/msg/ToolVectorActual',
                self._tool_vector_cb, 10)
            self.create_subscription(
                String, '/dobot_bringup_ros2/msg/FeedInfo',
                self._feed_info_cb, 10)
            self.create_subscription(
                String, '/gripper/state',
                self._gripper_state_cb, 10)

        # ── Service clients for robot commands ──────────────────
        self._enable_client = self.create_client(
            EnableRobot, f'{self._srv_prefix}EnableRobot')
        self._disable_client = self.create_client(
            DisableRobot, f'{self._srv_prefix}DisableRobot')
        self._clear_error_client = self.create_client(
            ClearError, f'{self._srv_prefix}ClearError')
        self._movj_client = self.create_client(
            MovJ, f'{self._srv_prefix}MovJ')
        self._rel_mov_user_client = self.create_client(
            RelMovLUser, f'{self._srv_prefix}RelMovLUser')
        self._rel_mov_tool_client = self.create_client(
            RelMovLTool, f'{self._srv_prefix}RelMovLTool')
        self._speed_factor_client = self.create_client(
            SpeedFactor, f'{self._srv_prefix}SpeedFactor')
        self._stop_client = self.create_client(
            Stop, f'{self._srv_prefix}Stop')

        # ── Gripper node clients ────────────────────────────────
        self._gripper_init_client = self.create_client(
            Trigger, '/gripper/init')
        self._gripper_action_client = ActionClient(
            self, Gripper, 'gripper')

    # ── Topic callbacks ──────────────────────────────────────────

    def _joint_state_cb(self, msg: JointState):
        with self._state_lock:
            self._joint_angles = [math.degrees(r) for r in msg.position]
            self._joint_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self._joint_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _tool_vector_cb(self, msg: ToolVectorActual):
        with self._state_lock:
            self._cartesian_pose = [msg.x, msg.y, msg.z, msg.rx, msg.ry, msg.rz]

    def _feed_info_cb(self, msg):
        try:
            d = json.loads(msg.data)
            with self._state_lock:
                self._robot_mode = d.get('robot_mode', -1)
        except Exception:
            pass

    def _gripper_state_cb(self, msg):
        try:
            with self._state_lock:
                self._gripper_state = json.loads(msg.data)
        except Exception:
            pass

    # ── Service helpers ────────────────────────────────────────

    def _wait_for_service(self, client, timeout: float = None) -> bool:
        """Wait for a service to become available."""
        timeout = timeout or self.service_timeout
        return client.wait_for_service(timeout_sec=timeout)

    def _wait_for_future(self, future, timeout: float = None):
        """Wait for an rclpy Future to complete.

        When managed_executor=True, an external executor is spinning this node,
        so we wait via threading.Event instead of calling spin_until_future_complete
        (which would conflict by trying to add the node to a second executor).
        """
        timeout = timeout or self.service_timeout
        if self._managed_executor:
            event = threading.Event()
            future.add_done_callback(lambda _: event.set())
            if not event.wait(timeout=timeout):
                return None
            return future.result()
        else:
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            return future.result()

    def _call_service(self, client, request):
        """Call a service and wait for response."""
        if not self._wait_for_service(client):
            raise TimeoutError(f"Service {client.srv_name} not available")

        future = client.call_async(request)
        result = self._wait_for_future(future)

        if result is None:
            raise RuntimeError(f"Service call failed: {client.srv_name}")

        return result

    # ── Robot state (from topics) ──────────────────────────────

    @property
    def jog_mode(self) -> str:
        """Current jog coordinate mode."""
        return self._jog_mode

    def set_jog_mode(self, mode: str) -> None:
        """Set jog coordinate mode ('user' or 'tool')."""
        if mode not in ('user', 'tool'):
            raise ValueError("Mode must be 'user' or 'tool'")
        self._jog_mode = mode

    def check_connection(self) -> bool:
        """Check if dobot_bringup_v4 services are available."""
        return self._wait_for_service(self._enable_client)

    def is_feedback_stale(self, max_age: float = 5.0) -> bool:
        """True if joint state timestamps have stopped advancing (feedback port dead)."""
        with self._state_lock:
            if self._joint_stamp == 0.0:
                return False  # No data yet, not stale
            now = self.get_clock().now().nanoseconds * 1e-9
            return (now - self._joint_stamp) > max_age

    def get_robot_mode(self) -> int:
        """Get current robot mode (from FeedInfo topic)."""
        with self._state_lock:
            return self._robot_mode

    def get_joint_angles(self) -> List[float]:
        """Get current joint angles in degrees (from /joint_states_robot topic)."""
        with self._state_lock:
            if self._joint_angles is None:
                raise RuntimeError("No joint data received yet")
            return list(self._joint_angles)

    def get_cartesian_pose(self) -> List[float]:
        """Get current cartesian pose (from ToolVectorActual topic)."""
        with self._state_lock:
            if self._cartesian_pose is None:
                raise RuntimeError("No cartesian data received yet")
            return list(self._cartesian_pose)

    def get_position(self) -> Position:
        """Get current robot position (joint and cartesian)."""
        joint = self.get_joint_angles()
        cartesian = self.get_cartesian_pose()
        return Position(joint=joint, cartesian=cartesian)

    # ── Robot commands ─────────────────────────────────────────

    def enable_robot(self) -> int:
        """Enable the robot."""
        request = EnableRobot.Request()
        response = self._call_service(self._enable_client, request)
        return response.res

    def disable_robot(self) -> int:
        """Disable the robot."""
        request = DisableRobot.Request()
        response = self._call_service(self._disable_client, request)
        return response.res

    def clear_error(self) -> int:
        """Clear robot errors."""
        request = ClearError.Request()
        response = self._call_service(self._clear_error_client, request)
        return response.res

    def stop(self) -> int:
        """Stop robot motion."""
        request = Stop.Request()
        response = self._call_service(self._stop_client, request)
        return response.res

    def set_speed_factor(self, speed: int) -> int:
        """Set global speed factor (1-100)."""
        speed = max(1, min(100, speed))
        request = SpeedFactor.Request()
        request.ratio = speed
        response = self._call_service(self._speed_factor_client, request)
        return response.res

    def move_joints(self, angles: List[float], wait: bool = False, tolerance: float = 0.5) -> int:
        """Move to absolute joint angles."""
        if len(angles) != 6:
            raise ValueError("Must provide exactly 6 joint angles")

        request = MovJ.Request()
        request.mode = True  # Joint mode
        request.a = float(angles[0])
        request.b = float(angles[1])
        request.c = float(angles[2])
        request.d = float(angles[3])
        request.e = float(angles[4])
        request.f = float(angles[5])
        request.param_value = []

        response = self._call_service(self._movj_client, request)

        if wait:
            self.wait_for_motion(angles, tolerance)

        return response.res

    def wait_for_motion(
        self,
        target: List[float],
        tolerance: float = 0.5,
        timeout: float = 30.0,
        feedback_callback=None,
    ) -> bool:
        """Wait for robot to reach target joint position."""
        import time
        start_time = time.time()

        while time.time() - start_time < timeout:
            current = self.get_joint_angles()
            max_error = max(abs(current[i] - target[i]) for i in range(6))

            if feedback_callback:
                feedback_callback(current, max_error)

            if max_error <= tolerance:
                return True
            time.sleep(0.1)

        return False

    def wait_for_cartesian_motion(
        self,
        target: List[float],
        tolerance: float = 1.0,
        timeout: float = 30.0,
        feedback_callback=None,
    ) -> bool:
        """Wait for robot to reach target cartesian position."""
        import time
        import math
        start_time = time.time()

        while time.time() - start_time < timeout:
            current = self.get_cartesian_pose()
            distance = math.sqrt(
                (current[0] - target[0])**2 +
                (current[1] - target[1])**2 +
                (current[2] - target[2])**2
            )

            if feedback_callback:
                feedback_callback(current, distance)

            if distance <= tolerance:
                return True
            time.sleep(0.1)

        return False

    def jog_joint(
        self,
        joint: int,
        offset: float,
        wait: bool = False,
        tolerance: float = 0.5,
        timeout: float = 30.0,
    ) -> int:
        """Jog a single joint by offset."""
        if joint < 1 or joint > 6:
            raise ValueError("Joint must be 1-6")

        current = self.get_joint_angles()
        target = list(current)
        target[joint - 1] += offset

        return self.move_joints(target, wait=wait, tolerance=tolerance)

    def jog(
        self,
        x: float = 0, y: float = 0, z: float = 0,
        rx: float = 0, ry: float = 0, rz: float = 0,
        wait: bool = False,
        tolerance: float = 1.0,
        timeout: float = 30.0,
    ) -> int:
        """Jog in cartesian space."""
        if wait:
            current_pose = self.get_cartesian_pose()
            target = [
                current_pose[0] + x,
                current_pose[1] + y,
                current_pose[2] + z,
                current_pose[3] + rx,
                current_pose[4] + ry,
                current_pose[5] + rz,
            ]

        if self._jog_mode == 'tool':
            request = RelMovLTool.Request()
        else:
            request = RelMovLUser.Request()

        request.a = float(x)
        request.b = float(y)
        request.c = float(z)
        request.d = float(rx)
        request.e = float(ry)
        request.f = float(rz)
        request.param_value = []

        if self._jog_mode == 'tool':
            response = self._call_service(self._rel_mov_tool_client, request)
        else:
            response = self._call_service(self._rel_mov_user_client, request)

        if wait:
            self.wait_for_cartesian_motion(target, tolerance, timeout)

        return response.res

    # ── Gripper API (via gripper_node) ─────────────────────────

    def gripper_init(self, timeout: float = 15.0):
        """Initialize gripper via /gripper/init service."""
        result = self._call_service(self._gripper_init_client, Trigger.Request())
        if not result.success:
            raise RuntimeError(f'Gripper init failed: {result.message}')

    def gripper_move(
        self,
        position: int,
        force: int = 50,
        speed: int = 50,
        wait: bool = True,
        timeout: float = 10.0,
    ) -> int:
        """Move gripper via /gripper action."""
        if not self._gripper_action_client.wait_for_server(timeout_sec=self.service_timeout):
            raise TimeoutError('Gripper action server not available')

        goal = Gripper.Goal()
        goal.position = position
        goal.speed = speed
        goal.force = force
        future = self._gripper_action_client.send_goal_async(goal)
        goal_handle = self._wait_for_future(future)
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError('Gripper goal rejected')

        if not wait:
            return 0

        result_future = goal_handle.get_result_async()
        result = self._wait_for_future(result_future, timeout=timeout)
        if result is None:
            return -1
        return result.result.status

    def gripper_open(self, force: int = 50, speed: int = 50, wait: bool = True) -> int:
        """Open gripper fully."""
        return self.gripper_move(1000, force=force, speed=speed, wait=wait)

    def gripper_close(self, force: int = 50, speed: int = 50, wait: bool = True) -> int:
        """Close gripper fully."""
        return self.gripper_move(0, force=force, speed=speed, wait=wait)

    def gripper_get_position(self) -> Optional[int]:
        """Get cached gripper position from /gripper/state topic."""
        with self._state_lock:
            return self._gripper_state.get('position', -1)

    def gripper_get_state(self) -> Optional[int]:
        """Get cached grip state from /gripper/state topic."""
        with self._state_lock:
            return self._gripper_state.get('grip_state', -1)

    def gripper_get_init_status(self) -> Optional[int]:
        """Get cached init status from /gripper/state topic."""
        with self._state_lock:
            return 1 if self._gripper_state.get('initialized') else 0

    def shutdown(self):
        """Shutdown the ROS2 client."""
        self.destroy_node()
