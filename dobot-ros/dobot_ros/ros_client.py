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
    InverseKin,
    MovJ,
    RelMovLUser,
    RelMovLTool,
    ServoP,
    ServoJ,
    SpeedFactor,
    StartDrag,
    StopDrag,
    Stop,
    Tool,
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
        self._start_drag_client = self.create_client(
            StartDrag, f'{self._srv_prefix}StartDrag')
        self._stop_drag_client = self.create_client(
            StopDrag, f'{self._srv_prefix}StopDrag')
        self._inverse_kin_client = self.create_client(
            InverseKin, f'{self._srv_prefix}InverseKin')
        self._servo_p_client = self.create_client(
            ServoP, f'{self._srv_prefix}ServoP')
        self._servo_j_client = self.create_client(
            ServoJ, f'{self._srv_prefix}ServoJ')
        self._tool_client = self.create_client(
            Tool, f'{self._srv_prefix}Tool')

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

    # ── Value validation (defense-in-depth for robot safety) ────
    # Delegated to dobot_ros.validation (no ROS deps, testable standalone).

    @staticmethod
    def _validate_finite(values: List[float], label: str = "values"):
        from dobot_ros.validation import validate_finite
        validate_finite(values, label)

    @staticmethod
    def _validate_pose_bounds(pose: List[float]):
        from dobot_ros.validation import validate_pose_bounds
        validate_pose_bounds(pose)

    @staticmethod
    def _validate_joint_bounds(joints: List[float]):
        from dobot_ros.validation import validate_joint_bounds
        validate_joint_bounds(joints)

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
        """Get current robot position (joint and cartesian) from a single lock acquisition."""
        with self._state_lock:
            if self._joint_angles is None:
                raise RuntimeError("No joint data received yet")
            if self._cartesian_pose is None:
                raise RuntimeError("No cartesian data received yet")
            return Position(joint=list(self._joint_angles), cartesian=list(self._cartesian_pose))

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

    def start_drag(self) -> int:
        """Enter drag/freedrive mode."""
        request = StartDrag.Request()
        response = self._call_service(self._start_drag_client, request)
        return response.res

    def stop_drag(self) -> int:
        """Exit drag/freedrive mode."""
        request = StopDrag.Request()
        response = self._call_service(self._stop_drag_client, request)
        return response.res

    def set_speed_factor(self, speed: int) -> int:
        """Set global speed factor (1-100)."""
        speed = max(1, min(100, speed))
        request = SpeedFactor.Request()
        request.ratio = speed
        response = self._call_service(self._speed_factor_client, request)
        return response.res

    def set_tool(self, index: int) -> int:
        """Activate a tool coordinate system (pre-configured on the robot).

        After Tool(0) the robot's reported cartesian pose is the **flange /
        wrist** in base frame. After Tool(N) (N ≥ 1, N must exist on the
        robot controller) the reported pose is the origin of that tool's
        defined frame — for the AG-105 with tool 1 set to {0,0,203,0,0,0}
        that's the fingertip.

        Raises TimeoutError if the Tool service is unavailable, RuntimeError
        on a non-zero response code. Caller should wrap + fall back to 0 if
        workspace-protection errors occur during the switch.
        """
        request = Tool.Request()
        request.index = int(index)
        response = self._call_service(self._tool_client, request)
        return response.res

    def inverse_kin(self, pose: List[float]) -> List[float]:
        """Compute inverse kinematics for a cartesian pose without moving.

        Takes [X, Y, Z, RX, RY, RZ] (mm, degrees).
        Returns [J1, J2, J3, J4, J5, J6] (degrees).
        The robot solves IK internally using its current joint angles to
        select the nearest solution. Pure query — no motion.
        """
        if len(pose) != 6:
            raise ValueError("Must provide [X, Y, Z, RX, RY, RZ]")
        request = InverseKin.Request()
        request.x = float(pose[0])
        request.y = float(pose[1])
        request.z = float(pose[2])
        request.rx = float(pose[3])
        request.ry = float(pose[4])
        request.rz = float(pose[5])
        request.use_joint_near = ""
        request.joint_near = ""
        request.user = ""
        request.tool = ""
        response = self._call_service(self._inverse_kin_client, request)

        # robot_return is a comma-separated string like "{j1,j2,j3,j4,j5,j6}"
        # res=0 means success; non-zero means the robot rejected the query.
        import re
        ret = response.robot_return or ''
        if response.res != 0 or not ret:
            raise RuntimeError(
                f"InverseKin failed: res={response.res}, return={ret!r}. "
                f"Robot may need to be enabled (mode 5) before IK queries work."
            )
        match = re.search(r'\{([^}]+)\}', ret)
        if not match:
            raise RuntimeError(f"InverseKin returned unexpected format: {ret!r}")
        angles = [float(v) for v in match.group(1).split(',')]
        if len(angles) != 6:
            raise RuntimeError(f"InverseKin returned {len(angles)} values, expected 6")
        return angles

    def servo_p(
        self,
        pose: List[float],
        t: float = 0.1,
        aheadtime: float = 50.0,
        gain: float = 500.0,
    ) -> int:
        """Stream an absolute cartesian pose via ServoP for high-rate servo control.

        Unlike MovJ/MovL (point-to-point with trajectory planning), ServoP is a
        continuous-servo interface: call it repeatedly with updated targets at
        20-100+ Hz for smooth streaming motion. Used by the VLA executor.

        t: intended interpolation time to reach this pose (s), range [0.004, 3600].
        aheadtime: D-term of internal PID, range [20, 100]. Default 50.
        gain: P-term of internal PID, range [200, 1000]. Default 500.

        ServoP expects monotonic streaming — pausing halts the controller.
        """
        if len(pose) != 6:
            raise ValueError("Must provide [X, Y, Z, RX, RY, RZ]")
        self._validate_finite(pose, "ServoP pose")
        self._validate_pose_bounds(pose)
        t = max(0.004, min(3600.0, t))  # clamp to Dobot spec range
        request = ServoP.Request()
        request.a = float(pose[0])
        request.b = float(pose[1])
        request.c = float(pose[2])
        request.d = float(pose[3])
        request.e = float(pose[4])
        request.f = float(pose[5])
        # Optional tuning params are passed as key=value strings per the
        # Dobot TCP-IP protocol convention.
        request.param_value = [
            f"t={t:f}",
            f"aheadtime={aheadtime:f}",
            f"gain={gain:f}",
        ]
        response = self._call_service(self._servo_p_client, request)
        return response.res

    def servo_j(
        self,
        joints: List[float],
        t: float = 0.1,
        aheadtime: float = 50.0,
        gain: float = 500.0,
    ) -> int:
        """Stream absolute joint angles via ServoJ (joint-space analog of servo_p)."""
        if len(joints) != 6:
            raise ValueError("Must provide exactly 6 joint angles")
        self._validate_finite(joints, "ServoJ joints")
        self._validate_joint_bounds(joints)
        t = max(0.004, min(3600.0, t))
        request = ServoJ.Request()
        request.a = float(joints[0])
        request.b = float(joints[1])
        request.c = float(joints[2])
        request.d = float(joints[3])
        request.e = float(joints[4])
        request.f = float(joints[5])
        request.param_value = [
            f"t={t:f}",
            f"aheadtime={aheadtime:f}",
            f"gain={gain:f}",
        ]
        response = self._call_service(self._servo_j_client, request)
        return response.res

    def move_pose(self, pose: List[float]) -> int:
        """Move to cartesian pose [X, Y, Z, RX, RY, RZ] using MovJ."""
        if len(pose) != 6:
            raise ValueError("Must provide [X, Y, Z, RX, RY, RZ]")
        self._validate_finite(pose, "MovJ pose")
        self._validate_pose_bounds(pose)
        request = MovJ.Request()
        request.mode = False  # Cartesian mode
        request.a = float(pose[0])
        request.b = float(pose[1])
        request.c = float(pose[2])
        request.d = float(pose[3])
        request.e = float(pose[4])
        request.f = float(pose[5])
        request.param_value = []
        response = self._call_service(self._movj_client, request)
        return response.res

    def move_joints(self, angles: List[float], wait: bool = False, tolerance: float = 0.5) -> int:
        """Move to absolute joint angles."""
        if len(angles) != 6:
            raise ValueError("Must provide exactly 6 joint angles")
        self._validate_finite(angles, "MovJ joints")
        self._validate_joint_bounds(angles)

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
        angle_tolerance: float = 3.0,
        timeout: float = 30.0,
        feedback_callback=None,
    ) -> bool:
        """Wait for robot to reach target cartesian position AND orientation.

        tolerance: XYZ Euclidean distance in mm.
        angle_tolerance: max angular error in degrees for RX/RY/RZ.
        """
        import time
        import math
        start_time = time.time()

        while time.time() - start_time < timeout:
            try:
                current = self.get_cartesian_pose()
            except RuntimeError:
                # Pose data lost (topic dropped, executor died). Halt robot
                # and report failure — don't let motion continue unsupervised.
                try:
                    self.stop()
                except Exception:
                    pass
                return False
            distance = math.sqrt(
                (current[0] - target[0])**2 +
                (current[1] - target[1])**2 +
                (current[2] - target[2])**2
            )

            # Check orientation convergence if target has 6 elements.
            angle_ok = True
            if len(target) >= 6 and len(current) >= 6:
                for i in range(3, 6):
                    # Normalize both to [-180, 180] then compare.
                    c = ((current[i] + 180) % 360) - 180
                    t = ((target[i] + 180) % 360) - 180
                    diff = abs(c - t)
                    diff = min(diff, 360.0 - diff)
                    if diff > angle_tolerance:
                        angle_ok = False
                        break

            if feedback_callback:
                feedback_callback(current, distance)

            if distance <= tolerance and angle_ok:
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
        offsets = [float(x), float(y), float(z), float(rx), float(ry), float(rz)]
        self._validate_finite(offsets, "jog offsets")
        # Cap single-jog magnitude to prevent gross errors from UI bugs.
        MAX_JOG_LINEAR = 500.0   # mm
        MAX_JOG_ROT = 180.0      # deg
        for i in range(3):
            if abs(offsets[i]) > MAX_JOG_LINEAR:
                raise ValueError(f"Jog offset {['X','Y','Z'][i]}={offsets[i]:.1f}mm exceeds ±{MAX_JOG_LINEAR}mm")
        for i in range(3, 6):
            if abs(offsets[i]) > MAX_JOG_ROT:
                raise ValueError(f"Jog offset {['RX','RY','RZ'][i-3]}={offsets[i]:.1f}° exceeds ±{MAX_JOG_ROT}°")
        x, y, z, rx, ry, rz = offsets

        # Capture current pose for wait target. For user-frame jog, the
        # target is current + offset. For tool-frame jog, the target in the
        # user frame is unpredictable without FK — so we wait for the robot
        # mode to leave RUNNING instead of checking position convergence.
        target = None
        if wait and self._jog_mode == 'user':
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
            if target is not None:
                self.wait_for_cartesian_motion(target, tolerance, timeout)
            else:
                # Tool-frame: can't predict user-frame target, so wait for
                # robot mode to leave RUNNING (mode 7).
                import time as _time
                start = _time.time()
                _time.sleep(0.3)  # let the move start
                while _time.time() - start < timeout:
                    mode = self.get_robot_mode()
                    if mode != 7:  # 7 = RUNNING
                        break
                    _time.sleep(0.1)

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
        # NOTE: position is clamped to [0, 1000] below for safety.
        wait: bool = True,
        timeout: float = 10.0,
    ) -> int:
        """Move gripper via /gripper action."""
        position = max(0, min(1000, int(position)))
        force = max(20, min(100, int(force)))
        speed = max(1, min(100, int(speed)))

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
