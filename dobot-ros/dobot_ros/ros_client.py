"""
ROS2 Service Client for Dobot CR robots.

Provides a high-level interface to dobot_bringup_v4 services.
"""

import re
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node

# Import dobot_msgs_v4 service types
from dobot_msgs_v4.srv import (
    EnableRobot,
    DisableRobot,
    ClearError,
    GetAngle,
    GetPose,
    MovJ,
    RelMovLUser,
    RelMovLTool,
    SpeedFactor,
    RobotMode,
    Stop,
    ModbusCreate,
    ModbusClose,
    SetHoldRegs,
    GetHoldRegs,
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
    ):
        """
        Initialize the ROS2 client.

        Args:
            namespace: ROS2 namespace for dobot_bringup_v4 services
            service_timeout: Timeout for service calls in seconds
        """
        super().__init__('dobot_ros_cli')

        self.namespace = namespace.rstrip('/')
        self.service_timeout = service_timeout
        self._jog_mode = 'user'

        # Build service prefix
        # Note: The ROS2 package uses 'dobot_bringup_ros2' not 'dobot_bringup_v4'
        prefix = f'{self.namespace}/' if self.namespace else ''
        self._srv_prefix = f'{prefix}dobot_bringup_ros2/srv/'

        # Create service clients
        self._enable_client = self.create_client(
            EnableRobot, f'{self._srv_prefix}EnableRobot')
        self._disable_client = self.create_client(
            DisableRobot, f'{self._srv_prefix}DisableRobot')
        self._clear_error_client = self.create_client(
            ClearError, f'{self._srv_prefix}ClearError')
        self._get_angle_client = self.create_client(
            GetAngle, f'{self._srv_prefix}GetAngle')
        self._get_pose_client = self.create_client(
            GetPose, f'{self._srv_prefix}GetPose')
        self._movj_client = self.create_client(
            MovJ, f'{self._srv_prefix}MovJ')
        self._rel_mov_user_client = self.create_client(
            RelMovLUser, f'{self._srv_prefix}RelMovLUser')
        self._rel_mov_tool_client = self.create_client(
            RelMovLTool, f'{self._srv_prefix}RelMovLTool')
        self._speed_factor_client = self.create_client(
            SpeedFactor, f'{self._srv_prefix}SpeedFactor')
        self._robot_mode_client = self.create_client(
            RobotMode, f'{self._srv_prefix}RobotMode')
        self._stop_client = self.create_client(
            Stop, f'{self._srv_prefix}Stop')

        # Gripper (Modbus) service clients
        self._modbus_create_client = self.create_client(
            ModbusCreate, f'{self._srv_prefix}ModbusCreate')
        self._modbus_close_client = self.create_client(
            ModbusClose, f'{self._srv_prefix}ModbusClose')
        self._set_hold_regs_client = self.create_client(
            SetHoldRegs, f'{self._srv_prefix}SetHoldRegs')
        self._get_hold_regs_client = self.create_client(
            GetHoldRegs, f'{self._srv_prefix}GetHoldRegs')

        # Gripper state
        self._gripper_modbus_index = -1
        self._gripper_initialized = False

    def _wait_for_service(self, client, timeout: float = None) -> bool:
        """Wait for a service to become available."""
        timeout = timeout or self.service_timeout
        return client.wait_for_service(timeout_sec=timeout)

    def _call_service(self, client, request):
        """Call a service and wait for response."""
        if not self._wait_for_service(client):
            raise TimeoutError(f"Service {client.srv_name} not available")

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.service_timeout)

        if future.result() is None:
            raise RuntimeError(f"Service call failed: {client.srv_name}")

        return future.result()

    def _parse_angle_response(self, response_str: str) -> List[float]:
        """Parse joint angles from GetAngle response string."""
        # Response format: "{j1,j2,j3,j4,j5,j6}"
        match = re.search(r'\{([^}]+)\}', response_str)
        if match:
            values = match.group(1).split(',')
            return [float(v.strip()) for v in values]
        raise ValueError(f"Could not parse angle response: {response_str}")

    def _parse_pose_response(self, response_str: str) -> List[float]:
        """Parse cartesian pose from GetPose response string."""
        # Response format: "{x,y,z,rx,ry,rz}"
        match = re.search(r'\{([^}]+)\}', response_str)
        if match:
            values = match.group(1).split(',')
            return [float(v.strip()) for v in values]
        raise ValueError(f"Could not parse pose response: {response_str}")

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
        return self._wait_for_service(self._get_angle_client)

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

    def get_robot_mode(self) -> int:
        """Get current robot mode."""
        request = RobotMode.Request()
        response = self._call_service(self._robot_mode_client, request)
        return response.res

    def get_joint_angles(self) -> List[float]:
        """Get current joint angles in degrees."""
        request = GetAngle.Request()
        response = self._call_service(self._get_angle_client, request)
        return self._parse_angle_response(response.robot_return)

    def get_cartesian_pose(self) -> List[float]:
        """Get current cartesian pose [X, Y, Z, RX, RY, RZ]."""
        request = GetPose.Request()
        request.user = 0
        request.tool = 0
        response = self._call_service(self._get_pose_client, request)
        return self._parse_pose_response(response.robot_return)

    def get_position(self) -> Position:
        """Get current robot position (joint and cartesian)."""
        joint = self.get_joint_angles()
        cartesian = self.get_cartesian_pose()
        return Position(joint=joint, cartesian=cartesian)

    def move_joints(self, angles: List[float], wait: bool = False, tolerance: float = 0.5) -> int:
        """
        Move to absolute joint angles.

        Args:
            angles: List of 6 joint angles in degrees
            wait: If True, block until motion completes
            tolerance: Position tolerance in degrees for wait mode
        """
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
        """
        Wait for robot to reach target joint position.

        Note: The Dobot ROS2 driver doesn't provide ROS2 Actions for motion,
        so we poll GetAngle until the target is reached. For a proper ROS2
        action interface, use the dobot_actions/move_joints action server.

        Args:
            target: Target joint angles in degrees
            tolerance: Position tolerance in degrees
            timeout: Maximum wait time in seconds
            feedback_callback: Optional callback(current, max_error) for progress

        Returns:
            True if target reached, False if timeout
        """
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
        """
        Wait for robot to reach target cartesian position.

        Args:
            target: Target cartesian pose [X, Y, Z, RX, RY, RZ]
            tolerance: Position tolerance in mm (for XYZ)
            timeout: Maximum wait time in seconds
            feedback_callback: Optional callback(current, distance) for progress

        Returns:
            True if target reached, False if timeout
        """
        import time
        import math
        start_time = time.time()

        while time.time() - start_time < timeout:
            current = self.get_cartesian_pose()
            # Distance in XYZ only
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
        """
        Jog a single joint by offset.

        Args:
            joint: Joint number (1-6)
            offset: Angle offset in degrees
            wait: If True, block until motion completes
            tolerance: Position tolerance in degrees for wait mode
            timeout: Maximum wait time in seconds
        """
        if joint < 1 or joint > 6:
            raise ValueError("Joint must be 1-6")

        # Get current angles and add offset
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
        """
        Jog in cartesian space.

        Args:
            x, y, z: Linear offsets in mm
            rx, ry, rz: Rotation offsets in degrees
            wait: If True, block until motion completes
            tolerance: Position tolerance in mm for wait mode
            timeout: Maximum wait time in seconds
        """
        # Get current pose to calculate target for wait mode
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

    # ── Gripper API ──────────────────────────────────────────────

    def _gripper_write_reg(self, addr: int, value: int):
        """Write a single holding register to the gripper."""
        req = SetHoldRegs.Request()
        req.index = self._gripper_modbus_index
        req.addr = addr
        req.count = 1
        req.val_tab = '{' + str(value) + '}'
        req.val_type = 'U16'
        result = self._call_service(self._set_hold_regs_client, req)
        if result.res != 0:
            raise RuntimeError(f'SetHoldRegs failed: addr={addr} res={result.res}')

    def _gripper_read_reg(self, addr: int) -> Optional[int]:
        """Read a single holding register from the gripper."""
        req = GetHoldRegs.Request()
        req.index = self._gripper_modbus_index
        req.addr = addr
        req.count = 1
        req.val_type = 'U16'
        result = self._call_service(self._get_hold_regs_client, req)
        match = re.search(r'\{(\d+)\}', result.robot_return)
        if match:
            return int(match.group(1))
        return None

    def gripper_connect(self, slave_id: int = 1):
        """Create Modbus TCP connection to gripper via internal gateway."""
        req = ModbusCreate.Request()
        req.ip = '127.0.0.1'
        req.port = 60000
        req.slave_id = slave_id
        req.is_rtu = 1
        result = self._call_service(self._modbus_create_client, req)
        match = re.search(r'\{(\d+)\}', result.robot_return)
        self._gripper_modbus_index = int(match.group(1)) if match else 0

    def gripper_disconnect(self):
        """Close Modbus connection to gripper."""
        if self._gripper_modbus_index >= 0:
            req = ModbusClose.Request()
            req.index = self._gripper_modbus_index
            self._call_service(self._modbus_close_client, req)
            self._gripper_modbus_index = -1
            self._gripper_initialized = False

    def gripper_init(self, timeout: float = 15.0):
        """Initialize the gripper (full init, 0xA5). Blocks until ready."""
        if self._gripper_modbus_index < 0:
            self.gripper_connect()
        self._gripper_write_reg(256, 165)  # 0x0100 = 0xA5
        import time
        start = time.time()
        while time.time() - start < timeout:
            status = self._gripper_read_reg(512)  # 0x0200
            if status == 1:
                self._gripper_initialized = True
                return
            time.sleep(0.5)
        raise RuntimeError('Gripper init timed out')

    def gripper_move(
        self,
        position: int,
        force: int = 50,
        speed: int = 50,
        wait: bool = True,
        timeout: float = 10.0,
    ) -> int:
        """
        Move gripper to position.

        Args:
            position: 0 (closed) to 1000 (open)
            force: 20-100 percent
            speed: 1-100 percent
            wait: block until motion completes
            timeout: max wait time in seconds

        Returns:
            Grip state: 1=reached, 2=object caught, 3=object dropped
        """
        if self._gripper_modbus_index < 0:
            raise RuntimeError('Gripper not connected. Call gripper_init() first.')
        self._gripper_write_reg(257, force)    # 0x0101
        self._gripper_write_reg(260, speed)    # 0x0104
        self._gripper_write_reg(259, position) # 0x0103

        if not wait:
            return 0

        import time
        start = time.time()
        while time.time() - start < timeout:
            state = self._gripper_read_reg(513)  # 0x0201
            if state is not None and state != 0:
                return state
            time.sleep(0.05)
        return -1  # timeout

    def gripper_open(self, force: int = 50, speed: int = 50, wait: bool = True) -> int:
        """Open gripper fully."""
        return self.gripper_move(1000, force=force, speed=speed, wait=wait)

    def gripper_close(self, force: int = 50, speed: int = 50, wait: bool = True) -> int:
        """Close gripper fully."""
        return self.gripper_move(0, force=force, speed=speed, wait=wait)

    def gripper_get_position(self) -> Optional[int]:
        """Read current gripper position (0=closed, 1000=open)."""
        return self._gripper_read_reg(514)  # 0x0202

    def gripper_get_state(self) -> Optional[int]:
        """Read grip state: 0=moving, 1=reached, 2=caught, 3=dropped."""
        return self._gripper_read_reg(513)  # 0x0201

    def gripper_get_init_status(self) -> Optional[int]:
        """Read init status: 0=not initialized, 1=initialized."""
        return self._gripper_read_reg(512)  # 0x0200

    def shutdown(self):
        """Shutdown the ROS2 client."""
        self.gripper_disconnect()
        self.destroy_node()
