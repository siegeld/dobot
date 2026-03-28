"""
Robot controller wrapper for Dobot CR series.
"""

import sys
import threading
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

# Add TCP-IP-Python-V4 to path
sys.path.insert(0, str(Path(__file__).parent.parent / "TCP-IP-Python-V4"))

from dobot_api import DobotApiDashboard, DobotApiFeedBack


class Position:
    """Represents robot position in both joint and cartesian space."""

    def __init__(
        self,
        joint: Optional[List[float]] = None,
        cartesian: Optional[List[float]] = None,
    ) -> None:
        """
        Initialize position.

        Args:
            joint: Joint angles [J1, J2, J3, J4, J5, J6] in degrees
            cartesian: Cartesian coordinates [X, Y, Z, RX, RY, RZ] in mm and degrees
        """
        self.joint = joint or [0.0] * 6
        self.cartesian = cartesian or [0.0] * 6

    @property
    def joint_dict(self) -> Dict[str, float]:
        """Get joint positions as dictionary."""
        return {f"J{i+1}": val for i, val in enumerate(self.joint)}

    @property
    def cartesian_dict(self) -> Dict[str, float]:
        """Get cartesian position as dictionary."""
        keys = ["X", "Y", "Z", "RX", "RY", "RZ"]
        return {key: val for key, val in zip(keys, self.cartesian)}

    def __repr__(self) -> str:
        return f"Position(joint={self.joint}, cartesian={self.cartesian})"


class DobotController:
    """High-level controller for Dobot CR robots."""

    def __init__(
        self,
        ip: str,
        control_port: int = 29999,
        feedback_port: int = 30004,
        timeout: int = 5,
    ) -> None:
        """
        Initialize Dobot controller.

        Args:
            ip: Robot IP address
            control_port: Port for control commands (default: 29999)
            feedback_port: Port for feedback data (default: 30004)
            timeout: Connection timeout in seconds
        """
        self.ip = ip
        self.control_port = control_port
        self.feedback_port = feedback_port
        self.timeout = timeout

        self._dashboard: Optional[DobotApiDashboard] = None
        self._feedback: Optional[DobotApiFeedBack] = None
        self._connected = False
        self._jog_mode: str = "user"  # Default coordinate mode: "user" or "tool"

        # Feedback thread for continuous polling (like SDK example)
        self._feedback_thread: Optional[threading.Thread] = None
        self._feedback_lock = threading.Lock()
        self._latest_feedback = None
        self._stop_feedback = False

    def _feedback_loop(self) -> None:
        """
        Continuously poll feedback to keep data fresh.
        Runs in background thread (like SDK example DobotDemo.GetFeed).
        """
        while not self._stop_feedback:
            try:
                feedback = self._feedback.feedBackData()
                if feedback is not None and len(feedback) > 0:
                    with self._feedback_lock:
                        self._latest_feedback = feedback
            except Exception:
                # Continue polling even on errors
                pass
            time.sleep(0.008)  # ~8ms matches Dobot feedback cycle

    def connect(self, speed_factor: int = 10) -> bool:
        """
        Connect to the robot and set initial speed factor.

        Args:
            speed_factor: Global speed factor (1-100), default 10% (slow and safe)

        Returns:
            True if connection successful

        Raises:
            ConnectionError: If connection fails
        """
        try:
            self._dashboard = DobotApiDashboard(self.ip, self.control_port)
            self._feedback = DobotApiFeedBack(self.ip, self.feedback_port)
            self._connected = True

            # Set speed factor (required before movement commands work)
            self._dashboard.SpeedFactor(speed_factor)

            # Continue() is required to enable command execution
            self._dashboard.Continue()

            # Start background feedback polling thread (like SDK example)
            self._stop_feedback = False
            self._feedback_thread = threading.Thread(target=self._feedback_loop, daemon=True)
            self._feedback_thread.start()

            return True
        except Exception as e:
            raise ConnectionError(f"Failed to connect to robot at {self.ip}: {e}")

    def disconnect(self) -> None:
        """Disconnect from the robot."""
        import socket

        # Stop feedback thread
        self._stop_feedback = True
        if self._feedback_thread and self._feedback_thread.is_alive():
            self._feedback_thread.join(timeout=1.0)

        # Manually close sockets to avoid error messages from SDK
        if self._dashboard and hasattr(self._dashboard, 'socket_dobot'):
            try:
                sock = self._dashboard.socket_dobot
                if sock and sock != 0:
                    try:
                        sock.shutdown(socket.SHUT_RDWR)
                    except (OSError, socket.error):
                        pass  # Socket may already be closed
                    try:
                        sock.close()
                    except (OSError, socket.error):
                        pass
                    self._dashboard.socket_dobot = 0  # Prevent SDK from trying to close again
            except Exception:
                pass

        if self._feedback and hasattr(self._feedback, 'socket_dobot'):
            try:
                sock = self._feedback.socket_dobot
                if sock and sock != 0:
                    try:
                        sock.shutdown(socket.SHUT_RDWR)
                    except (OSError, socket.error):
                        pass  # Socket may already be closed
                    try:
                        sock.close()
                    except (OSError, socket.error):
                        pass
                    self._feedback.socket_dobot = 0  # Prevent SDK from trying to close again
            except Exception:
                pass

        self._connected = False

    @property
    def is_connected(self) -> bool:
        """Check if robot is connected."""
        return self._connected

    def _ensure_connected(self) -> None:
        """Ensure robot is connected."""
        if not self._connected:
            raise RuntimeError("Robot not connected. Call connect() first.")

    def get_position(self) -> Position:
        """
        Get current robot position in both joint and cartesian space.

        Uses cached feedback from background polling thread for fresh data.

        Returns:
            Position object containing joint and cartesian coordinates

        Raises:
            RuntimeError: If robot not connected or no feedback available
        """
        self._ensure_connected()

        # Get latest feedback from background thread (always fresh!)
        with self._feedback_lock:
            feedback = self._latest_feedback

        if feedback is None or len(feedback) == 0:
            raise RuntimeError("No feedback data available from robot")

        # Extract joint positions (QActual) - in degrees
        joint_pos = list(feedback[0]["QActual"])

        # DEBUG: Print raw feedback to see what we're getting
        import sys
        print(f"[DEBUG get_position] Raw QActual: {feedback[0]['QActual']}", file=sys.stderr)
        print(f"[DEBUG get_position] After list(): {joint_pos}", file=sys.stderr)
        print(f"[DEBUG get_position] Types: {[type(x) for x in joint_pos]}", file=sys.stderr)

        # Extract cartesian position (ToolVectorActual) - [X, Y, Z, RX, RY, RZ]
        cartesian_pos = list(feedback[0]["ToolVectorActual"])

        return Position(joint=joint_pos, cartesian=cartesian_pos)

    def get_joint_angles(self) -> List[float]:
        """
        Get current joint angles.

        Returns:
            List of 6 joint angles [J1, J2, J3, J4, J5, J6] in degrees

        Raises:
            RuntimeError: If robot not connected
        """
        return self.get_position().joint

    def enable_robot(self) -> None:
        """Enable the robot."""
        self._ensure_connected()
        self._dashboard.EnableRobot()

    def disable_robot(self) -> None:
        """Disable the robot."""
        self._ensure_connected()
        self._dashboard.DisableRobot()

    def clear_error(self) -> None:
        """Clear robot errors."""
        self._ensure_connected()
        self._dashboard.ClearError()

    def request_control(self) -> None:
        """
        Request TCP/IP control of the robot.

        Must be called before sending movement commands (jog, MovJ, MovL, etc.)
        when the robot is in local control mode.

        Raises:
            RuntimeError: If robot not connected
        """
        self._ensure_connected()
        result = self._dashboard.RequestControl()
        # Result format is typically like "RequestControl(),0"
        # where 0 indicates success

    @property
    def jog_mode(self) -> str:
        """Get current jog coordinate mode ('user' or 'tool')."""
        return self._jog_mode

    def set_jog_mode(self, mode: str) -> None:
        """
        Set jog coordinate mode.

        Args:
            mode: Coordinate mode - 'user' or 'tool'

        Raises:
            ValueError: If mode is not 'user' or 'tool'
        """
        if mode not in ("user", "tool"):
            raise ValueError(f"Invalid jog mode: {mode}. Must be 'user' or 'tool'")
        self._jog_mode = mode

    def move_joints(self, angles: List[float]) -> None:
        """
        Move to absolute joint angles.

        Args:
            angles: List of 6 joint angles [J1, J2, J3, J4, J5, J6] in degrees

        Raises:
            RuntimeError: If robot not connected
            ValueError: If angles list is not length 6
        """
        if len(angles) != 6:
            raise ValueError(f"Expected 6 joint angles, got {len(angles)}")

        self._ensure_connected()

        # Convert to Python floats
        target = [float(x) for x in angles]

        # Move to position using MoveJ with joint mode (coordinateMode=1)
        # Speed controlled by global SpeedFactor (set to 10% in connect())

        # DEBUG: Print what we're sending
        import sys
        print(f"\n[DEBUG] Calling MovJ with:", file=sys.stderr)
        print(f"  Joint angles: {target}", file=sys.stderr)
        print(f"  coordinateMode: 1 (joint)", file=sys.stderr)
        print(f"  SpeedFactor: 10% (global)", file=sys.stderr)

        result = self._dashboard.MovJ(*target, 1)
        print(f"  Result: {result}", file=sys.stderr)

    def jog_joint(self, joint: int, offset: float) -> None:
        """
        Jog a single joint by an offset angle.

        This uses absolute MoveJ positioning to avoid singularities.

        Args:
            joint: Joint number (1-6)
            offset: Angle offset in degrees (positive or negative)

        Raises:
            RuntimeError: If robot not connected
            ValueError: If joint number invalid
        """
        if joint < 1 or joint > 6:
            raise ValueError(f"Joint must be 1-6, got {joint}")

        self._ensure_connected()

        # Get current joint angles and convert to Python floats
        current = [float(x) for x in self.get_joint_angles()]

        # Add offset to target joint (convert to 0-indexed)
        current[joint - 1] += offset

        # Move using move_joints
        self.move_joints(current)

    def jog(
        self,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        rx: float = 0.0,
        ry: float = 0.0,
        rz: float = 0.0,
        speed: int = -1,
        user: int = -1,
        tool: int = -1,
    ) -> None:
        """
        Jog robot by relative offsets in current coordinate mode.

        Args:
            x: X-axis offset in mm
            y: Y-axis offset in mm
            z: Z-axis offset in mm
            rx: Rotation around X-axis in degrees
            ry: Rotation around Y-axis in degrees
            rz: Rotation around Z-axis in degrees
            speed: Movement speed in mm/s (-1 to use global SpeedFactor)
            user: User coordinate system index (-1 for default, 0-9 for custom)
            tool: Tool coordinate system index (-1 for default, 0-9 for custom)

        Raises:
            RuntimeError: If robot not connected
        """
        self._ensure_connected()

        # Use appropriate SDK method based on coordinate mode
        # NOTE: Do NOT pass v parameter - let SpeedFactor control speed
        # Passing v causes insane joint speeds (18519°/s vs 191°/s limit)
        if self._jog_mode == "user":
            # Jog in user coordinate system
            self._dashboard.RelMovLUser(
                offset_x=x,
                offset_y=y,
                offset_z=z,
                offset_rx=rx,
                offset_ry=ry,
                offset_rz=rz,
                user=user,
                tool=tool,
                speed=speed,
            )
        else:  # tool mode
            # Jog in tool coordinate system
            self._dashboard.RelMovLTool(
                offset_x=x,
                offset_y=y,
                offset_z=z,
                offset_rx=rx,
                offset_ry=ry,
                offset_rz=rz,
                user=user,
                tool=tool,
                speed=speed,
            )

    def __enter__(self) -> "DobotController":
        """Context manager entry."""
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        """Context manager exit."""
        self.disconnect()
