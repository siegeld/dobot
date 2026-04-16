"""Motion validation functions — defense-in-depth safety checks.

No ROS dependencies. These are the last-resort guards before any value
reaches the robot controller. Importable from tests without rclpy.
"""

import math
from typing import List


def validate_finite(values: List[float], label: str = "values"):
    """Reject NaN/Inf before any value reaches the robot controller."""
    for i, v in enumerate(values):
        if not math.isfinite(v):
            raise ValueError(
                f"{label}[{i}] is {v} — NaN/Inf must never reach the robot"
            )


def validate_pose_bounds(pose: List[float]):
    """Sanity-check a cartesian pose against hard physical limits."""
    MAX_XY = 1500.0
    MIN_Z = -50.0     # mm — the wrist should almost never go below the base plane
    MAX_Z = 1500.0
    MAX_ROT = 360.0
    labels = ["X", "Y", "Z", "RX", "RY", "RZ"]
    for i in range(2):
        if abs(pose[i]) > MAX_XY:
            raise ValueError(
                f"Pose {labels[i]}={pose[i]:.1f}mm exceeds hard limit ±{MAX_XY}mm"
            )
    if pose[2] < MIN_Z:
        raise ValueError(f"Pose Z={pose[2]:.1f}mm below hard floor {MIN_Z}mm")
    if pose[2] > MAX_Z:
        raise ValueError(f"Pose Z={pose[2]:.1f}mm exceeds hard ceiling {MAX_Z}mm")
    for i in range(3, 6):
        if abs(pose[i]) > MAX_ROT:
            raise ValueError(
                f"Pose {labels[i]}={pose[i]:.1f}° exceeds hard limit ±{MAX_ROT}°"
            )


def validate_joint_bounds(joints: List[float]):
    """Sanity-check joint angles against CR5 physical limits."""
    LIMITS = [
        (-360, 360), (-360, 360), (-160, 160),
        (-360, 360), (-360, 360), (-360, 360),
    ]
    for i, (lo, hi) in enumerate(LIMITS):
        if joints[i] < lo or joints[i] > hi:
            raise ValueError(
                f"Joint J{i+1}={joints[i]:.1f}° outside limits [{lo}, {hi}]"
            )
