"""CR5 forward / inverse kinematics from the vendor URDF.

Joint origins are transcribed from ``DOBOT_6Axis_ROS2_V4/dobot_rviz/urdf/
cr5_robot.urdf`` (the same numbers drive the three.js viewer in
``web/static/js/robot3d.js``). Every joint axis is the local Z of its frame.

Conventions
-----------
* Joint angles: degrees, Dobot order J1..J6, applied directly to the URDF
  joints (the live 3D viewer does the same and renders the real arm
  correctly, so sign/offset match).
* Pose: Dobot ``[X, Y, Z, RX, RY, RZ]`` in mm / degrees. The rotation is
  assumed to be fixed-axis roll-pitch-yaw, ``R = Rz(RZ) · Ry(RY) · Rx(RX)``
  — the convention under which the robot's documented "vertical" wrist
  (RX=180, RY=0) points the tool Z straight down, and the same convention
  URDF uses for its own ``rpy``.
* Tool frame: the Link6 (flange) frame. Its +Z is the flange normal — the
  viewer builds the gripper along +Z and the AG-105 tip sits at +203 mm.
  ``Tool(1)`` on the controller is ``{0,0,203,0,0,0}``, i.e. a pure +Z
  translation, so ``tool_length_mm`` here is that offset.

What is NOT yet validated against the live robot: the RZ zero (a constant
yaw offset between Link6 X and Dobot's tool X would show up as a fixed RZ
error) and the Euler convention itself. ``scripts/validate_fk.py`` samples
(joints, pose) pairs from the running driver and fits both; run it the
first time the arm is on. ``FLANGE_YAW_OFFSET_DEG`` is the knob it sets.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence

import numpy as np

# ── URDF joint origins (metres, radians) ────────────────────────────
# (xyz, rpy) of each <joint><origin>, parent → child, before the joint's
# own rotation about local Z.
_JOINT_ORIGINS = [
    ((0.0, 0.0, 0.147), (0.0, 0.0, 0.0)),                      # joint1
    ((0.0, 0.0, 0.0), (math.pi / 2, math.pi / 2, 0.0)),        # joint2
    ((-0.427, 0.0, 0.0), (0.0, 0.0, 0.0)),                     # joint3
    ((-0.357, 0.0, 0.141), (0.0, 0.0, -math.pi / 2)),          # joint4
    ((0.0, -0.116, 0.0), (math.pi / 2, 0.0, 0.0)),             # joint5
    ((0.0, 0.105, 0.0), (-math.pi / 2, 0.0, 0.0)),             # joint6
]

# Joint limits in degrees (URDF: ±6.27 rad except J3 ±2.86 rad).
JOINT_LIMITS_DEG: List[tuple] = [
    (-359.0, 359.0), (-359.0, 359.0), (-163.0, 163.0),
    (-359.0, 359.0), (-359.0, 359.0), (-359.0, 359.0),
]

# Fixed yaw between Link6 X and Dobot's tool X. 0 until validated live.
FLANGE_YAW_OFFSET_DEG = 0.0

DEFAULT_TOOL_LENGTH_MM = 203.0


class IKError(Exception):
    """Pose unreachable or IK failed to converge."""


# ── Rotation helpers ────────────────────────────────────────────────

def rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)


def rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)


def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


def rpy_to_matrix(rx_deg: float, ry_deg: float, rz_deg: float) -> np.ndarray:
    """Dobot RX/RY/RZ (degrees) → rotation matrix, R = Rz·Ry·Rx."""
    return rot_z(math.radians(rz_deg)) @ rot_y(math.radians(ry_deg)) @ rot_x(math.radians(rx_deg))


def matrix_to_rpy(R: np.ndarray, rz_hint_deg: Optional[float] = None) -> List[float]:
    """Rotation matrix → Dobot RX/RY/RZ degrees (R = Rz·Ry·Rx).

    At the gimbal singularity (RY = ±90°) RX and RZ are not independent;
    we put all the rotation into RZ and set RX = 0.
    """
    sy = -R[2, 0]
    sy = max(-1.0, min(1.0, sy))
    ry = math.asin(sy)
    if abs(abs(sy) - 1.0) < 1e-9:
        rx = 0.0
        rz = math.atan2(-R[0, 1], R[1, 1]) if sy > 0 else math.atan2(R[0, 1], -R[1, 1])
    else:
        rx = math.atan2(R[2, 1], R[2, 2])
        rz = math.atan2(R[1, 0], R[0, 0])
    out = [math.degrees(rx), math.degrees(ry), math.degrees(rz)]
    # Dobot reports RX in (-180, 180]; a vertical wrist shows as RX=180 or
    # RX=-180 depending on rounding — normalise to +180 for readability.
    if out[0] <= -179.9999:
        out[0] = 180.0
    return out


def _tf(xyz: Sequence[float], rpy: Sequence[float]) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = rot_z(rpy[2]) @ rot_y(rpy[1]) @ rot_x(rpy[0])
    T[:3, 3] = xyz
    return T


def _rz4(a: float) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = rot_z(a)
    return T


_ORIGIN_TFS = [_tf(xyz, rpy) for xyz, rpy in _JOINT_ORIGINS]
_FLANGE_YAW_TF = _rz4(math.radians(FLANGE_YAW_OFFSET_DEG))


# ── Forward kinematics ──────────────────────────────────────────────

def fk_matrix(joints_deg: Sequence[float], tool_length_mm: float = 0.0) -> np.ndarray:
    """Base → tool homogeneous transform (metres). ``tool_length_mm`` is a
    pure +Z offset from the flange (Tool 0 → 0, Tool 1 → 203)."""
    if len(joints_deg) != 6:
        raise ValueError("need 6 joint angles")
    T = np.eye(4)
    for i in range(6):
        T = T @ _ORIGIN_TFS[i] @ _rz4(math.radians(joints_deg[i]))
    T = T @ _FLANGE_YAW_TF
    if tool_length_mm:
        off = np.eye(4)
        off[2, 3] = tool_length_mm / 1000.0
        T = T @ off
    return T


def fk(joints_deg: Sequence[float], tool_length_mm: float = 0.0) -> List[float]:
    """Joints (deg) → Dobot pose ``[X, Y, Z, RX, RY, RZ]`` (mm, deg)."""
    T = fk_matrix(joints_deg, tool_length_mm)
    p = T[:3, 3] * 1000.0
    rpy = matrix_to_rpy(T[:3, :3])
    return [float(p[0]), float(p[1]), float(p[2]), rpy[0], rpy[1], rpy[2]]


def joint_positions(joints_deg: Sequence[float]) -> List[List[float]]:
    """Origin of each joint frame plus the flange, in mm — for drawing."""
    pts = []
    T = np.eye(4)
    for i in range(6):
        T = T @ _ORIGIN_TFS[i]
        pts.append((T[:3, 3] * 1000.0).tolist())
        T = T @ _rz4(math.radians(joints_deg[i]))
    pts.append((T[:3, 3] * 1000.0).tolist())
    return pts


# ── Inverse kinematics ──────────────────────────────────────────────

def _pose_error(T_cur: np.ndarray, T_goal: np.ndarray) -> np.ndarray:
    """6-vector [dx dy dz (m), wx wy wz (rad)] from current to goal."""
    dp = T_goal[:3, 3] - T_cur[:3, 3]
    R_err = T_goal[:3, :3] @ T_cur[:3, :3].T
    # Rotation-matrix log → axis-angle.
    cos_a = max(-1.0, min(1.0, (np.trace(R_err) - 1.0) / 2.0))
    ang = math.acos(cos_a)
    if ang < 1e-9:
        w = np.zeros(3)
    elif abs(ang - math.pi) < 1e-6:
        # 180° — pick the axis from the diagonal.
        axis = np.sqrt(np.maximum((np.diag(R_err) + 1.0) / 2.0, 0.0))
        w = axis * ang
    else:
        w = np.array([R_err[2, 1] - R_err[1, 2],
                      R_err[0, 2] - R_err[2, 0],
                      R_err[1, 0] - R_err[0, 1]]) * (ang / (2.0 * math.sin(ang)))
    return np.concatenate([dp, w])


def _wrap_deg(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def within_limits(joints_deg: Sequence[float]) -> bool:
    return all(lo <= q <= hi for q, (lo, hi) in zip(joints_deg, JOINT_LIMITS_DEG))


def ik(
    pose: Sequence[float],
    seed_deg: Optional[Sequence[float]] = None,
    tool_length_mm: float = 0.0,
    max_iters: int = 300,
    pos_tol_mm: float = 0.05,
    rot_tol_deg: float = 0.01,
    damping: float = 1e-3,
) -> List[float]:
    """Damped-least-squares IK. Returns joints (deg) reaching ``pose``.

    Seeded from ``seed_deg`` (default: a generic elbow-up posture) so the
    solution stays on the seed's branch — the way a cartesian MovL keeps
    the arm configuration. Raises ``IKError`` if it does not converge or
    the answer violates joint limits.
    """
    if len(pose) != 6:
        raise ValueError("pose must be [X,Y,Z,RX,RY,RZ]")
    T_goal = np.eye(4)
    T_goal[:3, :3] = rpy_to_matrix(pose[3], pose[4], pose[5])
    T_goal[:3, 3] = np.asarray(pose[:3], dtype=float) / 1000.0

    seeds = [list(seed_deg)] if seed_deg is not None else []
    seeds += [[0.0, 0.0, -90.0, 0.0, 90.0, 0.0], [0.0, 0.0, 90.0, 0.0, -90.0, 0.0],
              [0.0, -45.0, -90.0, 0.0, 45.0, 0.0]]

    best = None
    for seed in seeds:
        q = np.radians(np.asarray(seed, dtype=float))
        for _ in range(max_iters):
            T_cur = fk_matrix(np.degrees(q), tool_length_mm)
            e = _pose_error(T_cur, T_goal)
            if np.linalg.norm(e[:3]) * 1000.0 < pos_tol_mm and math.degrees(np.linalg.norm(e[3:])) < rot_tol_deg:
                q_deg = [_wrap_deg(math.degrees(v)) for v in q]
                if within_limits(q_deg):
                    return q_deg
                break
            J = _jacobian(q, tool_length_mm)
            JJt = J @ J.T + damping * np.eye(6)
            dq = J.T @ np.linalg.solve(JJt, e)
            # Step clamp keeps the linearisation honest far from the goal.
            n = np.linalg.norm(dq)
            if n > 0.5:
                dq *= 0.5 / n
            q = q + dq
        else:
            pass
        T_cur = fk_matrix(np.degrees(q), tool_length_mm)
        err = np.linalg.norm(_pose_error(T_cur, T_goal)[:3]) * 1000.0
        if best is None or err < best[0]:
            best = (err, q)
    raise IKError(f"IK did not converge for pose {[round(v, 1) for v in pose]} "
                  f"(best residual {best[0]:.1f} mm)" if best else "IK failed")


def _jacobian(q: np.ndarray, tool_length_mm: float, h: float = 1e-6) -> np.ndarray:
    """Numeric geometric Jacobian (6×6) at joint vector ``q`` (radians)."""
    T0 = fk_matrix(np.degrees(q), tool_length_mm)
    J = np.zeros((6, 6))
    for i in range(6):
        dq = q.copy()
        dq[i] += h
        Ti = fk_matrix(np.degrees(dq), tool_length_mm)
        J[:, i] = _pose_error(T0, Ti) / h
    return J


def reachable(pose: Sequence[float], seed_deg: Optional[Sequence[float]] = None,
              tool_length_mm: float = 0.0) -> bool:
    try:
        ik(pose, seed_deg=seed_deg, tool_length_mm=tool_length_mm)
        return True
    except IKError:
        return False


@dataclass
class EulerFit:
    """Result of fitting the Euler convention against live data."""
    convention: str
    yaw_offset_deg: float
    max_pos_err_mm: float
    max_rot_err_deg: float
