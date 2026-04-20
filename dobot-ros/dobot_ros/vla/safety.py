"""Safety clamps for VLA-generated actions.

The VLA model is not trustworthy. Every action it emits must be checked against
hard workspace limits and per-step motion caps before being sent to ServoP.
"""

from __future__ import annotations

import json
import logging
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Tuple


log = logging.getLogger(__name__)


@dataclass
class SafetyLimits:
    # Per-step motion caps (limit how far one action can move the EE).
    max_xyz_step_mm: float = 20.0
    max_rot_step_deg: float = 5.0

    # Workspace bounds in robot base frame (all mm).
    x_min: float = -600.0
    x_max: float = 600.0
    y_min: float = -600.0
    y_max: float = 600.0
    z_min: float = 0.0        # floor / table plane
    z_max: float = 800.0

    # Wrist angle limits (deg). Conservative; tighten as needed.
    rx_min: float = -180.0
    rx_max: float = 180.0
    ry_min: float = -180.0
    ry_max: float = 180.0
    rz_min: float = -180.0
    rz_max: float = 180.0

    @classmethod
    def from_table_plane(
        cls,
        path: Path,
        margin_mm: float = 10.0,
        tool_length_mm: float = 0.0,
    ) -> "SafetyLimits":
        """Build limits from a table_plane.json calibration file.

        The table corners are recorded as the **wrist/flange** pose when the
        gripper tip was touching the table (see CLAUDE.md). Therefore:

          actual_table_z (fingertip frame) ≈ wrist_corner_z − tool_length_mm

        `tool_length_mm` selects which frame the floor is expressed in:
          * 0.0  → commanded poses are in WRIST frame (Tool 0 default).
                   z_min = max(corner_wrist_z) + margin_mm.
          * N    → commanded poses are in FINGERTIP frame (Tool N with
                   length N mm). z_min = max(corner_wrist_z) − N + margin_mm
                   so the fingertip can't go below the table surface.
        """
        lims = cls()
        try:
            data = json.loads(Path(path).read_text())
        except (FileNotFoundError, json.JSONDecodeError) as e:
            log.warning("Could not load %s: %s — using default safety limits", path, e)
            return lims

        corners = data.get("corners") or data.get("points") or []
        if len(corners) >= 3:
            xs = [c["x"] if isinstance(c, dict) else c[0] for c in corners]
            ys = [c["y"] if isinstance(c, dict) else c[1] for c in corners]
            zs = [c["z"] if isinstance(c, dict) else c[2] for c in corners]
            lims.x_min, lims.x_max = min(xs), max(xs)
            lims.y_min, lims.y_max = min(ys), max(ys)
            # Max corner Z is the highest point of the table surface in wrist coords.
            # Subtract the tool length when the active tool has shifted the reported
            # frame to the fingertip, so the floor is correct in that frame.
            lims.z_min = max(zs) - float(tool_length_mm) + float(margin_mm)
        return lims


@dataclass
class ClampResult:
    pose: List[float]
    delta: List[float]
    clamped: bool
    reasons: List[str] = field(default_factory=list)


def clamp_delta(delta: List[float], limits: SafetyLimits) -> Tuple[List[float], List[str]]:
    """Clamp a raw delta action to per-step motion caps. Scales XYZ uniformly to
    preserve direction if the norm exceeds max_xyz_step_mm."""
    reasons: List[str] = []

    # NaN/Inf guard — reject before any arithmetic.
    if not all(math.isfinite(v) for v in delta):
        reasons.append("non-finite delta values")
        return [0.0] * 6, reasons

    dx, dy, dz, drx, dry, drz = delta

    xyz_norm = math.sqrt(dx * dx + dy * dy + dz * dz)
    if xyz_norm > limits.max_xyz_step_mm:
        scale = limits.max_xyz_step_mm / xyz_norm
        dx, dy, dz = dx * scale, dy * scale, dz * scale
        reasons.append(f"xyz_step_norm={xyz_norm:.1f}>max_xyz_step_mm")

    for name, v, cap in (("drx", drx, limits.max_rot_step_deg),
                        ("dry", dry, limits.max_rot_step_deg),
                        ("drz", drz, limits.max_rot_step_deg)):
        if abs(v) > cap:
            reasons.append(f"{name}={v:.2f}>max_rot_step_deg")
    drx = max(-limits.max_rot_step_deg, min(limits.max_rot_step_deg, drx))
    dry = max(-limits.max_rot_step_deg, min(limits.max_rot_step_deg, dry))
    drz = max(-limits.max_rot_step_deg, min(limits.max_rot_step_deg, drz))

    return [dx, dy, dz, drx, dry, drz], reasons


def clamp_pose(pose: List[float], limits: SafetyLimits) -> Tuple[List[float], List[str]]:
    """Clamp an absolute pose to workspace bounds."""
    reasons: List[str] = []

    if not all(math.isfinite(v) for v in pose):
        reasons.append("non-finite pose values")
        return list(pose), reasons  # return as-is; caller should abort

    x, y, z, rx, ry, rz = pose

    for name, v, lo, hi in (
        ("x", x, limits.x_min, limits.x_max),
        ("y", y, limits.y_min, limits.y_max),
        ("z", z, limits.z_min, limits.z_max),
        ("rx", rx, limits.rx_min, limits.rx_max),
        ("ry", ry, limits.ry_min, limits.ry_max),
        ("rz", rz, limits.rz_min, limits.rz_max),
    ):
        if v < lo:
            reasons.append(f"{name}={v:.2f}<{lo:.2f}")
        elif v > hi:
            reasons.append(f"{name}={v:.2f}>{hi:.2f}")

    x = max(limits.x_min, min(limits.x_max, x))
    y = max(limits.y_min, min(limits.y_max, y))
    z = max(limits.z_min, min(limits.z_max, z))
    rx = max(limits.rx_min, min(limits.rx_max, rx))
    ry = max(limits.ry_min, min(limits.ry_max, ry))
    rz = max(limits.rz_min, min(limits.rz_max, rz))

    return [x, y, z, rx, ry, rz], reasons


def apply(
    current_pose: List[float],
    delta: List[float],
    limits: SafetyLimits,
) -> ClampResult:
    """Full per-action safety pipeline: clamp delta, add, clamp pose."""
    reasons: List[str] = []
    clamped_delta, r1 = clamp_delta(delta, limits)
    reasons.extend(r1)
    target_pose = [current_pose[i] + clamped_delta[i] for i in range(6)]
    clamped_pose, r2 = clamp_pose(target_pose, limits)
    reasons.extend(r2)

    return ClampResult(
        pose=clamped_pose,
        delta=[clamped_pose[i] - current_pose[i] for i in range(6)],
        clamped=bool(reasons),
        reasons=reasons,
    )
