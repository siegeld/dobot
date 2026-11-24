"""Safety guards shared by the simulator and the real adapter — the
engine-level fail-safes that hold regardless of what a policy asks for.

The floor guard follows the project convention (see strategies/base.py):
``table_z`` is the *wrist* Z when the fingertip touches the table. With
Tool 0 active a commanded Z is the wrist; with Tool ≥1 it is the fingertip.

What is enforced, every motion, both backends:
  * floor        — fingertip never closer than ``min_clearance_mm`` to the table
  * workspace    — XY inside the calibrated table bounds
  * tilt         — wrist within ``max_tilt_deg`` of vertical (RX=180, RY=0),
                   so the gripper body can never swing below the fingertip
  * travel cap   — one move may not exceed ``max_move_mm``
  * path check   — the *whole* path, sampled through the kinematic model,
                   must satisfy floor + workspace (not just the endpoint)
  * speed/force  — capped at ``max_speed`` / ``max_force``
"""

from __future__ import annotations

import math
from dataclasses import dataclass, asdict
from typing import List, Optional, Sequence

from dobot_ros.policies.types import FloorViolation, PolicyError, WorkspaceViolation


class TiltViolation(PolicyError):
    """Commanded wrist orientation is too far from vertical."""


class TravelViolation(PolicyError):
    """A single move is longer than the per-move travel cap."""


@dataclass
class WorkspaceBounds:
    x_min: float
    x_max: float
    y_min: float
    y_max: float

    def contains(self, x: float, y: float, slack_mm: float = 0.0) -> bool:
        return (self.x_min - slack_mm <= x <= self.x_max + slack_mm
                and self.y_min - slack_mm <= y <= self.y_max + slack_mm)

    def to_dict(self) -> dict:
        return {"x_min": self.x_min, "x_max": self.x_max, "y_min": self.y_min, "y_max": self.y_max}

    @classmethod
    def from_points(cls, points: Sequence[Sequence[float]], margin_mm: float = 0.0) -> "WorkspaceBounds":
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        return cls(min(xs) + margin_mm, max(xs) - margin_mm, min(ys) + margin_mm, max(ys) - margin_mm)


def _wrap(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


@dataclass
class Guards:
    table_z: float                 # wrist Z at fingertip-touch
    tool_length_mm: float          # Tool 1 offset (203 for the AG-105)
    min_clearance_mm: float
    workspace: Optional[WorkspaceBounds] = None
    max_tilt_deg: Optional[float] = 15.0
    max_move_mm: float = 700.0
    max_speed: int = 50
    max_force: int = 60
    path_step_mm: float = 10.0

    # ── derived ─────────────────────────────────────────────────────
    def min_pose_z(self, tool_index: int) -> float:
        """Lowest Z a commanded pose may have in the given tool frame."""
        if tool_index == 0:
            return self.table_z + self.min_clearance_mm
        return self.table_z - self.tool_length_mm + self.min_clearance_mm

    def fingertip_height(self, pose_z: float, tool_index: int) -> float:
        """Fingertip height above the table surface for a vertical wrist."""
        tip = pose_z if tool_index >= 1 else pose_z - self.tool_length_mm
        return tip - (self.table_z - self.tool_length_mm)

    def clamp_speed(self, speed: Optional[int], default: int) -> int:
        s = int(speed if speed is not None else default)
        return max(1, min(self.max_speed, s))

    def clamp_force(self, force: int) -> int:
        return max(20, min(self.max_force, int(force)))

    # ── checks ──────────────────────────────────────────────────────
    def check(self, pose: Sequence[float], tool_index: int) -> None:
        """Endpoint check: floor, workspace, tilt."""
        z_min = self.min_pose_z(tool_index)
        if pose[2] < z_min - 1e-6:
            raise FloorViolation(
                f"Z={pose[2]:.1f} below floor limit {z_min:.1f} (tool {tool_index}, "
                f"clearance {self.min_clearance_mm:.0f} mm)")
        if self.workspace is not None and not self.workspace.contains(pose[0], pose[1]):
            raise WorkspaceViolation(
                f"XY=({pose[0]:.0f}, {pose[1]:.0f}) outside workspace "
                f"X[{self.workspace.x_min:.0f},{self.workspace.x_max:.0f}] "
                f"Y[{self.workspace.y_min:.0f},{self.workspace.y_max:.0f}]")
        if self.max_tilt_deg is not None and len(pose) >= 5:
            rx_off = abs(_wrap(pose[3] - 180.0))
            ry_off = abs(_wrap(pose[4]))
            if rx_off > self.max_tilt_deg or ry_off > self.max_tilt_deg:
                raise TiltViolation(
                    f"wrist tilt RX={pose[3]:.1f} RY={pose[4]:.1f} exceeds ±{self.max_tilt_deg:.0f}° from vertical")

    def check_travel(self, start: Sequence[float], goal: Sequence[float]) -> None:
        d = math.dist(start[:3], goal[:3])
        if d > self.max_move_mm:
            raise TravelViolation(f"move of {d:.0f} mm exceeds per-move cap {self.max_move_mm:.0f} mm")

    def check_line_path(self, start: Sequence[float], goal: Sequence[float], tool_index: int,
                        seed_joints: Optional[Sequence[float]] = None) -> List[List[float]]:
        """Sample the straight tool path start→goal through the kinematic
        model: every sample must be reachable within joint limits and pass
        floor/workspace. Returns the joint path (useful for previews)."""
        from dobot_ros.sim import kinematics as K
        from dobot_ros.policies.types import Unreachable
        tl = self.tool_length_mm if tool_index >= 1 else 0.0
        n = max(1, int(math.ceil(math.dist(start[:3], goal[:3]) / self.path_step_mm)))
        g = list(goal)
        for i in range(3, 6):
            g[i] = start[i] + _wrap(goal[i] - start[i])
        q = list(seed_joints) if seed_joints is not None else None
        path = []
        for k in range(1, n + 1):
            a = k / n
            p = [start[i] + a * (g[i] - start[i]) for i in range(6)]
            self._check_floor_ws(p, tool_index, where=f"path sample {k}/{n}")
            try:
                q = K.ik(p, seed_deg=q, tool_length_mm=tl)
            except K.IKError as e:
                raise Unreachable(f"path sample {k}/{n} unreachable: {e}") from None
            path.append(q)
        return path

    def check_joint_path(self, q0: Sequence[float], q1: Sequence[float], tool_index: int,
                         step_deg: float = 3.0) -> None:
        """Joint-interpolated move: FK every sample and check floor/workspace."""
        from dobot_ros.sim import kinematics as K
        tl = self.tool_length_mm if tool_index >= 1 else 0.0
        span = max(abs(a - b) for a, b in zip(q0, q1))
        n = max(1, int(math.ceil(span / step_deg)))
        for k in range(1, n + 1):
            a = k / n
            q = [s + a * (g - s) for s, g in zip(q0, q1)]
            self._check_floor_ws(K.fk(q, tl), tool_index, where=f"joint path sample {k}/{n}")

    def _check_floor_ws(self, pose: Sequence[float], tool_index: int, where: str) -> None:
        z_min = self.min_pose_z(tool_index)
        if pose[2] < z_min - 1e-6:
            raise FloorViolation(f"{where}: Z={pose[2]:.1f} below floor limit {z_min:.1f}")
        if self.workspace is not None and not self.workspace.contains(pose[0], pose[1], slack_mm=5.0):
            raise WorkspaceViolation(f"{where}: XY=({pose[0]:.0f}, {pose[1]:.0f}) leaves workspace")

    def to_dict(self) -> dict:
        d = asdict(self)
        d["workspace"] = self.workspace.to_dict() if self.workspace else None
        return d
