"""Strategy: Servo Top-Down Pick (ServoP).

Same vertical approach as simple_top_down, but motion is streamed via the
running ServoTester instead of point-to-point MovL. This piggybacks on the
tester's already-debugged machinery: rate-limited tick loop, lock_vertical
force-projection, workspace floor guard (tool-aware), and ServoP plumbing.

Why a separate strategy:
  - MovL is a single trajectory-planned move per waypoint — instant, but
    not interruptible mid-segment, no rate envelope.
  - ServoP streams the commanded pose at the configured tick rate; the
    tester's max_velocity_xyz / max_velocity_rpy give us a hard cap on
    actual end-effector speed regardless of waypoint distance, and
    lock_vertical clamps every emitted pose to RX=180/RY=0 even if the
    target offset accumulates drift.

Execution model:
  1. Orchestrator brings the robot to (Tool 1, vertical, lock_vertical=True)
     during setup, then captures the post-setup pose as the anchor by
     starting the tester (tester.start() reads get_cartesian_pose()).
  2. For each waypoint, we compute the offset from the anchor and call
     set_target_offset(offset). The tester's tick loop ramps the
     commanded offset toward it at max_velocity_xyz mm/s.
  3. We poll the tester status until the commanded offset is within
     converge_tol_mm of the target (or the timeout elapses).
  4. Gripper open/close calls happen between waypoints, same as MovL.
"""

from __future__ import annotations

import logging
import math
import time
from typing import List

from dobot_ros.strategies.base import (
    ConfirmFn, ParameterDef, PickContext, PickPlan, PickStrategy, PickWaypoint,
)

log = logging.getLogger(__name__)


class ServoTopDown(PickStrategy):
    name = "Servo Top-Down (ServoP)"
    slug = "servo_top_down"
    description = (
        "Vertical approach streamed via ServoP. Rate-capped motion, "
        "lock_vertical-enforced pose, tool-aware floor guard."
    )
    motion_mode = "servo"

    @classmethod
    def parameter_defs(cls) -> List[ParameterDef]:
        return [
            ParameterDef("approach_height_mm", "float", 100.0,
                         "Height above grasp Z for the approach pose",
                         min=20, max=300, step=10, unit="mm"),
            ParameterDef("grasp_clearance_mm", "float", 5.0,
                         "Minimum clearance above table at grasp",
                         min=0, max=100, step=1, unit="mm"),
            ParameterDef("lift_height_mm", "float", 150.0,
                         "How high to lift after grasping",
                         min=20, max=400, step=10, unit="mm"),
            ParameterDef("use_object_height", "bool", True,
                         "Adjust grasp Z upward for tall objects (uses depth-estimated height)"),
            ParameterDef("gripper_open_pos", "int", 800,
                         "Gripper position for open",
                         min=0, max=1000, step=50),
            ParameterDef("gripper_close_pos", "int", 200,
                         "Gripper position for close",
                         min=0, max=1000, step=50),
            ParameterDef("gripper_force", "int", 50,
                         "Grip force (AG-105: force also controls speed)",
                         min=20, max=100, step=5, unit="%"),
            ParameterDef("max_velocity_xyz", "float", 60.0,
                         "Cap on end-effector linear speed during the pick",
                         min=10, max=200, step=10, unit="mm/s"),
            ParameterDef("converge_tol_mm", "float", 2.0,
                         "Waypoint convergence tolerance",
                         min=0.5, max=10, step=0.5, unit="mm"),
            ParameterDef("waypoint_timeout_s", "float", 20.0,
                         "Max time to reach a single waypoint before aborting",
                         min=2, max=120, step=1, unit="s"),
            ParameterDef("confirm_before_close", "bool", False,
                         "Pause for an explicit confirm right before closing on the object"),
        ]

    def plan(self, ctx: PickContext) -> PickPlan:
        p = ctx.params

        approach_h = float(p.get("approach_height_mm", 100))
        clearance = float(p.get("grasp_clearance_mm", 5))
        lift_h = float(p.get("lift_height_mm", 150))
        use_height = bool(p.get("use_object_height", True))
        open_pos = int(p.get("gripper_open_pos", 800))
        close_pos = int(p.get("gripper_close_pos", 200))
        confirm_close = bool(p.get("confirm_before_close", False))

        grasp_above = max(clearance, ctx.min_clearance_mm)
        if use_height and ctx.object_present and ctx.object_height_mm > 10:
            grasp_above = max(grasp_above, ctx.object_height_mm / 2.0)

        grasp_z = ctx.pose_z_from_table_z(grasp_above)
        approach_z = ctx.pose_z_from_table_z(grasp_above + approach_h)
        retract_z = ctx.pose_z_from_table_z(grasp_above + lift_h)

        rx, ry = 180.0, 0.0
        rz = ctx.rotation_deg

        return PickPlan(
            move_speed=None,  # servo strategies don't use the global speed factor
            waypoints=[
                PickWaypoint(
                    pose=[ctx.robot_x, ctx.robot_y, approach_z, rx, ry, rz],
                    label="Approach (above target)",
                    gripper_action="open", gripper_pos=open_pos,
                ),
                PickWaypoint(
                    pose=[ctx.robot_x, ctx.robot_y, grasp_z, rx, ry, rz],
                    label="Descend to grasp",
                    pause_s=0.2,
                ),
                PickWaypoint(
                    pose=[ctx.robot_x, ctx.robot_y, grasp_z, rx, ry, rz],
                    label="Close gripper",
                    gripper_action="close", gripper_pos=close_pos,
                    pause_s=0.3,
                    confirm=("Robot is at grasp pose. Close gripper on object?"
                             if confirm_close else None),
                ),
                PickWaypoint(
                    pose=[ctx.robot_x, ctx.robot_y, retract_z, rx, ry, rz],
                    label="Retract (lift)",
                ),
            ],
        )

    # ── Execution ──────────────────────────────────────────────────
    def execute(self, ctx, plan, client, confirm_fn: ConfirmFn, servo=None) -> None:
        if servo is None:
            raise RuntimeError(
                "servo_top_down requires a running ServoTester — orchestrator "
                "must start it before calling execute()")
        if not servo.is_running():
            raise RuntimeError("ServoTester is not running")

        force = int(ctx.params.get("gripper_force", 50))
        cap_xyz = float(ctx.params.get("max_velocity_xyz", 60.0))
        tol_mm = float(ctx.params.get("converge_tol_mm", 2.0))
        timeout_s = float(ctx.params.get("waypoint_timeout_s", 20.0))

        # Constrain the tester's velocity envelope for the duration of this
        # pick. The tester's prior values are restored by the orchestrator
        # via the same update_config() path when it cleans up.
        servo.update_config(max_velocity_xyz=cap_xyz)

        anchor = list(servo.status().anchor_pose)
        if len(anchor) != 6:
            raise RuntimeError("ServoTester anchor missing — was it started?")

        for i, wp in enumerate(plan.waypoints):
            if wp.confirm:
                confirm_fn(wp.confirm)

            if wp.gripper_action == "open" and wp.gripper_pos is not None:
                log.info("servo_top_down [%d/%d] %s: open gripper → %d",
                         i + 1, len(plan.waypoints), wp.label, wp.gripper_pos)
                try:
                    client.gripper_move(wp.gripper_pos, force=force, speed=50, wait=True)
                except Exception as e:
                    log.warning("gripper open failed: %s", e)

            offset = [wp.pose[k] - anchor[k] for k in range(6)]
            log.info("servo_top_down [%d/%d] %s → offset %s",
                     i + 1, len(plan.waypoints), wp.label,
                     [round(v, 2) for v in offset])
            servo.set_target_offset(offset)
            self._wait_converged(servo, offset, tol_mm, timeout_s, wp.label)

            if wp.pause_s > 0:
                time.sleep(wp.pause_s)

            if wp.gripper_action == "close" and wp.gripper_pos is not None:
                log.info("servo_top_down [%d/%d] %s: close gripper → %d",
                         i + 1, len(plan.waypoints), wp.label, wp.gripper_pos)
                try:
                    client.gripper_move(wp.gripper_pos, force=force, speed=50, wait=True)
                except Exception as e:
                    log.warning("gripper close failed: %s", e)

    @staticmethod
    def _wait_converged(servo, target_offset, tol_mm: float,
                        timeout_s: float, label: str) -> None:
        """Poll the tester until commanded XYZ offset matches target within
        tol_mm (RPY uses a fixed 1° tolerance; lock_vertical pins RX/RY so
        only RZ matters in practice). Raises on timeout."""
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            st = servo.status()
            cur = list(st.last_target_offset) if st.last_target_offset else [0.0] * 6
            dx = target_offset[0] - cur[0]
            dy = target_offset[1] - cur[1]
            dz = target_offset[2] - cur[2]
            xyz_err = math.sqrt(dx * dx + dy * dy + dz * dz)
            drz = abs(((target_offset[5] - cur[5]) + 180) % 360 - 180)
            if xyz_err <= tol_mm and drz <= 1.0:
                return
            time.sleep(0.05)
        raise RuntimeError(
            f"servo_top_down: '{label}' did not converge within {timeout_s:.1f}s")
