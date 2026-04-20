"""Strategy: Simple Top-Down Pick (MovL).

Straight vertical approach from directly above the object, using point-to-
point MovL motion between waypoints. Safe baseline for testing calibration
accuracy.

Waypoints:
  1. Open gripper
  2. Move to approach pose (directly above target, approach_height above grasp Z)
  3. Descend to grasp Z
  4. Close gripper
  5. Retract straight up

The wrist is forced vertical (RX=180, RY=0) for every waypoint — the
orchestrator runs Vertical + Lock during setup, so the pick never inherits
a tilted wrist from the prior pose.
"""

from __future__ import annotations

import logging
import time
from typing import List

from dobot_ros.strategies.base import (
    ConfirmFn, ParameterDef, PickContext, PickPlan, PickStrategy, PickWaypoint,
)

log = logging.getLogger(__name__)


class SimpleTopDown(PickStrategy):
    name = "Simple Top-Down (MovL)"
    slug = "simple_top_down"
    description = "Straight vertical approach via point-to-point MovL. Safe baseline."
    motion_mode = "movl"

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
            ParameterDef("move_speed", "int", 30,
                         "Robot speed factor during the pick sequence",
                         min=1, max=100, step=5, unit="%"),
        ]

    def plan(self, ctx: PickContext) -> PickPlan:
        p = ctx.params

        approach_h = float(p.get("approach_height_mm", 100))
        clearance = float(p.get("grasp_clearance_mm", 5))
        lift_h = float(p.get("lift_height_mm", 150))
        use_height = bool(p.get("use_object_height", True))
        open_pos = int(p.get("gripper_open_pos", 800))
        close_pos = int(p.get("gripper_close_pos", 200))
        speed = int(p.get("move_speed", 30))

        # Heights above table surface. SAFETY: always enforce min_clearance_mm
        # as the absolute floor; object_height only ever raises grasp.
        grasp_above = max(clearance, ctx.min_clearance_mm)
        if use_height and ctx.object_present and ctx.object_height_mm > 10:
            grasp_above = max(grasp_above, ctx.object_height_mm / 2.0)

        # Convert "height above table" to a pose Z in the active tool frame.
        grasp_z = ctx.pose_z_from_table_z(grasp_above)
        approach_z = ctx.pose_z_from_table_z(grasp_above + approach_h)
        retract_z = ctx.pose_z_from_table_z(grasp_above + lift_h)

        # Force vertical: setup phase guarantees the wrist is at (180, 0)
        # and lock_vertical is on. Hard-coding here makes plans previewable
        # without depending on whether setup has run yet.
        rx, ry = 180.0, 0.0
        rz = ctx.rotation_deg

        return PickPlan(
            move_speed=speed,
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
                ),
                PickWaypoint(
                    pose=[ctx.robot_x, ctx.robot_y, retract_z, rx, ry, rz],
                    label="Retract (lift)",
                ),
            ],
        )

    def execute(self, ctx, plan, client, confirm_fn: ConfirmFn, servo=None) -> None:
        force = int(ctx.params.get("gripper_force", 50))
        for i, wp in enumerate(plan.waypoints):
            if wp.confirm:
                confirm_fn(wp.confirm)  # raises on cancel/timeout

            if wp.gripper_action == "open" and wp.gripper_pos is not None:
                log.info("simple_top_down [%d/%d] %s: open gripper → %d",
                         i + 1, len(plan.waypoints), wp.label, wp.gripper_pos)
                try:
                    client.gripper_move(wp.gripper_pos, force=force, speed=50, wait=True)
                except Exception as e:
                    log.warning("gripper open failed: %s", e)

            log.info("simple_top_down [%d/%d] %s → %s",
                     i + 1, len(plan.waypoints), wp.label, wp.pose)
            client.move_pose(wp.pose)
            if not client.wait_for_cartesian_motion(wp.pose, tolerance=3.0, timeout=15.0):
                raise RuntimeError(
                    f"step {i+1} '{wp.label}' timed out — robot may not have arrived")

            if wp.pause_s > 0:
                time.sleep(wp.pause_s)

            if wp.gripper_action == "close" and wp.gripper_pos is not None:
                log.info("simple_top_down [%d/%d] %s: close gripper → %d",
                         i + 1, len(plan.waypoints), wp.label, wp.gripper_pos)
                try:
                    client.gripper_move(wp.gripper_pos, force=force, speed=50, wait=True)
                except Exception as e:
                    log.warning("gripper close failed: %s", e)
