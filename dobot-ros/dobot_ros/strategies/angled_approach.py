"""Strategy: Angled Approach Pick.

Approaches the object from an offset direction at a configurable angle,
then drops straight down to grasp. Useful when top-down clearance is
limited or when the gripper needs to avoid occluding the camera view.

Waypoints:
  1. Open gripper
  2. Move to hover (high, laterally offset from target)
  3. Move to approach (directly above target)
  4. Descend to grasp Z
  5. Close gripper
  6. Retract straight up
"""

from __future__ import annotations

import math
from typing import List

from dobot_ros.strategies.base import (
    ParameterDef, PickContext, PickPlan, PickStrategy, PickWaypoint,
)


class AngledApproach(PickStrategy):
    name = "Angled Approach"
    slug = "angled_approach"
    description = "Approach from an offset direction at an angle, then drop straight to grasp."

    @classmethod
    def parameter_defs(cls) -> List[ParameterDef]:
        return [
            ParameterDef("approach_height_mm", "float", 120.0,
                         "Height above grasp Z for the final approach",
                         min=20, max=300, step=10, unit="mm"),
            ParameterDef("hover_height_mm", "float", 200.0,
                         "Height for the lateral hover waypoint (higher = safer)",
                         min=50, max=400, step=10, unit="mm"),
            ParameterDef("approach_offset_mm", "float", 80.0,
                         "Lateral offset from target for the angled entry",
                         min=10, max=200, step=10, unit="mm"),
            ParameterDef("approach_direction", "choice", "auto",
                         "Direction to approach from. 'auto' picks the direction away from the robot base.",
                         choices=["auto", "from_back", "from_left", "from_right", "from_front"]),
            ParameterDef("grasp_clearance_mm", "float", 5.0,
                         "Minimum clearance above table at grasp",
                         min=0, max=100, step=1, unit="mm"),
            ParameterDef("lift_height_mm", "float", 150.0,
                         "Lift height after grasping",
                         min=20, max=400, step=10, unit="mm"),
            ParameterDef("pre_rotate", "bool", True,
                         "Rotate wrist to grasp orientation at the hover waypoint (before descent)"),
            ParameterDef("gripper_open_pos", "int", 800,
                         "Gripper position for open",
                         min=0, max=1000, step=50),
            ParameterDef("gripper_close_pos", "int", 200,
                         "Gripper position for close",
                         min=0, max=1000, step=50),
            ParameterDef("gripper_force", "int", 50,
                         "Grip force",
                         min=20, max=100, step=5, unit="%"),
            ParameterDef("move_speed", "int", 25,
                         "Robot speed factor during the pick sequence",
                         min=1, max=100, step=5, unit="%"),
        ]

    def _compute_offset_direction(self, ctx: PickContext, direction: str) -> tuple:
        """Return (dx, dy) unit vector for the approach offset direction."""
        if direction == "auto":
            # Move away from the robot base origin (0, 0) to minimize reach issues.
            dx, dy = ctx.robot_x, ctx.robot_y
            norm = math.sqrt(dx * dx + dy * dy)
            if norm < 10.0:
                dx, dy = -1.0, 0.0  # fallback: from the back
                import logging
                logging.warning("angled_approach: target near base (%.1f,%.1f), using fallback direction 'from_back'",
                                ctx.robot_x, ctx.robot_y)
            else:
                dx, dy = dx / norm, dy / norm
        elif direction == "from_back":
            dx, dy = -1.0, 0.0
        elif direction == "from_front":
            dx, dy = 1.0, 0.0
        elif direction == "from_left":
            dx, dy = 0.0, 1.0
        elif direction == "from_right":
            dx, dy = 0.0, -1.0
        else:
            dx, dy = -1.0, 0.0
        return dx, dy

    def plan(self, ctx: PickContext) -> PickPlan:
        p = ctx.params

        approach_h = float(p.get("approach_height_mm", 120))
        hover_h = float(p.get("hover_height_mm", 200))
        offset = float(p.get("approach_offset_mm", 80))
        direction = str(p.get("approach_direction", "auto"))
        clearance = float(p.get("grasp_clearance_mm", 5))
        lift_h = float(p.get("lift_height_mm", 150))
        pre_rotate = bool(p.get("pre_rotate", True))
        open_pos = int(p.get("gripper_open_pos", 800))
        close_pos = int(p.get("gripper_close_pos", 200))
        speed = int(p.get("move_speed", 25))

        # SAFETY: always enforce min_clearance_mm as the absolute floor.
        grasp_z = ctx.table_z + max(clearance, ctx.min_clearance_mm)
        if ctx.object_present and ctx.object_height_mm > 10:
            height_z = ctx.table_z + ctx.object_height_mm / 2.0
            grasp_z = max(grasp_z, height_z)  # only raise, never lower below clearance

        approach_z = grasp_z + approach_h
        hover_z = grasp_z + hover_h
        retract_z = grasp_z + lift_h

        rx, ry = ctx.current_pose[3], ctx.current_pose[4]
        rz_target = ctx.rotation_deg
        rz_hover = rz_target if pre_rotate else ctx.current_pose[5]

        # Offset direction.
        dx, dy = self._compute_offset_direction(ctx, direction)
        hover_x = ctx.robot_x + dx * offset
        hover_y = ctx.robot_y + dy * offset

        return PickPlan(
            move_speed=speed,
            waypoints=[
                PickWaypoint(
                    pose=[hover_x, hover_y, hover_z, rx, ry, rz_hover],
                    label=f"Hover (offset {direction})",
                    gripper_action="open", gripper_pos=open_pos,
                ),
                PickWaypoint(
                    pose=[ctx.robot_x, ctx.robot_y, approach_z, rx, ry, rz_target],
                    label="Approach (above target)",
                ),
                PickWaypoint(
                    pose=[ctx.robot_x, ctx.robot_y, grasp_z, rx, ry, rz_target],
                    label="Descend to grasp",
                    pause_s=0.2,
                ),
                PickWaypoint(
                    pose=[ctx.robot_x, ctx.robot_y, grasp_z, rx, ry, rz_target],
                    label="Close gripper",
                    gripper_action="close", gripper_pos=close_pos,
                    pause_s=0.3,
                ),
                PickWaypoint(
                    pose=[ctx.robot_x, ctx.robot_y, retract_z, rx, ry, rz_target],
                    label="Retract (lift)",
                ),
            ],
        )
