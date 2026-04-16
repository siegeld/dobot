"""Strategy: Simple Top-Down Pick.

Straight vertical approach from directly above the object. The most basic
and safest strategy — good baseline for testing calibration accuracy.

Waypoints:
  1. Open gripper
  2. Move to approach pose (directly above target, approach_height above grasp Z)
  3. Descend to grasp Z
  4. Close gripper
  5. Retract straight up
"""

from __future__ import annotations

from typing import List

from dobot_ros.strategies.base import (
    ParameterDef, PickContext, PickPlan, PickStrategy, PickWaypoint,
)


class SimpleTopDown(PickStrategy):
    name = "Simple Top-Down"
    slug = "simple_top_down"
    description = "Straight vertical approach from above. Safe baseline for testing."

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

        # Grasp Z: table surface + clearance, optionally adjusted for object height.
        grasp_z = ctx.table_z + max(clearance, ctx.min_clearance_mm)
        if use_height and ctx.object_present and ctx.object_height_mm > 10:
            grasp_z = ctx.table_z + max(clearance, ctx.object_height_mm / 2.0)

        approach_z = grasp_z + approach_h
        retract_z = grasp_z + lift_h

        # Preserve current wrist orientation (RX, RY), set RZ to object rotation.
        rx, ry = ctx.current_pose[3], ctx.current_pose[4]
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
