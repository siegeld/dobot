"""Pick up a block of a given color from the table.

Hand-coded policy. Written against the Robot/World protocols so it runs on
the simulator and on the arm unchanged. See POLICIES.md.
"""

from __future__ import annotations

import time
from typing import List

from dobot_ros.strategies.base import ParameterDef
from dobot_ros.policies.base import Policy
from dobot_ros.policies.types import GraspMissed, NotFound, PolicyResult, Robot, Trace, World
from dobot_ros.policies import primitives as P


class PickColoredBlock(Policy):
    name = "Pick colored block"
    slug = "pick_colored_block"
    description = ("Find a block of the requested color, hover above it, descend, "
                   "close the jaws across its short side, verify the grasp from the "
                   "jaw position, and lift. Retries with a fresh look on a miss.")

    @classmethod
    def parameter_defs(cls) -> List[ParameterDef]:
        return [
            ParameterDef("color", "choice", "red", "Block color to pick",
                         choices=["red", "green", "blue", "yellow", "orange", "purple", "white", "black"]),
            ParameterDef("approach_mm", "float", 80.0, "Hover height above the table", 20, 300, 5, unit="mm"),
            ParameterDef("grasp_height_mm", "float", 20.0, "Fingertip height above the table when closing", 0, 100, 1, unit="mm"),
            ParameterDef("lift_mm", "float", 150.0, "Lift height after a successful grasp", 20, 400, 5, unit="mm"),
            ParameterDef("open_margin_mm", "float", 25.0, "Extra jaw opening beyond the block width", 0, 60, 1, unit="mm"),
            ParameterDef("force", "int", 20, "Gripper force", 20, 100, 5, unit="%"),
            ParameterDef("max_attempts", "int", 3, "Grasp attempts before giving up", 1, 5, 1),
            ParameterDef("look_retries", "int", 3, "Perception re-queries before declaring the block absent", 1, 10, 1),
        ]

    def run(self, robot: Robot, world: World, p: dict, trace: Trace) -> PolicyResult:
        t0 = time.monotonic()
        robot.set_tool(1)
        try:
            obj = P.look(world, trace, retries=int(p["look_retries"]), color=p["color"], label="block")
        except NotFound as e:
            return PolicyResult(ok=False, reason=str(e), attempts=0)

        attempts = 0
        while attempts < int(p["max_attempts"]):
            attempts += 1
            # Grasp height: never below the clearance floor, never above the block top.
            grasp_h = max(float(p["grasp_height_mm"]), world.min_clearance_mm)
            top = obj.top_z - (world.table_z - robot.tool_length_mm)
            grasp_h = min(grasp_h, max(top - 5.0, world.min_clearance_mm))

            jaw_yaw = obj.short_axis_yaw()
            width = obj.width_across(jaw_yaw)
            P.open_gripper(robot, trace, min(P.GRIPPER_STROKE_MM, width + float(p["open_margin_mm"])))
            P.hover_above(robot, world, trace, obj.xy, float(p["approach_mm"]), jaw_yaw)
            P.descend_to(robot, world, trace, grasp_h)
            try:
                jaw = P.grasp(robot, trace, width_hint_mm=width, force=int(p["force"]))
            except GraspMissed as e:
                trace.log(f"attempt {attempts}: {e}")
                P.lift(robot, world, trace, float(p["approach_mm"]))
                try:
                    obj = P.look(world, trace, retries=int(p["look_retries"]), color=p["color"], label="block")
                except NotFound as e2:
                    return PolicyResult(ok=False, reason=f"lost target after miss: {e2}",
                                        attempts=attempts, elapsed_s=time.monotonic() - t0)
                continue
            P.lift(robot, world, trace, float(p["lift_mm"]))
            return PolicyResult(ok=True, attempts=attempts,
                                observations={"jaw_width_mm": round(jaw, 1), "object": obj.id},
                                elapsed_s=time.monotonic() - t0)

        return PolicyResult(ok=False, reason=f"grasp missed {attempts} times",
                            attempts=attempts, elapsed_s=time.monotonic() - t0)
