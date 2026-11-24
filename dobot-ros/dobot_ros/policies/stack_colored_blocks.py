"""Stack one colored block on top of another.

Second hand-coded policy — exists to prove the primitives compose beyond a
single pick: pick block A, carry it over block B, release it on B's top
face. Runs unchanged on the simulator and the arm. See POLICIES.md.
"""

from __future__ import annotations

import time
from typing import List

from dobot_ros.strategies.base import ParameterDef
from dobot_ros.policies.base import Policy
from dobot_ros.policies.types import GraspMissed, NotFound, PolicyResult, Robot, Trace, World
from dobot_ros.policies import primitives as P

_COLORS = ["red", "green", "blue", "yellow", "orange", "purple", "white", "black"]


class StackColoredBlocks(Policy):
    name = "Stack colored blocks"
    slug = "stack_colored_blocks"
    description = ("Pick the block of color A and place it on top of the block of color B. "
                   "Release height comes from B's measured top plus a small drop; the jaws "
                   "are aligned to A's short side for the grasp and to B's yaw for the place.")

    @classmethod
    def parameter_defs(cls) -> List[ParameterDef]:
        return [
            ParameterDef("top_color", "choice", "red", "Block to pick up (goes on top)", choices=_COLORS),
            ParameterDef("base_color", "choice", "blue", "Block to stack onto", choices=_COLORS),
            ParameterDef("approach_mm", "float", 80.0, "Hover height above the table", 20, 300, 5, unit="mm"),
            ParameterDef("grasp_height_mm", "float", 20.0, "Fingertip height above the table when closing", 0, 100, 1, unit="mm"),
            ParameterDef("carry_mm", "float", 160.0, "Carry height above the table", 50, 400, 5, unit="mm"),
            ParameterDef("drop_mm", "float", 5.0, "Gap between the two blocks when releasing", 0, 30, 1, unit="mm"),
            ParameterDef("open_margin_mm", "float", 25.0, "Extra jaw opening beyond the block width", 0, 60, 1, unit="mm"),
            ParameterDef("force", "int", 20, "Gripper force", 20, 100, 5, unit="%"),
            ParameterDef("max_attempts", "int", 3, "Grasp attempts before giving up", 1, 5, 1),
        ]

    def run(self, robot: Robot, world: World, p: dict, trace: Trace) -> PolicyResult:
        t0 = time.monotonic()
        if p["top_color"] == p["base_color"]:
            return PolicyResult(ok=False, reason="top and base colors must differ", attempts=0)
        robot.set_tool(1)
        try:
            top = P.look(world, trace, color=p["top_color"], label="block")
            base = P.look(world, trace, color=p["base_color"], label="block")
        except NotFound as e:
            return PolicyResult(ok=False, reason=str(e), attempts=0)

        surface = world.table_z - robot.tool_length_mm
        attempts = 0
        jaw = None
        while attempts < int(p["max_attempts"]):
            attempts += 1
            grasp_h = max(float(p["grasp_height_mm"]), world.min_clearance_mm)
            grasp_h = min(grasp_h, max(top.top_z - surface - 5.0, world.min_clearance_mm))
            jaw_yaw = top.short_axis_yaw()
            width = top.width_across(jaw_yaw)
            P.open_gripper(robot, trace, min(P.GRIPPER_STROKE_MM, width + float(p["open_margin_mm"])))
            P.hover_above(robot, world, trace, top.xy, float(p["approach_mm"]), jaw_yaw)
            P.descend_to(robot, world, trace, grasp_h)
            try:
                jaw = P.grasp(robot, trace, width_hint_mm=width, force=int(p["force"]))
                break
            except GraspMissed as e:
                trace.log(f"attempt {attempts}: {e}")
                P.lift(robot, world, trace, float(p["approach_mm"]))
                try:
                    top = P.look(world, trace, color=p["top_color"], label="block")
                except NotFound as e2:
                    return PolicyResult(ok=False, reason=f"lost top block: {e2}", attempts=attempts,
                                        elapsed_s=time.monotonic() - t0)
        if jaw is None:
            return PolicyResult(ok=False, reason=f"grasp missed {attempts} times", attempts=attempts,
                                elapsed_s=time.monotonic() - t0)

        # Carry above the base; the held block's bottom is grasp_h below the
        # fingertip... no: the fingertip is grasp_h above the table and the block
        # bottom sits on the table, so bottom = fingertip - grasp_h. Release so
        # the bottom lands drop_mm above the base's top.
        P.lift(robot, world, trace, float(p["carry_mm"]))
        base_top_h = base.top_z - surface
        release_h = base_top_h + float(p["drop_mm"]) + grasp_h
        place_yaw = base.yaw_deg + (jaw_yaw - top.yaw_deg)  # keep the same relative jaw/block alignment
        P.place_at(robot, world, trace, base.xy, release_h, float(p["carry_mm"]), place_yaw)

        # Verify: the top block should now be reported above the base.
        obs = {}
        try:
            after = P.look(world, trace, color=p["top_color"], label="block")
            dx = ((after.position[0] - base.position[0]) ** 2 + (after.position[1] - base.position[1]) ** 2) ** 0.5
            stacked = dx < 25.0 and after.bottom_z > base.position[2]
            obs = {"xy_error_mm": round(dx, 1), "stacked": stacked, "jaw_width_mm": round(jaw, 1)}
            if not stacked:
                return PolicyResult(ok=False, reason=f"top block not on base after place (xy err {dx:.0f} mm)",
                                    attempts=attempts, observations=obs, elapsed_s=time.monotonic() - t0)
        except NotFound:
            obs = {"verify": "top block not visible after place"}
        return PolicyResult(ok=True, attempts=attempts, observations=obs, elapsed_s=time.monotonic() - t0)
