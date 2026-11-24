"""Stack one colored block on top of another.

Second hand-coded policy — exists to prove the primitives compose beyond a
single pick: pick block A, carry it over block B, release it on B's top
face. Runs unchanged on the simulator and the arm. See POLICIES.md.
"""

from __future__ import annotations

import math
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
            ParameterDef("drop_mm", "float", 15.0, "Gap between the two blocks when releasing (absorbs base-height error)", 0, 40, 1, unit="mm"),
            ParameterDef("look_frames", "int", 3, "Frames averaged per look (√n noise reduction)", 1, 10, 1),
            ParameterDef("open_margin_mm", "float", 25.0, "Extra jaw opening beyond the block width", 0, 60, 1, unit="mm"),
            ParameterDef("force", "int", 20, "Gripper force", 20, 100, 5, unit="%"),
            ParameterDef("max_attempts", "int", 3, "Grasp attempts before giving up", 1, 5, 1),
        ]

    def run(self, robot: Robot, world: World, p: dict, trace: Trace) -> PolicyResult:
        t0 = time.monotonic()
        if p["top_color"] == p["base_color"]:
            return PolicyResult(ok=False, reason="top and base colors must differ", attempts=0)
        robot.set_tool(1)
        n = int(p["look_frames"])
        try:
            top = P.look_avg(world, trace, n=n, color=p["top_color"], label="block")
            base = P.look_avg(world, trace, n=n, color=p["base_color"], label="block")
        except NotFound as e:
            return PolicyResult(ok=False, reason=str(e), attempts=0)

        surface = world.table_z - robot.tool_length_mm
        attempts = 0
        last_miss = ""
        jaw = None
        while attempts < int(p["max_attempts"]):
            attempts += 1
            try:
                grasp_h = P.grasp_height(top, world, robot, float(p["grasp_height_mm"]))
                jaw_yaw, opening, clear = P.choose_grasp(top, world.objects(), float(p["open_margin_mm"]))
            except P.GraspInfeasible as e:
                return PolicyResult(ok=False, reason=str(e), attempts=attempts - 1,
                                    elapsed_s=time.monotonic() - t0)
            width = top.width_across(jaw_yaw)
            trace.log(f"grasp plan: yaw {jaw_yaw:.0f}° opening {opening:.0f} mm (clearance {clear:.0f} mm)")
            P.open_gripper(robot, trace, opening)
            P.hover_above(robot, world, trace, top.xy, float(p["approach_mm"]), jaw_yaw)
            P.descend_to(robot, world, trace, grasp_h)
            try:
                jaw = P.grasp(robot, trace, width_hint_mm=width, force=int(p["force"]))
                break
            except GraspMissed as e:
                why = P.explain_miss(top, world, robot, grasp_h)
                trace.log(f"attempt {attempts}: {e} — {why}")
                last_miss = why
                P.lift(robot, world, trace, float(p["approach_mm"]))
                try:
                    top = P.look_avg(world, trace, n=n, color=p["top_color"], label="block")
                except NotFound as e2:
                    return PolicyResult(ok=False, reason=f"lost top block: {e2}", attempts=attempts,
                                        elapsed_s=time.monotonic() - t0)
        if jaw is None:
            return PolicyResult(ok=False, reason=f"grasp missed {attempts} times: {last_miss}", attempts=attempts,
                                elapsed_s=time.monotonic() - t0)

        P.lift(robot, world, trace, float(p["carry_mm"]))

        # In-hand re-observation: the block rarely sits dead-centre between the
        # fingers (perception error at grasp time), so measure where it actually
        # is relative to the fingertip and place the *block*, not the tool.
        off = [0.0, 0.0]
        try:
            held = P.look_avg(world, trace, n=n, color=p["top_color"], label="block")
            cur = robot.pose()
            off = [held.position[0] - cur[0], held.position[1] - cur[1]]
            if math.hypot(*off) > 30.0:   # implausible — ignore rather than trust
                trace.log(f"in-hand offset {off} implausible, ignoring")
                off = [0.0, 0.0]
            else:
                trace.log(f"in-hand offset ({off[0]:.1f}, {off[1]:.1f}) mm")
        except NotFound:
            trace.log("held block not visible; assuming centred")

        # Fresh, averaged look at the base right before placing.
        try:
            base = P.look_avg(world, trace, n=n, color=p["base_color"], label="block")
        except NotFound:
            trace.log("base not re-observed; using the initial estimate")

        # The fingertip is grasp_h above the block's bottom. Release so the
        # bottom lands drop_mm above the base's (noisy) top.
        base_top_h = base.top_z - surface
        release_h = base_top_h + float(p["drop_mm"]) + grasp_h
        target_xy = [base.position[0] - off[0], base.position[1] - off[1]]
        place_yaw = base.yaw_deg + (jaw_yaw - top.yaw_deg)  # keep the same relative jaw/block alignment
        P.place_at(robot, world, trace, target_xy, release_h, float(p["carry_mm"]), place_yaw)

        # Verify: the top block should now be reported above the base.
        obs = {}
        try:
            after = P.look_avg(world, trace, n=n, color=p["top_color"], label="block")
            dx = ((after.position[0] - base.position[0]) ** 2 + (after.position[1] - base.position[1]) ** 2) ** 0.5
            # Centre must land on the base footprint for the stack to stand.
            stacked = dx < min(base.size[0], base.size[1]) / 2.0 and after.bottom_z > base.position[2]
            obs = {"xy_error_mm": round(dx, 1), "stacked": stacked, "jaw_width_mm": round(jaw, 1)}
            if not stacked:
                return PolicyResult(ok=False, reason=f"top block not on base after place (xy err {dx:.0f} mm)",
                                    attempts=attempts, observations=obs, elapsed_s=time.monotonic() - t0)
        except NotFound:
            obs = {"verify": "top block not visible after place"}
        return PolicyResult(ok=True, attempts=attempts, observations=obs, elapsed_s=time.monotonic() - t0)
