"""Stack every block on the table into one tower.

The block with the largest footprint is the base and stays where it is
(so a block too flat to grip can still carry the tower), provided the
fingers can release onto it without touching a neighbour; otherwise the
next-largest clear block is used, and if nothing is clear the largest
block is first moved to a clear site. The rest go on in descending
footprint order. Carry height rises with the tower and all travel between
the pile and the tower happens at carry height so the fingers never clip
the tower's edge. Each placement is verified by re-observing the block; a
block that fell off is re-queued, twice, then given up on. See POLICIES.md.
"""

from __future__ import annotations

import math
import time
from typing import Dict, List, Optional

from dobot_ros.strategies.base import ParameterDef
from dobot_ros.policies.base import Policy
from dobot_ros.policies.types import GraspMissed, NotFound, PolicyResult, Robot, Trace, Unreachable, World, WorldObject
from dobot_ros.policies import primitives as P

PLACE_YAW = 90.0


def _area(o: WorldObject) -> float:
    return o.size[0] * o.size[1]


class StackAllBlocks(Policy):
    name = "Stack all blocks into a tower"
    slug = "stack_all_blocks"
    description = ("Build one tower from every block on the table: the biggest block is the base, the "
                   "rest go on largest-first. Carry height follows the tower, every placement is "
                   "checked by re-observing the block, and a block that fell off is retried.")

    @classmethod
    def parameter_defs(cls) -> List[ParameterDef]:
        return [
            ParameterDef("approach_mm", "float", 80.0, "Hover height above the table for a pick", 20, 300, 5, unit="mm"),
            ParameterDef("grasp_height_mm", "float", 20.0, "Fingertip height above a block's bottom when closing", 0, 100, 1, unit="mm"),
            ParameterDef("clearance_mm", "float", 40.0, "Carry clearance between the carried block's bottom and the tower top", 15, 150, 5, unit="mm"),
            ParameterDef("drop_mm", "float", 18.0, "Gap between the carried block and the tower top at release", 0, 40, 1, unit="mm"),
            ParameterDef("open_margin_mm", "float", 25.0, "Extra jaw opening beyond the block width at pick", 0, 60, 1, unit="mm"),
            ParameterDef("force", "int", 20, "Gripper force", 20, 100, 5, unit="%"),
            ParameterDef("place_clearance_mm", "float", 8.0, "Clearance demanded around the tower site before building on it", 0, 30, 1, unit="mm"),
            ParameterDef("max_tower_mm", "float", 400.0, "Stop adding blocks once the tower would exceed this", 40, 600, 10, unit="mm"),
            ParameterDef("max_moves", "int", 12, "Pick-and-place budget", 1, 40, 1),
            ParameterDef("look_frames", "int", 3, "Frames averaged per look", 1, 10, 1),
        ]

    def run(self, robot: Robot, world: World, p: dict, trace: Trace) -> PolicyResult:
        t0 = time.monotonic()
        n = int(p["look_frames"])
        robot.set_tool(1)
        surface = world.table_z - robot.tool_length_mm

        def blocks(frames: int = 1) -> List[WorldObject]:
            acc: Dict[str, List[WorldObject]] = {}
            for _ in range(max(1, frames)):
                for o in world.objects():
                    if o.label == "block" and o.support != "gripper":
                        acc.setdefault(o.id, []).append(o)
            out = []
            for samples in acc.values():
                b = samples[0]; k = len(samples)
                b.position = [sum(x.position[i] for x in samples) / k for i in range(3)]
                b.size = [sum(x.size[i] for x in samples) / k for i in range(3)]
                out.append(b)
            return out

        objs = blocks(n)
        if not objs:
            return PolicyResult(ok=False, reason="no blocks visible", attempts=0)
        margin = float(p["place_clearance_mm"])

        # ── choose the base ─────────────────────────────────────────
        base: Optional[WorldObject] = None
        for cand in sorted(objs, key=_area, reverse=True):
            # Release clearance for the widest block that will go on top.
            widest = max((o for o in objs if o.id != cand.id), key=lambda o: max(o.size[:2]), default=cand)
            dims = (max(widest.size[:2]), max(widest.size[:2]))
            if P.site_clear(cand.xy, dims, objs, min(P.GRIPPER_STROKE_MM, max(widest.size[:2]) + 12.0), margin,
                            exclude=[cand.id], jaw_yaw_deg=PLACE_YAW):
                base = cand
                break
        moves = 0
        if base is None:
            # Nothing is clear: move the largest block to a clear site first.
            big = max(objs, key=_area)
            site = self._find_clear_site(world, robot, objs, big, margin)
            if site is None:
                return PolicyResult(ok=False, reason="no clear site for a tower base", attempts=0)
            trace.log(f"no clear base; moving {big.color} '{big.id}' to ({site[0]:.0f}, {site[1]:.0f})")
            ok, why = self._pick_and_place(robot, world, trace, p, big, site, surface, objs, 0.0)
            if not ok:
                return PolicyResult(ok=False, reason=f"could not move base block: {why}", attempts=1)
            moves += 1
            objs = blocks(n)
            base = next((o for o in objs if o.id == big.id), None) or big
        trace.log(f"base: {base.color} '{base.id}' at ({base.xy[0]:.0f}, {base.xy[1]:.0f}), "
                  f"{base.size[0]:.0f}×{base.size[1]:.0f}×{base.size[2]:.0f}")

        # ── stack the rest, largest first ───────────────────────────
        tower_ids = [base.id]
        hopeless: Dict[str, str] = {}
        fails: Dict[str, int] = {}
        misses = 0
        iterations = 0
        while moves < int(p["max_moves"]):
            iterations += 1
            if iterations > int(p["max_moves"]) * 4:
                trace.log("iteration budget exhausted"); break
            objs = blocks(n)
            top = self._tower_top(objs, tower_ids, base, surface)
            todo = [o for o in objs if o.id not in tower_ids and o.id not in hopeless]
            if not todo:
                todo = [o for o in blocks(3) if o.id not in tower_ids and o.id not in hopeless]
                if not todo:
                    break
            todo.sort(key=lambda o: (-_area(o), math.dist(o.xy, base.xy)))
            obj = todo[0]
            if top + obj.size[2] > float(p["max_tower_mm"]) + 10.0:   # +10: perceived heights are noisy
                hopeless[obj.id] = f"tower would exceed {p['max_tower_mm']:.0f} mm"
                trace.log(f"skipping {obj.color} '{obj.id}': {hopeless[obj.id]}")
                continue
            trace.log(f"move {moves + 1}: {obj.color} '{obj.id}' onto the tower (top at {top:.0f} mm)")
            ok, why = self._pick_and_place(robot, world, trace, p, obj, base.xy, surface, objs, top)
            if not ok:
                if "clearance floor" in why or "stroke" in why:
                    hopeless[obj.id] = why
                    trace.log(f"giving up on {obj.color} '{obj.id}': {why}")
                    continue
                if why.startswith("miss"):
                    misses += 1
                fails[obj.id] = fails.get(obj.id, 0) + 1
                trace.log(f"'{obj.id}' not stacked ({why}); attempt {fails[obj.id]}")
                if fails[obj.id] >= 2:
                    hopeless[obj.id] = why
                continue
            moves += 1
            # Verify: the block should now sit in the tower column and off the
            # table. (Exact height is not required — perceived heights are the
            # noisiest thing the camera gives us.)
            try:
                placed = P.look_avg(world, trace, n=n, id=obj.id)
                # "Off the table" relative to the base's own top, so a flat base
                # (20 mm) does not make every correct placement look fallen.
                on_tower = (math.dist(placed.xy, base.xy) < max(base.size[:2]) / 2.0
                            and placed.bottom_z - surface > (base.top_z - surface) - 12.0)
            except NotFound:
                on_tower = True  # cannot see it (occluded by the arm) — trust the place
            if on_tower:
                tower_ids.append(obj.id)
            else:
                fails[obj.id] = fails.get(obj.id, 0) + 1
                trace.log(f"'{obj.id}' is not on the tower after placing (it fell?); attempt {fails[obj.id]}")
                if fails[obj.id] >= 2:
                    hopeless[obj.id] = "fell off twice"

        objs = blocks(3)
        top = self._tower_top(objs, tower_ids, base, surface)
        left = [o for o in objs if o.id not in tower_ids]
        obs = {"moves": moves, "misses": misses, "tower_blocks": len(tower_ids), "tower_height_mm": round(top),
               "order": tower_ids, "given_up": hopeless}
        # park above the tower at a safe height
        try:
            P.lift(robot, world, trace, top + float(p["clearance_mm"]) + 40.0)
        except Exception:
            pass
        if left:
            why = "; ".join(f"{o.color} '{o.id}'" + (f" ({hopeless[o.id]})" if o.id in hopeless else "") for o in left)
            return PolicyResult(ok=False, reason=f"{len(left)} block(s) not in the tower after {moves} moves: {why}",
                                attempts=moves, observations=obs, elapsed_s=time.monotonic() - t0)
        return PolicyResult(ok=True, attempts=moves, observations=obs, elapsed_s=time.monotonic() - t0)

    # ── helpers ─────────────────────────────────────────────────────
    @staticmethod
    def _tower_top(objs: List[WorldObject], tower_ids: List[str], base: WorldObject, surface: float) -> float:
        """Height of the tower's top above the table: the observed top of the
        block placed last. (Not the max over every tower block — the max of
        several noisy readings is biased high, which once made the policy
        think a correctly placed block had fallen off.)"""
        by_id = {o.id: o for o in objs}
        for tid in reversed(tower_ids):
            if tid in by_id:
                return by_id[tid].top_z - surface
        return base.top_z - surface

    @staticmethod
    def _find_clear_site(world, robot, objs: List[WorldObject], block: WorldObject, margin: float):
        ws = getattr(world, "workspace", None) or robot.guards.workspace
        dims = (max(block.size[:2]), max(block.size[:2]))
        opening = min(P.GRIPPER_STROKE_MM, max(block.size[:2]) + 12.0)
        cy = (ws.y_min + ws.y_max) / 2.0
        for dx in (80, 140, 200, 260):
            for dy in (0, -100, 100, -200, 200, -300, 300):
                site = [ws.x_max - dx, cy + dy]
                if not ws.contains(site[0], site[1]):
                    continue
                if P.site_clear(site, dims, objs, opening, margin, exclude=[block.id], jaw_yaw_deg=PLACE_YAW):
                    return site
        return None

    def _pick_and_place(self, robot, world, trace, p, obj: WorldObject, target_xy, surface: float,
                        objs: List[WorldObject], top_mm: float):
        """Pick ``obj`` and release it with its bottom ``drop_mm`` above
        ``top_mm`` at ``target_xy``. Returns (ok, reason)."""
        n = int(p["look_frames"])
        try:
            obj = P.look_avg(world, trace, n=n, id=obj.id)
        except NotFound:
            return False, "not visible"
        try:
            grasp_h = P.grasp_height(obj, world, robot, float(p["grasp_height_mm"]))
            jaw_yaw, opening, clear = P.choose_grasp(obj, objs, float(p["open_margin_mm"]))
        except P.GraspInfeasible as e:
            return False, str(e)
        width = obj.width_across(jaw_yaw)
        carry_h = max(float(p["approach_mm"]), top_mm + grasp_h + float(p["clearance_mm"]))
        trace.log(f"grasp yaw {jaw_yaw:.0f}° opening {opening:.0f} mm (clearance {clear:.0f} mm); carry at {carry_h:.0f} mm")
        P.open_gripper(robot, trace, opening)
        # Travel at carry height, then drop onto the pick.
        P.hover_above(robot, world, trace, obj.xy, carry_h, jaw_yaw)
        P.hover_above(robot, world, trace, obj.xy, float(p["approach_mm"]), jaw_yaw)
        P.descend_to(robot, world, trace, grasp_h)
        try:
            P.grasp(robot, trace, width_hint_mm=width, force=int(p["force"]))
        except GraspMissed as e:
            P.lift(robot, world, trace, float(p["approach_mm"]))
            return False, "miss: " + P.explain_miss(obj, world, robot, grasp_h)
        P.lift(robot, world, trace, carry_h)
        off = [0.0, 0.0]
        try:
            held = P.look_avg(world, trace, n=n, id=obj.id)
            cur = robot.pose()
            off = [held.position[0] - cur[0], held.position[1] - cur[1]]
            if math.hypot(*off) > 30.0:
                off = [0.0, 0.0]
        except NotFound:
            pass
        release_h = top_mm + float(p["drop_mm"]) + grasp_h
        try:
            P.place_at(robot, world, trace, [target_xy[0] - off[0], target_xy[1] - off[1]], release_h, carry_h,
                       PLACE_YAW, open_width_mm=min(P.GRIPPER_STROKE_MM, width + 12.0))
        except Unreachable as e:
            trace.log(f"tower site unreachable ({e}); releasing here")
            P.descend_to(robot, world, trace, grasp_h + float(p["drop_mm"]))
            P.open_gripper(robot, trace, min(P.GRIPPER_STROKE_MM, width + 12.0))
            P.lift(robot, world, trace, carry_h)
            return False, "tower site unreachable"
        return True, ""
