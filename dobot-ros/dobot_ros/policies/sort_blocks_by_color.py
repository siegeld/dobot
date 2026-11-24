"""Sort every block on the table into colour zones.

Zones are laid out along the near edge of the workspace (the edge closest
to the robot base, where reach is best): one zone per colour seen, zones
spaced along Y, up to ``slots_per_zone`` blocks in a row along X inside
each zone. The policy loops: look at everything, pick the nearest block
that is not inside its own colour's zone, move it to that zone's next free
slot, re-look, until nothing is out of place. A block it cannot grasp
right now (boxed in by neighbours) is skipped and retried after the
others have moved. See POLICIES.md.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence

from dobot_ros.strategies.base import ParameterDef
from dobot_ros.policies.base import Policy
from dobot_ros.policies.types import (
    GraspMissed, NotFound, PolicyResult, Robot, Trace, Unreachable, World, WorldObject,
)
from dobot_ros.policies import primitives as P

PLACE_YAW = 90.0   # fingers spread along Y at release: neighbours in a zone sit along X


@dataclass
class Zone:
    color: str
    cy: float
    slots: List[List[float]]
    half_x: float
    half_y: float

    @property
    def cx(self) -> float:
        return sum(s[0] for s in self.slots) / len(self.slots)

    def contains(self, xy: Sequence[float]) -> bool:
        return abs(xy[0] - self.cx) <= self.half_x and abs(xy[1] - self.cy) <= self.half_y

    def on_slot(self, xy: Sequence[float], tol_mm: float) -> bool:
        """Sorted = parked on one of this zone's slots, not merely inside the strip."""
        return any(math.dist(s, xy) <= tol_mm for s in self.slots)

    def free_slot(self, objects: List[WorldObject], bad: set, carried: WorldObject,
                  opening_mm: float, finger_w_mm: float = P.FINGER_W_MM,
                  margin_mm: float = 0.0, carried_dims: Sequence[float] = (40.0, 40.0)) -> Optional[List[float]]:
        """First slot where the carried block's footprint and the open finger
        pads (spread along Y at release) clear every block on the table."""
        # After release the block's grasped axis lies along Y (PLACE_YAW), so
        # its footprint at the slot is axis-aligned: width across the jaw
        # axis along Y, the other side along X.
        for i, s in enumerate(self.slots):
            if (self.color, i) in bad:
                continue
            ok = True
            for o in objects:
                if o.id == carried.id:
                    continue
                if not P.boxes_clear(s, carried_dims, o, margin_mm):
                    ok = False; break
                for sgn in (-1.0, 1.0):
                    pad = (s[0], s[1] + sgn * (opening_mm / 2.0 + finger_w_mm / 2.0))
                    if P._rect_clearance(pad[0], pad[1], o) < finger_w_mm / 2.0 + 2.0 + margin_mm:
                        ok = False; break
                if not ok:
                    break
            if ok:
                return s
        return None


class SortBlocksByColor(Policy):
    name = "Sort blocks by color"
    slug = "sort_blocks_by_color"
    description = ("Move every block into a zone for its colour along the near edge of the table, "
                   "one zone per colour, blocks in a row inside each zone. Loops look → pick → place "
                   "until nothing is out of place; blocks it cannot reach between neighbours are "
                   "retried after the others have moved.")

    @classmethod
    def parameter_defs(cls) -> List[ParameterDef]:
        return [
            ParameterDef("approach_mm", "float", 80.0, "Hover height above the table", 20, 300, 5, unit="mm"),
            ParameterDef("grasp_height_mm", "float", 20.0, "Fingertip height above the table when closing", 0, 100, 1, unit="mm"),
            ParameterDef("carry_mm", "float", 120.0, "Carry height above the table", 50, 400, 5, unit="mm"),
            ParameterDef("drop_mm", "float", 8.0, "Height of the block's bottom above the table at release", 0, 40, 1, unit="mm"),
            ParameterDef("open_margin_mm", "float", 25.0, "Extra jaw opening beyond the block width at pick", 0, 60, 1, unit="mm"),
            ParameterDef("force", "int", 20, "Gripper force", 20, 100, 5, unit="%"),
            ParameterDef("zone_pitch_mm", "float", 120.0, "Spacing between colour zones (along Y)", 80, 250, 5, unit="mm"),
            ParameterDef("slot_pitch_mm", "float", 60.0, "Spacing between blocks inside a zone (along X)", 45, 120, 5, unit="mm"),
            ParameterDef("slots_per_zone", "int", 3, "Blocks per zone", 1, 6, 1),
            ParameterDef("zone_inset_mm", "float", 70.0, "First slot's distance from the near workspace edge", 40, 200, 5, unit="mm"),
            ParameterDef("place_clearance_mm", "float", 8.0, "Extra clearance demanded around a slot before placing", 0, 30, 1, unit="mm"),
            ParameterDef("max_moves", "int", 12, "Pick-and-place budget", 1, 40, 1),
            ParameterDef("look_frames", "int", 3, "Frames averaged per look", 1, 10, 1),
        ]

    # ── layout ──────────────────────────────────────────────────────
    @staticmethod
    def layout(colors: List[str], ws, p: dict) -> Dict[str, Zone]:
        """Zones along Y from the Y-min side, slots along X from the near
        (X-max) edge inward. Raises ValueError if the colours don't fit."""
        pitch, spitch = float(p["zone_pitch_mm"]), float(p["slot_pitch_mm"])
        n_slots, inset = int(p["slots_per_zone"]), float(p["zone_inset_mm"])
        y0 = ws.y_min + inset
        zones: Dict[str, Zone] = {}
        for k, color in enumerate(colors):
            cy = y0 + k * pitch
            if cy + pitch / 2.0 > ws.y_max + 1e-6:
                raise ValueError(f"{len(colors)} colours do not fit in the workspace at {pitch:.0f} mm zone pitch")
            slots = [[ws.x_max - inset - j * spitch, cy] for j in range(n_slots)]
            zones[color] = Zone(color, cy, slots, half_x=(n_slots - 1) * spitch / 2.0 + spitch / 2.0,
                                half_y=pitch / 2.0)
        return zones

    # ── run ─────────────────────────────────────────────────────────
    def run(self, robot: Robot, world: World, p: dict, trace: Trace) -> PolicyResult:
        t0 = time.monotonic()
        n = int(p["look_frames"])
        slot_tol = float(p["slot_pitch_mm"]) * 0.25     # seated = within 15 mm of a slot centre at 60 mm pitch
        robot.set_tool(1)

        def blocks(frames: int = 1) -> List[WorldObject]:
            """Union of detections over several frames (by id) with positions
            averaged per block, so a single dropout never hides a block and
            a single noisy frame never misplaces one."""
            acc: Dict[str, List[WorldObject]] = {}
            for _ in range(max(1, frames)):
                for o in world.objects():
                    if o.label == "block" and o.support != "gripper":
                        acc.setdefault(o.id, []).append(o)
            out = []
            for samples in acc.values():
                b = samples[0]
                k = len(samples)
                b.position = [sum(x.position[i] for x in samples) / k for i in range(3)]
                b.size = [sum(x.size[i] for x in samples) / k for i in range(3)]
                out.append(b)
            return out

        objs = blocks(3)
        colors = sorted({o.color for o in objs})
        if not colors:
            return PolicyResult(ok=False, reason="no blocks visible", attempts=0)
        try:
            zones = self.layout(colors, getattr(world, "workspace", None) or robot.guards.workspace, p)
        except (ValueError, AttributeError) as e:
            return PolicyResult(ok=False, reason=f"zone layout failed: {e}", attempts=0)
        for z in zones.values():
            trace.log(f"zone {z.color}: y={z.cy:.0f}, slots x={[round(s[0]) for s in z.slots]}")

        def out_of_place(objs_: List[WorldObject]) -> List[WorldObject]:
            return [o for o in objs_ if o.color in zones and not zones[o.color].on_slot(o.xy, slot_tol)]

        moves = misses = 0
        skipped: Dict[str, int] = {}        # id → times set aside this pass
        hopeless: Dict[str, str] = {}       # id → why it can never be sorted
        bad_slots: set = set()
        per_color: Dict[str, int] = {}

        iterations = 0
        while moves < int(p["max_moves"]):
            iterations += 1
            if iterations > int(p["max_moves"]) * 5:
                trace.log("iteration budget exhausted — stopping")
                break
            objs = blocks(n)
            todo = [o for o in out_of_place(objs) if o.id not in hopeless]
            if not todo:
                # Don't trust one frame for "done": re-look before stopping.
                todo = [o for o in out_of_place(blocks(3)) if o.id not in hopeless]
                if not todo:
                    break
            # Prefer blocks not yet set aside; a block set aside twice is given up on.
            for o in todo:
                if skipped.get(o.id, 0) >= 2 and o.id not in hopeless:
                    hopeless[o.id] = "could not be grasped or placed after two attempts"
            todo = [o for o in todo if o.id not in hopeless]
            if not todo:
                break
            fresh = [o for o in todo if skipped.get(o.id, 0) == 0]
            cands = fresh or todo
            # Blocks lying inside a zone strip but not on a slot obstruct the
            # layout — move those first, then the one nearest its zone.
            def priority(o: WorldObject):
                obstructs = any(z.contains(o.xy) for z in zones.values())
                return (0 if obstructs else 1, math.dist(o.xy, [zones[o.color].cx, zones[o.color].cy]))
            target = min(cands, key=priority)
            zone = zones[target.color]
            try:
                obj = P.look_avg(world, trace, n=n, id=target.id)
            except NotFound:
                skipped[target.id] = skipped.get(target.id, 0) + 1
                continue
            try:
                grasp_h = P.grasp_height(obj, world, robot, float(p["grasp_height_mm"]))
                jaw_yaw, opening, clear = P.choose_grasp(obj, objs, float(p["open_margin_mm"]))
            except P.GraspInfeasible as e:
                msg = str(e)
                if "clearance floor" in msg or "stroke" in msg:
                    hopeless[obj.id] = msg            # geometry, not neighbours — never
                    trace.log(f"giving up on {obj.color} '{obj.id}': {msg}")
                else:
                    skipped[obj.id] = skipped.get(obj.id, 0) + 1
                    trace.log(f"set aside {obj.color} '{obj.id}' for now: {msg}")
                continue
            width = obj.width_across(jaw_yaw)
            release_open = min(P.GRIPPER_STROKE_MM, width + 12.0)
            # Footprint at the slot: the jaw-axis width ends up along Y.
            other = obj.width_across(jaw_yaw + 90.0)
            slot = zone.free_slot(objs, bad_slots, obj, release_open, margin_mm=float(p["place_clearance_mm"]),
                                  carried_dims=(other, width))
            if slot is None:
                # Something is sitting on the zone; moving the others first
                # usually clears it, so set this one aside rather than give up.
                skipped[obj.id] = skipped.get(obj.id, 0) + 1
                if skipped[obj.id] >= 2:
                    hopeless[obj.id] = f"no clear slot in the {zone.color} zone"
                trace.log(f"no clear slot for {obj.color} '{obj.id}' right now")
                continue
            trace.log(f"move {moves + 1}: {obj.color} '{obj.id}' → slot ({slot[0]:.0f}, {slot[1]:.0f}); "
                      f"grasp yaw {jaw_yaw:.0f}° opening {opening:.0f} mm clearance {clear:.0f} mm")

            # pick
            P.open_gripper(robot, trace, opening)
            P.hover_above(robot, world, trace, obj.xy, float(p["approach_mm"]), jaw_yaw)
            P.descend_to(robot, world, trace, grasp_h)
            try:
                P.grasp(robot, trace, width_hint_mm=width, force=int(p["force"]))
            except GraspMissed as e:
                misses += 1
                why = P.explain_miss(obj, world, robot, grasp_h)
                trace.log(f"miss on '{obj.id}': {e} — {why}")
                P.lift(robot, world, trace, float(p["approach_mm"]))
                if "clearance floor" in why:
                    hopeless[obj.id] = why
                else:
                    skipped[obj.id] = skipped.get(obj.id, 0) + 1
                continue
            P.lift(robot, world, trace, float(p["carry_mm"]))

            # in-hand offset → place the block, not the tool
            off = [0.0, 0.0]
            try:
                held = P.look_avg(world, trace, n=n, id=obj.id)
                cur = robot.pose()
                off = [held.position[0] - cur[0], held.position[1] - cur[1]]
                if math.hypot(*off) > 30.0:
                    off = [0.0, 0.0]
            except NotFound:
                pass
            target_xy = [slot[0] - off[0], slot[1] - off[1]]
            try:
                P.place_at(robot, world, trace, target_xy, grasp_h + float(p["drop_mm"]),
                           float(p["carry_mm"]), PLACE_YAW, open_width_mm=release_open)
            except Unreachable as e:
                # Slot out of reach: release where we are (still above the
                # table, nothing below) and retire that slot.
                trace.log(f"slot unreachable ({e}); retiring it")
                bad_slots.add((zone.color, zone.slots.index(slot)))
                P.descend_to(robot, world, trace, grasp_h + float(p["drop_mm"]))
                P.open_gripper(robot, trace, release_open)
                P.lift(robot, world, trace, float(p["approach_mm"]))
                continue
            moves += 1
            per_color[obj.color] = per_color.get(obj.color, 0) + 1

        remaining = out_of_place(blocks(3))
        P.lift(robot, world, trace, float(p["carry_mm"]))
        obs = self._obs(moves, misses, per_color, remaining)
        obs["given_up"] = dict(hopeless)
        if remaining:
            why = "; ".join(f"{o.color} '{o.id}'" + (f" ({hopeless[o.id]})" if o.id in hopeless else "")
                            for o in remaining)
            return PolicyResult(ok=False, reason=f"{len(remaining)} block(s) still out of place after {moves} moves: {why}",
                                attempts=moves, observations=obs, elapsed_s=time.monotonic() - t0)
        return PolicyResult(ok=True, attempts=moves, observations=obs, elapsed_s=time.monotonic() - t0)

    @staticmethod
    def _obs(moves, misses, per_color, unsorted) -> dict:
        return {"moves": moves, "misses": misses, "per_color": per_color, "unsorted": len(unsorted)}
