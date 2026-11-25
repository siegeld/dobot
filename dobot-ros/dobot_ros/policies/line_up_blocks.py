"""Line every block on the table up in one straight, evenly spaced row.

The row runs parallel to the nearest workspace edge (where reach is best),
``row_inset_mm`` inside it, centred along that edge. Slot pitch is the
longest block side plus ``gap_mm``, so neighbours never touch. Each block
is assigned a slot up front (``order``: largest-first, grouped by colour,
or whatever needs the fewest moves) and the row is filled slot by slot. A
block sitting where the next slot goes is moved out of the way first — to
its own slot if that is clear, otherwise to a parking spot off the row.
Every placement is verified by re-observing the block; a block that is not
on its slot (or not square to the row) afterwards is retried, twice, then
given up on with a reason. See POLICIES.md.
"""

from __future__ import annotations

import math
import time
from typing import Dict, List, Optional, Sequence, Tuple

from dobot_ros.strategies.base import ParameterDef
from dobot_ros.policies.base import Policy
from dobot_ros.policies.types import GraspMissed, NotFound, PolicyResult, Robot, Trace, Unreachable, World, WorldObject
from dobot_ros.policies import primitives as P


def _area(o: WorldObject) -> float:
    return o.size[0] * o.size[1]


def _yaw_err(yaw_deg: float, ref_deg: float) -> float:
    """Smallest angle between two axis-aligned frames (mod 90°)."""
    return abs(((yaw_deg - ref_deg + 45.0) % 90.0) - 45.0)


class LineUpBlocks(Policy):
    name = "Line up all blocks in a row"
    slug = "line_up_blocks"
    description = ("Arrange every block on the table in one straight, evenly spaced row along the near "
                   "edge of the workspace, square to the row. Blocks in the way of a slot are moved "
                   "aside first; every placement is checked by re-observing the block.")

    @classmethod
    def parameter_defs(cls) -> List[ParameterDef]:
        return [
            ParameterDef("approach_mm", "float", 80.0, "Hover height above the table", 20, 300, 5, unit="mm"),
            ParameterDef("grasp_height_mm", "float", 20.0, "Fingertip height above a block's bottom when closing", 0, 100, 1, unit="mm"),
            ParameterDef("carry_mm", "float", 120.0, "Carry height above the table", 50, 400, 5, unit="mm"),
            ParameterDef("drop_mm", "float", 8.0, "Height of the block's bottom above the table at release", 0, 40, 1, unit="mm"),
            ParameterDef("open_margin_mm", "float", 25.0, "Extra jaw opening beyond the block width at pick", 0, 60, 1, unit="mm"),
            ParameterDef("force", "int", 20, "Gripper force", 20, 100, 5, unit="%"),
            ParameterDef("row_axis", "choice", "y", "Direction the row runs", choices=["y", "x"]),
            ParameterDef("row_inset_mm", "float", 70.0, "Row's distance from the near workspace edge", 40, 250, 5, unit="mm"),
            ParameterDef("gap_mm", "float", 20.0, "Space between neighbouring blocks", 10, 80, 1, unit="mm"),
            ParameterDef("order", "choice", "size", "Order along the row", choices=["size", "color", "any"]),
            ParameterDef("align_tol_deg", "float", 15.0, "How far off square a block may sit and still count as lined up", 3, 45, 1, unit="deg"),
            ParameterDef("place_clearance_mm", "float", 8.0, "Extra clearance demanded around a slot before placing", 0, 30, 1, unit="mm"),
            ParameterDef("max_moves", "int", 20, "Pick-and-place budget", 1, 60, 1),
            ParameterDef("look_frames", "int", 3, "Frames averaged per look", 1, 10, 1),
        ]

    # ── layout ──────────────────────────────────────────────────────
    @staticmethod
    def layout(n: int, longest_mm: float, ws, p: dict) -> Tuple[List[List[float]], float, float]:
        """Slot centres for ``n`` blocks, the pitch, and the release jaw yaw.

        Along Y at x = x_max − inset (default) or along X at y = y_max − inset;
        centred on the workspace span. Raises ValueError if the row does not fit.
        """
        pitch = float(longest_mm) + float(p["gap_mm"])
        inset = float(p["row_inset_mm"])
        edge_margin = 40.0
        if p["row_axis"] == "y":
            lo, hi, cross = ws.y_min, ws.y_max, ws.x_max - inset
        else:
            lo, hi, cross = ws.x_min, ws.x_max, ws.y_max - inset
        length = (n - 1) * pitch
        if length > (hi - lo) - 2 * edge_margin + 1e-6:
            raise ValueError(f"{n} blocks at {pitch:.0f} mm pitch need {length:.0f} mm; the workspace "
                             f"offers {(hi - lo) - 2 * edge_margin:.0f} mm along {p['row_axis'].upper()}")
        mid = (lo + hi) / 2.0
        coords = [mid - length / 2.0 + k * pitch for k in range(n)]
        if p["row_axis"] == "y":
            slots = [[cross, c] for c in coords]
            place_yaw = 0.0        # jaws along X: open fingers reach across the row, never along it
        else:
            slots = [[c, cross] for c in coords]
            place_yaw = 90.0
        return slots, pitch, place_yaw

    @classmethod
    def layout_avoiding(cls, n: int, longest_mm: float, ws, p: dict, avoid: List[WorldObject],
                        margin_mm: float) -> Tuple[List[List[float]], float, float]:
        """``layout`` at the configured inset, or — when a block in ``avoid``
        (one that cannot or may not be movable) lies on the row — the nearest
        deeper inset whose slots all clear those blocks. Falls back to the
        configured inset if no inset clears them."""
        base = cls.layout(n, longest_mm, ws, p)
        if not avoid:
            return base
        dims = (longest_mm, longest_mm)
        opening = min(P.GRIPPER_STROKE_MM, longest_mm + 12.0)
        span = (ws.x_max - ws.x_min) if p["row_axis"] == "y" else (ws.y_max - ws.y_min)
        inset0 = float(p["row_inset_mm"])
        extra = 0.0
        while inset0 + extra <= span - 40.0:
            pp = dict(p); pp["row_inset_mm"] = inset0 + extra
            slots, pitch, yaw = cls.layout(n, longest_mm, ws, pp)
            if all(P.site_clear(s, dims, avoid, opening, margin_mm, jaw_yaw_deg=yaw) for s in slots):
                return slots, pitch, yaw
            extra += 20.0
        return base

    # ── run ─────────────────────────────────────────────────────────
    def run(self, robot: Robot, world: World, p: dict, trace: Trace) -> PolicyResult:
        t0 = time.monotonic()
        n = int(p["look_frames"])
        robot.set_tool(1)
        axis = 1 if p["row_axis"] == "y" else 0
        margin = float(p["place_clearance_mm"])

        def blocks(frames: int = 1) -> List[WorldObject]:
            """Union of detections over several frames (by id), positions,
            sizes and yaw averaged per block."""
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
                # circular mean mod 90° (a cube's yaw is only defined mod 90)
                sx = sum(math.cos(math.radians(4 * x.yaw_deg)) for x in samples)
                sy = sum(math.sin(math.radians(4 * x.yaw_deg)) for x in samples)
                b.yaw_deg = math.degrees(math.atan2(sy, sx)) / 4.0
                out.append(b)
            return out

        objs = blocks(3)
        if not objs:
            return PolicyResult(ok=False, reason="no blocks visible", attempts=0)
        ws = getattr(world, "workspace", None) or robot.guards.workspace

        # Blocks the gripper can never hold (too flat for the clearance floor,
        # too wide for the stroke) are left out of the row, with the reason.
        hopeless: Dict[str, str] = {}
        for o in objs:
            try:
                P.grasp_height(o, world, robot, float(p["grasp_height_mm"]))
                P.choose_grasp(o, [], float(p["open_margin_mm"]))
            except P.GraspInfeasible as e:
                hopeless[o.id] = str(e)
                trace.log(f"leaving {o.color} '{o.id}' out of the row: {e}")
        movable = [o for o in objs if o.id not in hopeless]
        # A block whose top barely clears the floor may still turn out to be
        # ungrippable; rank those last so a failure leaves the row contiguous.
        surface = world.table_z - robot.tool_length_mm
        marginal = {o.id: (world.min_clearance_mm + 5.0) - (o.top_z - surface)      # id → how doubtful (mm short)
                    for o in movable if o.top_z - surface <= world.min_clearance_mm + 5.0}
        if not movable:
            return PolicyResult(ok=False, reason="no block can be grasped: " + "; ".join(hopeless.values()), attempts=0)

        longest = max(max(o.size[:2]) for o in movable)
        try:
            slots, pitch, place_yaw = self.layout_avoiding(len(movable), longest, ws, p,
                                                           [o for o in objs if o.id in marginal or o.id in hopeless], margin)
        except ValueError as e:
            return PolicyResult(ok=False, reason=f"row layout failed: {e}", attempts=0)
        slot_tol = pitch * 0.25
        align_tol = float(p["align_tol_deg"])
        relayouts = 0

        def describe_row() -> str:
            return (f"row along {p['row_axis'].upper()} at {'x' if axis == 1 else 'y'}={slots[0][1 - axis]:.0f}, "
                    f"{len(slots)} slots, pitch {pitch:.0f} mm: {[round(s[axis]) for s in slots]}")
        trace.log(describe_row())

        def seated(o: WorldObject, k: int) -> bool:
            return math.dist(o.xy, slots[k]) <= slot_tol and _yaw_err(o.yaw_deg, place_yaw) <= align_tol

        def dims_at_release(o: WorldObject, jaw_yaw: float) -> Tuple[float, float]:
            """Axis-aligned footprint once released square to the row: the
            grasped width lies along the jaw axis, the other side along the row."""
            w, other = o.width_across(jaw_yaw), o.width_across(jaw_yaw + 90.0)
            return (w, other) if place_yaw == 0.0 else (other, w)

        # ── assign blocks to slots ──────────────────────────────────
        assign = self._assign(movable, slots, axis, slot_tol, place_yaw, align_tol, str(p["order"]), marginal)
        for o in movable:
            trace.log(f"slot {assign[o.id]}: {o.color} '{o.id}' ({o.size[0]:.0f}×{o.size[1]:.0f})")

        moves = misses = 0
        fails: Dict[str, int] = {}
        confirmed: set = set()       # blocks seen seated on their slot after a place
        iterations = 0
        while moves < int(p["max_moves"]):
            iterations += 1
            if iterations > int(p["max_moves"]) * 4:
                trace.log("iteration budget exhausted — stopping"); break
            objs = blocks(n)
            by_id = {o.id: o for o in objs}
            pending = [(k, bid) for bid, k in sorted(assign.items(), key=lambda kv: kv[1])
                       if bid not in hopeless and (bid not in by_id or not seated(by_id[bid], k))]
            if not pending:
                objs = blocks(3); by_id = {o.id: o for o in objs}
                pending = [(k, bid) for bid, k in sorted(assign.items(), key=lambda kv: kv[1])
                           if bid not in hopeless and (bid not in by_id or not seated(by_id[bid], k))]
                if not pending:
                    break
            k, bid = pending[0]
            obj = by_id.get(bid)
            if bid in confirmed and obj is not None:
                # A block we already saw seated is cheap to re-check and
                # expensive to re-move: take a longer look before unseating it.
                again = next((o for o in blocks(2 * n + 1) if o.id == bid), None)
                if again is not None and seated(again, k):
                    continue
                confirmed.discard(bid)
                trace.log(f"'{bid}' no longer seated on slot {k} "
                          f"({math.dist((again or obj).xy, slots[k]):.0f} mm off, "
                          f"{_yaw_err((again or obj).yaw_deg, place_yaw):.0f}° off square)")
            if obj is None:
                fails[bid] = fails.get(bid, 0) + 1
                if fails[bid] >= 3:
                    hopeless[bid] = "lost from view"
                continue
            slot = slots[k]
            jaw_guess = obj.short_axis_yaw()
            dims = dims_at_release(obj, jaw_guess)
            release_open = min(P.GRIPPER_STROKE_MM, obj.width_across(jaw_guess) + 12.0)

            # Something else sitting on this slot? Move it out of the way first.
            occupants = [o for o in objs if o.id != bid
                         and not P.site_clear(slot, dims, [o], release_open, margin, jaw_yaw_deg=place_yaw)]
            if occupants:
                occ = min(occupants, key=lambda o: math.dist(o.xy, slot))
                if occ.id in hopeless:
                    # An immovable block lies on the row: shift the whole row
                    # away from it (once or twice) rather than leave a gap.
                    if relayouts < 2:
                        try:
                            new_slots, pitch, place_yaw = self.layout_avoiding(
                                len(slots), longest, ws, p, [o for o in objs if o.id in hopeless or o.id in marginal], margin)
                        except ValueError:
                            new_slots = slots
                        if new_slots != slots:
                            relayouts += 1
                            slots = new_slots
                            assign = self._assign(movable, slots, axis, slot_tol, place_yaw, align_tol, str(p["order"]), marginal)
                            trace.log(f"{occ.color} '{occ.id}' cannot be moved off the row; shifting the row — " + describe_row())
                            continue
                    hopeless[bid] = f"slot {k} is blocked by {occ.color} '{occ.id}' which cannot be moved"
                    trace.log(f"giving up on {obj.color} '{obj.id}': {hopeless[bid]}")
                    continue
                dest = None
                j = assign.get(occ.id)
                if j is not None and j != k:
                    odims = dims_at_release(occ, occ.short_axis_yaw())
                    if P.site_clear(slots[j], odims, objs, min(P.GRIPPER_STROKE_MM, occ.width_across(occ.short_axis_yaw()) + 12.0),
                                    margin, exclude=[occ.id], jaw_yaw_deg=place_yaw):
                        dest = slots[j]
                if dest is None:
                    dest = self._parking_site(ws, objs, occ, slots, pitch, margin, place_yaw)
                if dest is None:
                    fails[bid] = fails.get(bid, 0) + 1
                    if fails[bid] >= 2:
                        hopeless[bid] = f"slot {k} is blocked by {occ.color} '{occ.id}' and nowhere to park it"
                    continue
                trace.log(f"move {moves + 1}: {occ.color} '{occ.id}' is on slot {k}; moving it to ({dest[0]:.0f}, {dest[1]:.0f})")
                ok, why = self._pick_and_place(robot, world, trace, p, occ, dest, objs, place_yaw)
                if ok:
                    moves += 1
                else:
                    self._note_failure(occ, why, fails, hopeless, trace)
                    if why.startswith("miss"):
                        misses += 1
                continue

            trace.log(f"move {moves + 1}: {obj.color} '{obj.id}' → slot {k} ({slot[0]:.0f}, {slot[1]:.0f})")
            ok, why = self._pick_and_place(robot, world, trace, p, obj, slot, objs, place_yaw)
            if not ok:
                self._note_failure(obj, why, fails, hopeless, trace)
                if why.startswith("miss"):
                    misses += 1
                continue
            moves += 1
            placed = next((o for o in blocks(2 * n + 1) if o.id == obj.id), None)
            if placed is None:
                continue            # occluded by the arm — the next look decides
            if seated(placed, k):
                confirmed.add(obj.id)
            else:
                fails[obj.id] = fails.get(obj.id, 0) + 1
                trace.log(f"'{obj.id}' is {math.dist(placed.xy, slot):.0f} mm from slot {k}, "
                          f"{_yaw_err(placed.yaw_deg, place_yaw):.0f}° off square; attempt {fails[obj.id]}")
                if fails[obj.id] >= 3:
                    hopeless[obj.id] = "would not seat on its slot"

        objs = blocks(3)
        by_id = {o.id: o for o in objs}
        left = [bid for bid, k in sorted(assign.items(), key=lambda kv: kv[1])
                if bid not in by_id or not seated(by_id[bid], k)]
        left += [bid for bid in hopeless if bid not in assign]
        obs = {"moves": moves, "misses": misses, "in_row": len(assign) - len([b for b in left if b in assign]),
               "slots": len(slots), "pitch_mm": round(pitch), "order": str(p["order"]),
               "row": [bid for bid, _ in sorted(assign.items(), key=lambda kv: kv[1])], "given_up": hopeless}
        try:
            P.lift(robot, world, trace, float(p["carry_mm"]))
        except Exception:
            pass
        if left:
            def name(bid):
                o = by_id.get(bid)
                s = f"{o.color} '{bid}'" if o else f"'{bid}'"
                return s + (f" ({hopeless[bid]})" if bid in hopeless else "")
            return PolicyResult(ok=False, reason=f"{len(left)} block(s) not in the row after {moves} moves: "
                                + "; ".join(name(b) for b in left),
                                attempts=moves, observations=obs, elapsed_s=time.monotonic() - t0)
        return PolicyResult(ok=True, attempts=moves, observations=obs, elapsed_s=time.monotonic() - t0)

    # ── helpers ─────────────────────────────────────────────────────
    @staticmethod
    def _assign(movable: List[WorldObject], slots: List[List[float]], axis: int, slot_tol: float,
                place_yaw: float, align_tol: float, order: str, marginal: Optional[Dict[str, float]] = None) -> Dict[str, int]:
        """Block id → slot index. ``size``: largest footprint first; ``color``:
        grouped by colour, largest first within a colour; ``any``: blocks already
        seated keep their slot and the rest fill the free slots in the order
        they lie along the row (fewest crossings, fewest moves). Ties are
        broken by position along the row so equal blocks never swap places.
        ``marginal`` blocks (doubtful grips, id → how doubtful) always go to the
        end of the row, the most doubtful last."""
        marginal = marginal or {}
        along = lambda o: o.position[axis]
        doubt = lambda o: 1.0 + marginal[o.id] if o.id in marginal else 0.0
        if order == "size":
            ranked = sorted(movable, key=lambda o: (doubt(o), -round(_area(o) / 100.0), along(o)))
            return {o.id: k for k, o in enumerate(ranked)}
        if order == "color":
            ranked = sorted(movable, key=lambda o: (doubt(o), o.color, -round(_area(o) / 100.0), along(o)))
            return {o.id: k for k, o in enumerate(ranked)}
        assign: Dict[str, int] = {}
        taken: set = set()
        for o in sorted(movable, key=along):
            if o.id in marginal:
                continue
            for k, s in enumerate(slots):
                if k not in taken and math.dist(o.xy, s) <= slot_tol and _yaw_err(o.yaw_deg, place_yaw) <= align_tol:
                    assign[o.id] = k; taken.add(k); break
        free = [k for k in range(len(slots)) if k not in taken]
        for o in sorted((o for o in movable if o.id not in assign), key=lambda o: (doubt(o), along(o))):
            assign[o.id] = free.pop(0)
        return assign

    @staticmethod
    def _parking_site(ws, objs: List[WorldObject], block: WorldObject, slots: List[List[float]],
                      pitch: float, margin: float, place_yaw: float) -> Optional[List[float]]:
        """A clear spot off the row (at least a pitch away from the row line)
        where ``block`` can be released without touching anything."""
        side = max(block.size[:2])
        dims = (side, side)
        opening = min(P.GRIPPER_STROKE_MM, side + 12.0)
        row_x = all(abs(s[1] - slots[0][1]) < 1e-6 for s in slots)   # row runs along X (constant y)
        best = None
        for dx in range(80, 400, 60):
            for dy in range(80, 700, 60):
                for sx in (-1, 1):
                    for sy in (-1, 1):
                        x = ws.x_max - dx if sx < 0 else ws.x_min + dx
                        y = ws.y_max - dy if sy < 0 else ws.y_min + dy
                        if not ws.contains(x, y, slack_mm=-40.0):
                            continue
                        off_row = abs(y - slots[0][1]) if row_x else abs(x - slots[0][0])
                        if off_row < pitch:
                            continue
                        if not P.site_clear([x, y], dims, objs, opening, margin + 10.0, exclude=[block.id],
                                            jaw_yaw_deg=place_yaw):
                            continue
                        d = math.dist([x, y], block.xy)
                        if best is None or d < best[0]:
                            best = (d, [x, y])
        return best[1] if best else None

    @staticmethod
    def _note_failure(obj: WorldObject, why: str, fails: Dict[str, int], hopeless: Dict[str, str], trace: Trace) -> None:
        if "clearance floor" in why or "stroke" in why:
            hopeless[obj.id] = why
            trace.log(f"giving up on {obj.color} '{obj.id}': {why}")
            return
        fails[obj.id] = fails.get(obj.id, 0) + 1
        trace.log(f"'{obj.id}' not moved ({why}); attempt {fails[obj.id]}")
        if fails[obj.id] >= 3:
            hopeless[obj.id] = why

    def _pick_and_place(self, robot, world, trace, p, obj: WorldObject, target_xy: Sequence[float],
                        objs: List[WorldObject], place_yaw: float):
        """Pick ``obj`` and set it down square to the row at ``target_xy``.
        Returns (ok, reason)."""
        n = int(p["look_frames"])
        try:
            obj = P.look_avg(world, trace, n=n, id=obj.id)
        except NotFound:
            # The multi-frame survey that chose this block did see it; a
            # focused look that misses every frame is dropout, not absence.
            trace.log(f"'{obj.id}' not seen in a focused look; using the survey estimate")
        try:
            grasp_h = P.grasp_height(obj, world, robot, float(p["grasp_height_mm"]))
            jaw_yaw, opening, clear = P.choose_grasp(obj, objs, float(p["open_margin_mm"]))
        except P.GraspInfeasible as e:
            return False, str(e)
        width = obj.width_across(jaw_yaw)
        release_open = min(P.GRIPPER_STROKE_MM, width + 12.0)
        trace.log(f"grasp yaw {jaw_yaw:.0f}° opening {opening:.0f} mm (clearance {clear:.0f} mm)")
        P.open_gripper(robot, trace, opening)
        P.hover_above(robot, world, trace, obj.xy, float(p["approach_mm"]), jaw_yaw)
        P.descend_to(robot, world, trace, grasp_h)
        try:
            P.grasp(robot, trace, width_hint_mm=width, force=int(p["force"]))
        except GraspMissed:
            P.lift(robot, world, trace, float(p["approach_mm"]))
            return False, "miss: " + P.explain_miss(obj, world, robot, grasp_h)
        P.lift(robot, world, trace, float(p["carry_mm"]))
        off = P.in_hand_offset(robot, world, trace, n, jaw_yaw, place_yaw, id=obj.id)
        try:
            P.place_at(robot, world, trace, [target_xy[0] - off[0], target_xy[1] - off[1]],
                       grasp_h + float(p["drop_mm"]), float(p["carry_mm"]), place_yaw, open_width_mm=release_open)
        except Unreachable as e:
            trace.log(f"target unreachable ({e}); releasing here")
            P.descend_to(robot, world, trace, grasp_h + float(p["drop_mm"]))
            P.open_gripper(robot, trace, release_open)
            P.lift(robot, world, trace, float(p["carry_mm"]))
            return False, "target unreachable"
        return True, ""
