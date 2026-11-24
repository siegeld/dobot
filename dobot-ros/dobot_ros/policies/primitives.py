"""Mid-level motion/perception primitives every policy composes.

All poses are built for a *vertical* wrist (RX=180, RY=0) in whatever tool
frame is active, using the project's ``pose_z_from_table_z`` convention so
heights are always "mm above the table at the fingertip".
"""

from __future__ import annotations

from typing import Any, List, Optional, Sequence

import copy
import math

from dobot_ros.policies.types import (
    GraspMissed, NotFound, PolicyError, Robot, Trace, World, WorldObject,
)

VERTICAL_RX = 180.0
VERTICAL_RY = 0.0

# AG-105: 0 = closed, 1000 = fully open = 105 mm between the fingers.
GRIPPER_STROKE_MM = 105.0


def width_to_pos(width_mm: float) -> int:
    return int(max(0, min(1000, round(width_mm / GRIPPER_STROKE_MM * 1000.0))))


def pos_to_width(pos: int) -> float:
    return max(0, min(1000, int(pos))) / 1000.0 * GRIPPER_STROKE_MM


def pose_z_for_height(robot: Robot, world: World, height_above_table_mm: float) -> float:
    """Z to command so the *fingertip* sits ``height`` mm above the table."""
    if robot.tool_index() == 0:
        return world.table_z + height_above_table_mm
    return world.table_z - robot.tool_length_mm + height_above_table_mm


def _wrap(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def vertical_pose(robot: Robot, world: World, xy: Sequence[float],
                  height_mm: float, yaw_deg: Optional[float] = None) -> List[float]:
    rz = robot.pose()[5] if yaw_deg is None else _wrap(yaw_deg)
    return [float(xy[0]), float(xy[1]), pose_z_for_height(robot, world, height_mm),
            VERTICAL_RX, VERTICAL_RY, rz]


# ── Perception ──────────────────────────────────────────────────────

def look(world: World, trace: Trace, retries: int = 3, **query: Any) -> WorldObject:
    """Query perception; raise NotFound (with what *was* seen) on a miss.

    Detectors flicker — a target can drop out of one frame and be back in
    the next — so a miss is re-queried ``retries`` times before giving up.
    """
    trace.call("look", retries=retries, **query)
    seen: List[WorldObject] = []
    for attempt in range(max(1, int(retries))):
        obj = world.find(**query)
        if obj is not None:
            trace.ret("look", obj)
            return obj
        seen = world.objects()
        trace.log(f"look miss {attempt + 1}/{retries} for {query}")
    trace.ret("look", None)
    desc = ", ".join(f"{o.color} {o.label}" for o in seen) or "nothing"
    raise NotFound(f"no match for {query}; visible: {desc}", seen)


def look_avg(world: World, trace: Trace, n: int = 3, retries: int = 3, **query: Any) -> WorldObject:
    """``look`` n times and average position / size / yaw. Perception noise
    is independent per frame, so this shrinks it by √n at the cost of n
    frames. Yaw is averaged as a deviation from the first sample so the
    ±90° symmetry of a block never produces a bogus mean."""
    samples = [look(world, trace, retries=retries, **query) for _ in range(max(1, int(n)))]
    base = copy.deepcopy(samples[0])
    k = len(samples)
    base.position = [sum(s.position[i] for s in samples) / k for i in range(3)]
    base.size = [sum(s.size[i] for s in samples) / k for i in range(3)]
    dev = [((s.yaw_deg - samples[0].yaw_deg + 90.0) % 180.0) - 90.0 for s in samples]
    base.yaw_deg = samples[0].yaw_deg + sum(dev) / k
    base.confidence = sum(s.confidence for s in samples) / k
    trace.log(f"look_avg {query}: {k} frames → xy=({base.position[0]:.1f}, {base.position[1]:.1f}) "
              f"yaw={base.yaw_deg:.1f} h={base.size[2]:.0f}")
    return base


class GraspInfeasible(PolicyError):
    """Object cannot be grasped: too short for the clearance floor, too wide
    for the stroke, or no finger approach clears the neighbours."""


def _rect_clearance(px: float, py: float, o: WorldObject) -> float:
    """Signed distance from point to the object's footprint (negative = inside)."""
    d = math.radians(-o.yaw_deg)
    dx, dy = px - o.position[0], py - o.position[1]
    lx = dx * math.cos(d) - dy * math.sin(d)
    ly = dx * math.sin(d) + dy * math.cos(d)
    ex, ey = abs(lx) - o.size[0] / 2.0, abs(ly) - o.size[1] / 2.0
    if ex > 0 and ey > 0:
        return math.hypot(ex, ey)
    return max(ex, ey)


FINGER_W_MM = 10.0


def choose_grasp(obj: WorldObject, others: List[WorldObject], open_margin_mm: float,
                 min_margin_mm: float = 6.0, finger_w_mm: float = FINGER_W_MM):
    """Pick the jaw axis and opening for ``obj`` given its neighbours.

    Candidates are the two footprint axes. For each, the required opening is
    the object's width across that axis plus a margin; the finger pads then
    sit at ±(opening/2 + finger/2). The candidate whose pads clear every
    neighbour by the most wins; the margin shrinks toward ``min_margin_mm``
    if that is what it takes to fit. Returns (jaw_yaw_deg, open_width_mm,
    clearance_mm). Raises GraspInfeasible when nothing clears.
    """
    best = None
    for yaw in (obj.short_axis_yaw(), obj.short_axis_yaw() + 90.0):
        width = obj.width_across(yaw)
        if width > GRIPPER_STROKE_MM - 4.0:
            continue
        for margin in sorted({open_margin_mm, max(min_margin_mm, open_margin_mm / 2.0), min_margin_mm}, reverse=True):
            opening = min(GRIPPER_STROKE_MM, width + margin)
            half = opening / 2.0 + finger_w_mm / 2.0
            ux, uy = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
            clear = float("inf")
            for sgn in (-1.0, 1.0):
                fx, fy = obj.position[0] + sgn * half * ux, obj.position[1] + sgn * half * uy
                for o in others:
                    if o.id == obj.id or o.support == "gripper":
                        continue
                    clear = min(clear, _rect_clearance(fx, fy, o) - finger_w_mm / 2.0)
            cand = (clear, margin, yaw, opening)
            if best is None or cand[:2] > best[:2]:
                best = cand
            if clear > 0:
                break
    if best is None:
        raise GraspInfeasible(f"{obj.color} {obj.label} is {min(obj.size[:2]):.0f} mm across its "
                              f"narrow side — wider than the {GRIPPER_STROKE_MM:.0f} mm stroke")
    clear, margin, yaw, opening = best
    if clear < 0:
        raise GraspInfeasible(f"no finger approach to {obj.color} {obj.label} clears its neighbours "
                              f"(best {clear:.0f} mm)")
    return yaw, opening, clear


def grasp_height(obj: WorldObject, world: World, robot: Robot, wanted_mm: float,
                 top_margin_mm: float = 5.0, height_tolerance_mm: float = 15.0) -> float:
    """Fingertip height above the table for the close.

    Perceived object height is the least trustworthy number the camera gives
    (the real adapter does not measure it at all), so it is used to *lower*
    the grasp toward the floor, never to refuse a grasp outright — unless the
    top is below the clearance floor by more than ``height_tolerance_mm``,
    which no plausible error explains. A marginal object is attempted at the
    floor; a miss there is reported with ``explain_miss``.
    """
    surface = world.table_z - robot.tool_length_mm
    top = obj.top_z - surface
    floor = world.min_clearance_mm
    if top < floor - height_tolerance_mm:
        raise GraspInfeasible(f"{obj.color} {obj.label} top is only {top:.0f} mm above the table; the "
                              f"{floor:.0f} mm clearance floor leaves nothing to grip")
    return min(max(wanted_mm, floor), max(top - top_margin_mm, floor))


def explain_miss(obj: WorldObject, world: World, robot: Robot, grasp_h: float) -> str:
    """Why a close-on-air probably happened, for the failure reason."""
    surface = world.table_z - robot.tool_length_mm
    top = obj.top_z - surface
    if top - 5.0 <= grasp_h:
        return (f"fingertips at {grasp_h:.0f} mm were at or above the {obj.color} {obj.label}'s top "
                f"(≈{top:.0f} mm) — the {world.min_clearance_mm:.0f} mm clearance floor is too high for it")
    return "jaws closed on air — position or yaw error"


# ── Motion ──────────────────────────────────────────────────────────

def go_home(robot: Robot, world: World, trace: Trace, xy: Sequence[float],
            height_mm: float = 250.0) -> None:
    """Vertical wrist at a safe height above a workspace point."""
    trace.call("go_home", xy=list(xy), height_mm=height_mm)
    robot.move_l(vertical_pose(robot, world, xy, height_mm))
    trace.ret("go_home")


def hover_above(robot: Robot, world: World, trace: Trace, xy: Sequence[float],
                height_mm: float, yaw_deg: Optional[float] = None) -> None:
    trace.call("hover_above", xy=list(xy), height_mm=height_mm, yaw_deg=yaw_deg)
    robot.move_l(vertical_pose(robot, world, xy, height_mm, yaw_deg))
    trace.ret("hover_above")


def descend_to(robot: Robot, world: World, trace: Trace, height_mm: float) -> None:
    """Straight down (same XY and yaw) to a fingertip height above the table."""
    trace.call("descend_to", height_mm=height_mm)
    cur = robot.pose()
    robot.move_l(vertical_pose(robot, world, cur[:2], height_mm, cur[5]))
    trace.ret("descend_to")


def lift(robot: Robot, world: World, trace: Trace, height_mm: float) -> None:
    trace.call("lift", height_mm=height_mm)
    cur = robot.pose()
    robot.move_l(vertical_pose(robot, world, cur[:2], height_mm, cur[5]))
    trace.ret("lift")


def open_gripper(robot: Robot, trace: Trace, width_mm: float = GRIPPER_STROKE_MM) -> int:
    trace.call("open_gripper", width_mm=width_mm)
    pos = robot.gripper(width_to_pos(width_mm))
    trace.ret("open_gripper", pos)
    return pos


def grasp(robot: Robot, trace: Trace, width_hint_mm: Optional[float] = None,
          force: int = 20, closed_threshold_mm: float = 3.0) -> float:
    """Close the jaws. Returns the final jaw width in mm.

    Raises GraspMissed if the jaws closed (nearly) fully — the one grasp
    signal the AG-105 gives us without tactile sensing.
    """
    trace.call("grasp", width_hint_mm=width_hint_mm, force=force)
    pos = robot.gripper(0, force=force)
    width = pos_to_width(pos)
    trace.ret("grasp", {"jaw_pos": pos, "jaw_width_mm": round(width, 1)})
    if width <= closed_threshold_mm:
        raise GraspMissed(f"jaws closed to {width:.1f} mm — nothing grasped")
    return width


def place_at(robot: Robot, world: World, trace: Trace, xy: Sequence[float],
             release_height_mm: float, approach_height_mm: float,
             yaw_deg: Optional[float] = None) -> None:
    trace.call("place_at", xy=list(xy), release_height_mm=release_height_mm,
               approach_height_mm=approach_height_mm, yaw_deg=yaw_deg)
    hover_above(robot, world, trace, xy, approach_height_mm, yaw_deg)
    descend_to(robot, world, trace, release_height_mm)
    open_gripper(robot, trace)
    lift(robot, world, trace, approach_height_mm)
    trace.ret("place_at")
