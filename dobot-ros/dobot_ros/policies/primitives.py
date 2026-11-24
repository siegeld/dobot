"""Mid-level motion/perception primitives every policy composes.

All poses are built for a *vertical* wrist (RX=180, RY=0) in whatever tool
frame is active, using the project's ``pose_z_from_table_z`` convention so
heights are always "mm above the table at the fingertip".
"""

from __future__ import annotations

from typing import Any, List, Optional, Sequence

from dobot_ros.policies.types import (
    GraspMissed, NotFound, Robot, Trace, World, WorldObject,
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
