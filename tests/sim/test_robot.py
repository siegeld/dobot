import math
import threading
import pytest

from dobot_ros.policies.types import (
    Collision, FloorViolation, PolicyAbort, WorkspaceViolation, WorldObject,
)
from dobot_ros.sim.noise import NoiseModel
from dobot_ros.sim.robot import SimRobot
from dobot_ros.sim.world import SimWorld


def make(scene="two_blocks"):
    w = SimWorld.from_scene(scene, noise=NoiseModel(enabled=False))
    r = SimRobot(w)
    r.set_tool(1)
    return w, r


def hover(w, r, xy, h, yaw=0.0):
    r.move_l([xy[0], xy[1], w.surface_z + h, 180, 0, yaw])


def test_start_pose_is_vertical_above_table():
    w, r = make()
    p = r.pose()
    assert abs(p[3] - 180) < 0.1 and abs(p[4]) < 0.1
    assert p[2] > w.surface_z + 200


def test_floor_guard_blocks_before_moving():
    w, r = make()
    before = r.pose()
    with pytest.raises(FloorViolation):
        r.move_l([-480, -250, w.surface_z + w.min_clearance_mm - 1, 180, 0, 0])
    assert r.pose() == pytest.approx(before)


def test_workspace_guard():
    w, r = make()
    with pytest.raises(WorkspaceViolation):
        r.move_l([0, 0, w.surface_z + 200, 180, 0, 0])


def test_move_l_reaches_pose_and_records_samples():
    w, r = make()
    n0 = len(r.trace.events)
    hover(w, r, (-480, -250), 80, 15)
    assert r.pose() == pytest.approx([-480, -250, w.surface_z + 80, 180, 0, 15], abs=0.2)
    kinds = [e.kind for e in r.trace.events[n0:]]
    assert kinds.count("sample") >= 3 and "call" in kinds and "return" in kinds


def test_grasp_attach_lift_release():
    w, r = make()
    red = w.get("red_block")
    r.gripper(700)                                  # 73 mm open
    hover(w, r, red.xy, 80, red.yaw_deg + 90)
    hover(w, r, red.xy, 25, red.yaw_deg + 90)
    pos = r.gripper(0)
    assert r.attached is red
    assert abs(pos / 1000 * 105 - 40.0) < 0.5        # stopped by the 40 mm block
    hover(w, r, red.xy, 150, red.yaw_deg + 90)
    assert red.support == "gripper"
    assert red.bottom_z > w.surface_z + 100           # it came along
    # carry sideways, release onto the table
    hover(w, r, (-520, -350), 150, red.yaw_deg + 90)
    hover(w, r, (-520, -350), 30, red.yaw_deg + 90)
    r.gripper(700)
    assert r.attached is None
    assert red.support == "table"
    assert red.bottom_z == pytest.approx(w.surface_z, abs=1e-6)
    assert red.xy == pytest.approx([-520, -350], abs=1.0)


def test_close_on_air_reads_zero():
    w, r = make()
    hover(w, r, (-550, -100), 60)
    assert r.gripper(0) == 0
    assert r.attached is None


def test_jaws_too_narrow_do_not_grasp():
    w, r = make()
    red = w.get("red_block")
    r.gripper(300)                                  # 31.5 mm — narrower than the block
    with pytest.raises(Collision):
        hover(w, r, red.xy, 80, red.yaw_deg)
        hover(w, r, red.xy, 25, red.yaw_deg)       # fingers land on top of it


def test_release_stacks_on_another_block():
    w, r = make()
    red, blue = w.get("red_block"), w.get("blue_block")
    r.gripper(700)
    hover(w, r, red.xy, 80, red.yaw_deg + 90); hover(w, r, red.xy, 25, red.yaw_deg + 90)
    r.gripper(0)
    hover(w, r, red.xy, 150, red.yaw_deg + 90)
    hover(w, r, blue.xy, 150, red.yaw_deg + 90)
    hover(w, r, blue.xy, 70, red.yaw_deg + 90)      # bottom of red at 50 → above blue's top (40)
    r.gripper(700)
    assert red.support == "blue_block"
    assert red.bottom_z == pytest.approx(blue.top_z)


def test_carried_object_hitting_another_is_a_collision():
    w, r = make()
    red, blue = w.get("red_block"), w.get("blue_block")
    r.gripper(700)
    hover(w, r, red.xy, 80, red.yaw_deg + 90); hover(w, r, red.xy, 25, red.yaw_deg + 90)
    r.gripper(0)
    hover(w, r, red.xy, 25 + 10, red.yaw_deg + 90)  # barely lifted: bottom at ~5 mm
    with pytest.raises(Collision):
        hover(w, r, blue.xy, 35, red.yaw_deg + 90)


def test_stop_event_aborts_motion():
    w, r = make()
    r.stop()
    with pytest.raises(PolicyAbort):
        hover(w, r, (-480, -250), 80)


def test_tool_frame_switch_changes_reported_z_by_tool_length():
    w, r = make()
    z1 = r.pose()[2]
    r.set_tool(0)
    z0 = r.pose()[2]
    assert z0 - z1 == pytest.approx(203.0, abs=1e-6)


def test_virtual_clock_advances_with_distance():
    w, r = make()
    t0 = r.clock.t
    hover(w, r, (-480, -250), 80)
    assert r.clock.t > t0
