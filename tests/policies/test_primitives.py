import pytest

from dobot_ros.policies import primitives as P
from dobot_ros.policies.types import GraspMissed, NotFound, Trace
from dobot_ros.sim.noise import NoiseModel
from dobot_ros.sim.robot import SimRobot
from dobot_ros.sim.world import SimWorld


@pytest.fixture
def rig():
    w = SimWorld.from_scene("two_blocks", noise=NoiseModel(enabled=False))
    r = SimRobot(w)
    return w, r, r.trace


def test_width_pos_conversion():
    assert P.width_to_pos(105) == 1000 and P.width_to_pos(0) == 0
    assert P.pos_to_width(500) == pytest.approx(52.5)


def test_pose_z_for_height_respects_tool_frame(rig):
    w, r, t = rig
    r.set_tool(0)
    assert P.pose_z_for_height(r, w, 10) == pytest.approx(w.table_z + 10)
    r.set_tool(1)
    assert P.pose_z_for_height(r, w, 10) == pytest.approx(w.table_z - 203 + 10)


def test_look_raises_with_seen_list(rig):
    w, r, t = rig
    with pytest.raises(NotFound) as ei:
        P.look(w, t, color="green")
    assert {o.color for o in ei.value.seen} == {"red", "blue"}
    assert "visible: " in str(ei.value)


def test_grasp_missed_on_air(rig):
    w, r, t = rig
    r.set_tool(1)
    P.hover_above(r, w, t, (-550, -100), 60)
    with pytest.raises(GraspMissed):
        P.grasp(r, t)


def test_place_at_sequence(rig):
    w, r, t = rig
    r.set_tool(1)
    red = w.get("red_block")
    P.open_gripper(r, t, 70)
    P.hover_above(r, w, t, red.xy, 80, red.short_axis_yaw())
    P.descend_to(r, w, t, 25)
    P.grasp(r, t)
    P.lift(r, w, t, 120)
    P.place_at(r, w, t, (-560, -400), 25, 120)
    assert red.support == "table" and red.xy == pytest.approx([-560, -400], abs=1.0)
    assert r.pose()[2] == pytest.approx(w.surface_z + 120, abs=0.2)
