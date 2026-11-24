import math
import pytest

from dobot_ros.policies.types import WorldObject
from dobot_ros.sim.noise import NoiseModel
from dobot_ros.sim.world import SimWorld


def test_scene_loads_objects_on_surface():
    w = SimWorld.from_scene("two_blocks", noise=NoiseModel(enabled=False))
    objs = w.objects()
    assert {o.color for o in objs} == {"red", "blue"}
    red = w.find(color="red")
    assert red.bottom_z == pytest.approx(w.surface_z)
    assert w.surface_z == pytest.approx(w.table_z - 203.0)


def test_find_is_case_insensitive_and_none_on_miss():
    w = SimWorld.from_scene("two_blocks", noise=NoiseModel(enabled=False))
    assert w.find(color="RED").id == "red_block"
    assert w.find(color="green") is None


def test_noise_is_seeded_and_bounded():
    a = SimWorld.from_scene("two_blocks", seed=7).find(color="red")
    b = SimWorld.from_scene("two_blocks", seed=7).find(color="red")
    assert a.position == b.position and a.yaw_deg == b.yaw_deg
    truth = SimWorld.from_scene("two_blocks", noise=NoiseModel(enabled=False)).find(color="red")
    assert math.dist(a.xy, truth.xy) < 25.0
    assert a.position != truth.position


def test_miss_prob_one_hides_everything():
    w = SimWorld.from_scene("two_blocks", noise=NoiseModel(miss_prob=1.0))
    assert w.objects() == []
    assert len(w.truth()) == 2


def test_randomize_places_non_overlapping_reachable_objects():
    w = SimWorld.from_scene("clutter", seed=3, random_positions=True, noise=NoiseModel(enabled=False))
    objs = w.truth()
    assert len(objs) == 6
    for i, a in enumerate(objs):
        assert w.workspace.contains(*a.xy)
        for b in objs[i + 1:]:
            assert math.dist(a.xy, b.xy) > 30.0


def test_support_z_stacking():
    w = SimWorld(noise=NoiseModel(enabled=False))
    w.add(WorldObject("base", "block", "red", [-450, -300, w.surface_z + 20], [40, 40, 40]))
    assert w.support_z_at(-450, -300) == pytest.approx(w.surface_z + 40)
    assert w.support_z_at(-600, -300) == pytest.approx(w.surface_z)


def test_width_across_and_short_axis():
    o = WorldObject("o", "block", "red", [0, 0, 0], [60, 30, 30], yaw_deg=0)
    assert o.width_across(0) == pytest.approx(60)
    assert o.width_across(90) == pytest.approx(30)
    assert o.short_axis_yaw() == pytest.approx(90)
