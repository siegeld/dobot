"""Grasp planning helpers + simulator finger model."""
import pytest

from dobot_ros.policies import primitives as P
from dobot_ros.policies.types import Collision, Trace, WorldObject
from dobot_ros.sim.noise import NoiseModel
from dobot_ros.sim.robot import SimRobot
from dobot_ros.sim.world import SimWorld


def blk(id_, x, y, sx=40, sy=40, sz=40, yaw=0.0, color="red"):
    return WorldObject(id_, "block", color, [x, y, -174.66 + sz / 2.0], [sx, sy, sz], yaw_deg=yaw)


def test_choose_grasp_prefers_short_axis_when_clear():
    o = blk("o", -480, -250, 60, 30)
    yaw, opening, clear = P.choose_grasp(o, [o], 25.0)
    assert yaw % 180 == pytest.approx(90)         # close across the 30 mm side
    assert opening == pytest.approx(55)
    assert clear == float("inf")


def test_choose_grasp_switches_axis_when_neighbours_block_it():
    o = blk("o", -480, -250)
    nb = [blk("a", -480, -205, color="blue"), blk("b", -480, -295, color="green")]  # 5 mm gaps along Y
    yaw, opening, clear = P.choose_grasp(o, [o] + nb, 25.0)
    assert abs(((yaw % 180) + 90) % 180 - 90) < 1e-6 or yaw % 180 == pytest.approx(0)
    assert yaw % 180 == pytest.approx(0)          # fingers along X, where it is clear
    assert clear > 0


def test_choose_grasp_shrinks_margin_before_giving_up():
    o = blk("o", -480, -250)
    nb = [blk("a", -480, -195, color="blue"), blk("b", -480, -305, color="green"),   # 15 mm gaps along Y
          blk("c", -425, -250, color="yellow"), blk("d", -535, -250, color="purple")]  # 15 mm gaps along X
    yaw, opening, clear = P.choose_grasp(o, [o] + nb, 25.0)
    assert opening < 65 and clear > 0            # 40 + 6 margin + 10 finger fits in 15 mm gaps


def test_choose_grasp_infeasible_when_boxed_in():
    o = blk("o", -480, -250)
    nb = [blk("a", -480, -205, color="blue"), blk("b", -480, -295, color="green"),
          blk("c", -435, -250, color="yellow"), blk("d", -525, -250, color="purple")]
    with pytest.raises(P.GraspInfeasible):
        P.choose_grasp(o, [o] + nb, 25.0)


def test_choose_grasp_too_wide():
    with pytest.raises(P.GraspInfeasible):
        P.choose_grasp(blk("o", -480, -250, 120, 120), [], 25.0)


class _W:
    table_z = 28.34; min_clearance_mm = 25.0
class _R:
    tool_length_mm = 203.0


def test_grasp_height_tolerates_noisy_height_but_refuses_flat_objects():
    assert P.grasp_height(blk("o", 0, 0, sz=40), _W, _R, 20.0) == pytest.approx(25.0)
    assert P.grasp_height(blk("o", 0, 0, sz=28), _W, _R, 20.0) == pytest.approx(25.0)  # tries at the floor
    with pytest.raises(P.GraspInfeasible):
        P.grasp_height(blk("o", 0, 0, sz=8), _W, _R, 20.0)
    assert "clearance floor" in P.explain_miss(blk("o", 0, 0, sz=20), _W, _R, 25.0)


def test_look_avg_reduces_noise_and_handles_yaw_wrap():
    w = SimWorld.from_scene("two_blocks", seed=5, noise=NoiseModel(xy_sigma_mm=8, yaw_sigma_deg=10, miss_prob=0.0))
    t = Trace()
    truth = w.get("red_block")
    single = [w.find(color="red") for _ in range(50)]
    avg = [P.look_avg(w, t, n=5, color="red") for _ in range(50)]
    err1 = sum(abs(s.position[0] - truth.position[0]) for s in single) / 50
    err5 = sum(abs(a.position[0] - truth.position[0]) for a in avg) / 50
    assert err5 < err1 * 0.7
    assert all(abs(((a.yaw_deg - truth.yaw_deg + 90) % 180) - 90) < 30 for a in avg)


def test_sim_finger_hits_neighbour():
    w = SimWorld(noise=NoiseModel(enabled=False))
    w.add(blk("o", -480, -250)); w.add(blk("n", -480, -200, color="blue"))   # 10 mm gap along Y
    r = SimRobot(w); r.set_tool(1)
    r.gripper(1000)                                                            # 105 mm open along the jaw axis
    r.move_l([-480, -250, w.surface_z + 80, 180, 0, 90])                     # jaw axis = Y → finger over the blue block
    with pytest.raises(Collision):
        r.move_l([-480, -250, w.surface_z + 25, 180, 0, 90])
    # same descent with the jaw axis along X is clean and grasps
    r.move_l([-480, -250, w.surface_z + 80, 180, 0, 0])
    r.move_l([-480, -250, w.surface_z + 25, 180, 0, 0])
    assert r.gripper(0) > 300 and r.attached is w.get("o")
