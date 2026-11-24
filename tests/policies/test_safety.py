"""Engine fail-safes: these hold no matter what a policy asks for."""
import pytest

from dobot_ros.policies.guards import Guards, TiltViolation, TravelViolation, WorkspaceBounds
from dobot_ros.policies.types import FloorViolation, Unreachable, WorkspaceViolation, Trace
from dobot_ros.sim.noise import NoiseModel
from dobot_ros.sim.robot import SimRobot
from dobot_ros.sim.world import SimWorld
from dobot_ros.policies.base import Policy
from dobot_ros.policies.types import PolicyResult


def guards():
    return Guards(table_z=28.3, tool_length_mm=203.0, min_clearance_mm=25.0,
                  workspace=WorkspaceBounds(-676, -271, -679, 86))


def test_tilt_rejected():
    g = guards()
    g.check([-480, -250, 100, 180, 0, 30], 1)
    g.check([-480, -250, 100, 170, 10, 30], 1)        # inside ±15°
    with pytest.raises(TiltViolation):
        g.check([-480, -250, 100, 150, 0, 30], 1)
    with pytest.raises(TiltViolation):
        g.check([-480, -250, 100, 180, 25, 30], 1)
    g.max_tilt_deg = None
    g.check([-480, -250, 100, 90, 0, 0], 1)           # disabled → anything goes


def test_travel_cap():
    g = guards()
    g.check_travel([-300, 0, 0, 180, 0, 0], [-600, 0, 0, 180, 0, 0])
    with pytest.raises(TravelViolation):
        g.check_travel([-300, 80, 0, 180, 0, 0], [-676, -679, 0, 180, 0, 0])


def test_speed_and_force_caps():
    g = guards()
    assert g.clamp_speed(100, 30) == 50 and g.clamp_speed(None, 30) == 30 and g.clamp_speed(0, 30) == 1
    assert g.clamp_force(100) == 60 and g.clamp_force(5) == 20


def test_line_path_preflight_rejects_unreachable_sample():
    g = guards()
    surface = 28.3 - 203.0
    ok = g.check_line_path([-480, -250, surface + 100, 180, 0, 0], [-300, -600, surface + 100, 180, 0, 0], 1)
    assert len(ok) >= 10
    with pytest.raises(Unreachable):
        g.check_line_path([-480, -250, surface + 100, 180, 0, 0], [-661, -676, surface + 60, 180, 0, 0], 1)


def test_joint_path_preflight_catches_floor_dip():
    g = guards()
    from dobot_ros.sim import kinematics as K
    surface = 28.3 - 203.0
    # Two legal endpoints on opposite sides of the base; the joint interpolation
    # between them swings the tool low — exactly what a cartesian MovJ would do.
    q0 = K.ik([-350, 60, surface + 30, 180, 0, 0], tool_length_mm=203)
    q1 = K.ik([-350, -600, surface + 30, 180, 0, 90], tool_length_mm=203)
    with pytest.raises((FloorViolation, WorkspaceViolation)):
        g.check_joint_path(q0, q1, 1)


def test_sim_robot_enforces_tilt_and_caps():
    w = SimWorld.from_scene("two_blocks", noise=NoiseModel(enabled=False))
    r = SimRobot(w)
    r.set_tool(1)
    with pytest.raises(TiltViolation):
        r.move_l([-480, -250, w.surface_z + 100, 150, 0, 0])
    with pytest.raises(TravelViolation):
        r.guards.max_move_mm = 50
        r.move_l([-480, -250, w.surface_z + 100, 180, 0, 0])
    r.guards.max_move_mm = 700
    assert r.gripper(500, force=100) >= 0
    ev = [e for e in r.trace.events if e.kind == "call" and e.name == "gripper"][-1]
    assert ev.args["force"] == 60


class _ReckLess(Policy):
    """A policy that tries to drive through the table."""
    name = "reckless"; slug = "_reckless"; description = ""
    @classmethod
    def parameter_defs(cls): return []
    def run(self, robot, world, params, trace):
        robot.set_tool(1)
        p = robot.pose()
        robot.move_l([p[0], p[1], world.table_z - robot.tool_length_mm - 20, 180, 0, p[5]])
        return PolicyResult(ok=True)


def test_rehearsal_blocks_a_table_crash(tmp_path):
    """Robot-target run: the sim dry-run must refuse before any hardware call."""
    from dobot_ros.policies.registry import PolicyRegistry
    from dobot_ros.policies.runner import PolicyRunner, RunRequest
    reg = PolicyRegistry(params_dir=tmp_path)
    reg._classes["_reckless"] = _ReckLess
    world = SimWorld.from_scene("two_blocks", noise=NoiseModel(enabled=False))
    fake_robot = SimRobot(world)                       # stands in for RealRobot
    calls = {"moves": 0}
    orig = fake_robot.move_l
    def spy(*a, **k):
        calls["moves"] += 1
        return orig(*a, **k)
    fake_robot.move_l = spy
    runner = PolicyRunner(reg, tmp_path / "t", robot_factory=lambda **kw: (fake_robot, world))
    rec = runner.run(RunRequest(slug="_reckless", target="robot"))
    assert rec["result"]["ok"] is False
    assert "FloorViolation" in rec["result"]["reason"]
    assert rec["rehearsal"]["blocked"] is True
    assert calls["moves"] == 0                         # the "real" robot never moved


def test_rehearsal_passes_a_sane_policy(tmp_path):
    from dobot_ros.policies.registry import PolicyRegistry
    from dobot_ros.policies.runner import PolicyRunner, RunRequest
    reg = PolicyRegistry(params_dir=tmp_path)
    world = SimWorld.from_scene("two_blocks", noise=NoiseModel(enabled=False))
    fake_robot = SimRobot(world)
    runner = PolicyRunner(reg, tmp_path / "t", robot_factory=lambda **kw: (fake_robot, world))
    rec = runner.run(RunRequest(slug="pick_colored_block", target="robot", params={"color": "red"}))
    assert rec["rehearsal"]["blocked"] is False and rec["rehearsal"]["ok"] is True
    assert rec["result"]["ok"] is True
