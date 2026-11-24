"""RealRobot against a fake DobotRosClient: proves the hardware path's
fail-safes without hardware. Every test here is a thing that must hold
on the arm tomorrow."""
import threading
import time
import pytest

from dobot_ros.policies.guards import Guards, TiltViolation, WorkspaceBounds
from dobot_ros.policies.types import FloorViolation, PolicyAbort, Trace, Unreachable
from dobot_ros.real.robot import RealRobot, SafetyStop
from dobot_ros.sim import kinematics as K

SURFACE = 28.34 - 203.0


class FakeClient:
    """Minimal DobotRosClient stand-in. Motion is simulated as a short
    timed interpolation so the watchdog has something to watch."""

    def __init__(self, pose, mode=5, move_s=0.35):
        self.pose = list(pose)
        self.mode = mode
        self.move_s = move_s
        self.calls = []
        self.speed = None
        self.grip = 1000
        self.stopped = 0
        self._thr = None
        self.feedback_override = None   # callable(t_frac) -> pose during motion

    def get_cartesian_pose(self): return list(self.pose)
    def get_joint_angles(self): return K.ik(self.pose, tool_length_mm=203.0)
    def get_robot_mode(self): return self.mode
    def set_speed_factor(self, s): self.speed = s; self.calls.append(("speed", s)); return 0
    def set_tool(self, i): self.calls.append(("tool", i)); return 0
    def stop(self): self.stopped += 1; self.calls.append(("stop",)); return 0
    def gripper_get_position(self): return self.grip
    def gripper_move(self, pos, force=50, speed=50, wait=True, timeout=10.0):
        self.calls.append(("gripper", pos, force)); self.grip = 404 if pos == 0 else pos; return 1

    def _animate(self, goal):
        start = list(self.pose); t0 = time.monotonic()
        while True:
            a = min(1.0, (time.monotonic() - t0) / self.move_s)
            self.pose = (self.feedback_override(a) if self.feedback_override
                         else [s + a * (g - s) for s, g in zip(start, goal)])
            if a >= 1.0: break
            time.sleep(0.02)

    def move_line(self, pose):
        self.calls.append(("movl", list(pose)))
        self._thr = threading.Thread(target=self._animate, args=(list(pose),), daemon=True); self._thr.start(); return 0

    def move_pose(self, pose):            # MovJ — must never be used for cartesian targets
        self.calls.append(("movj", list(pose))); raise AssertionError("MovJ used for a cartesian target")

    def move_joints(self, joints, wait=False):
        self.calls.append(("movj_joints", list(joints)))
        goal = K.fk(joints, 203.0)
        self._thr = threading.Thread(target=self._animate, args=(goal,), daemon=True); self._thr.start(); return 0


def make(confirm=None, mode="step", **kw):
    start = [-480.0, -250.0, SURFACE + 150.0, 180.0, 0.0, 0.0]
    client = FakeClient(start, **kw)
    g = Guards(28.34, 203.0, 25.0, WorkspaceBounds(-676, -271, -679, 86))
    stop = threading.Event()
    calls = []
    def cf(msg): calls.append(msg); (confirm or (lambda m: None))(msg)
    r = RealRobot(client, Trace(), g, 203.0, get_tool_index=lambda: 1, confirm_fn=cf, confirm_mode=mode,
                  stop_event=stop, move_timeout_s=5.0, watchdog_hz=50.0)
    return client, r, calls, stop


def test_move_l_uses_movl_after_preflight_and_confirm():
    c, r, confirms, _ = make()
    r.move_l([-480, -250, SURFACE + 60, 180, 0, 30], speed=100)
    kinds = [x[0] for x in c.calls]
    assert "movl" in kinds and "movj" not in kinds
    assert c.speed == 50                                  # capped
    assert len(confirms) == 1 and "MovL" in confirms[0]
    assert r.pose()[2] == pytest.approx(SURFACE + 60, abs=0.1)


def test_rejects_floor_tilt_travel_before_any_command():
    c, r, _, _ = make()
    with pytest.raises(FloorViolation):
        r.move_l([-480, -250, SURFACE + 10, 180, 0, 0])
    with pytest.raises(TiltViolation):
        r.move_l([-480, -250, SURFACE + 60, 150, 0, 0])
    with pytest.raises(Unreachable):
        r.move_l([-661, -676, SURFACE + 60, 180, 0, 0])
    assert not any(x[0] == "movl" for x in c.calls)


def test_confirm_cancel_aborts_without_motion():
    def cancel(msg): raise PolicyAbort("user cancelled")
    c, r, _, _ = make(confirm=cancel)
    with pytest.raises(PolicyAbort):
        r.move_l([-480, -250, SURFACE + 60, 180, 0, 0])
    assert not any(x[0] == "movl" for x in c.calls)


def test_once_mode_skips_per_move_confirm():
    c, r, confirms, _ = make(mode="once")
    r.move_l([-480, -250, SURFACE + 60, 180, 0, 0])
    assert confirms == []


def test_error_mode_blocks_motion():
    c, r, _, _ = make(mode="once")
    c.mode = 9
    with pytest.raises(SafetyStop):
        r.move_l([-480, -250, SURFACE + 60, 180, 0, 0])
    c.mode = 4
    with pytest.raises(SafetyStop):
        r.move_l([-480, -250, SURFACE + 60, 180, 0, 0])


def test_watchdog_stops_when_feedback_dips_below_floor():
    c, r, _, _ = make(mode="once")
    goal = [-480.0, -250.0, SURFACE + 60.0, 180.0, 0.0, 0.0]
    # Controller "overshoots": feedback sags 40 mm under the floor mid-move.
    z0 = SURFACE + 150.0
    def sag(a):
        if a < 0.3:   z = z0 + (goal[2] - z0) * (a / 0.3)       # normal descent
        elif a < 0.9: z = SURFACE + 25.0 - 40.0                  # 40 mm under the floor
        else:         z = goal[2]
        return [goal[0], goal[1], z, 180.0, 0.0, 0.0]
    c.feedback_override = sag
    with pytest.raises(SafetyStop) as ei:
        r.move_l(goal)
    assert "watchdog" in str(ei.value) and c.stopped >= 1


def test_watchdog_stops_on_collision_mode():
    c, r, _, _ = make(mode="once")
    def flip(a):
        if a > 0.4: c.mode = 11
        return [-480.0, -250.0, SURFACE + 150.0 - a * 90.0, 180.0, 0.0, 0.0]
    c.feedback_override = flip
    with pytest.raises(SafetyStop):
        r.move_l([-480, -250, SURFACE + 60, 180, 0, 0])
    assert c.stopped >= 1


def test_stop_event_during_motion():
    c, r, _, stop = make(mode="once", move_s=1.0)
    threading.Timer(0.2, stop.set).start()
    with pytest.raises(PolicyAbort):
        r.move_l([-480, -250, SURFACE + 60, 180, 0, 0])
    assert c.stopped >= 1


def test_gripper_force_cap_and_jaw_readback():
    c, r, _, _ = make(mode="once")
    pos = r.gripper(0, force=100)
    assert c.calls[-1] == ("gripper", 0, 60)
    assert pos == 404


def test_move_j_preflights_joint_path_and_uses_joint_movj():
    c, r, _, _ = make(mode="once")
    q = K.ik([-480, -250, SURFACE + 100, 180, 0, 20], tool_length_mm=203.0)
    r.move_j(q)
    assert c.calls[-1][0] == "movj_joints"
    # a joint target whose FK is under the floor is refused before the call
    low = K.ik([-480, -250, SURFACE + 5, 180, 0, 0], tool_length_mm=203.0)
    with pytest.raises(FloorViolation):
        r.move_j(low)
