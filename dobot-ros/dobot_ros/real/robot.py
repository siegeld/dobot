"""RealRobot — Robot protocol over DobotRosClient with the engine fail-safes.

Every motion goes through, in order:
  1. stop flag                      — a pending stop aborts before anything moves
  2. robot mode check               — not in ERROR / COLLISION / DISABLED
  3. endpoint guards                — floor, workspace, tilt (Guards.check)
  4. travel cap                     — Guards.check_travel
  5. model path preflight           — Guards.check_line_path through the URDF
                                      model: reachable, inside limits, above the
                                      floor at every sample
  6. confirm (step mode)            — the dashboard modal
  7. MovL (straight-line) issue     — never MovJ for cartesian targets, so the
                                      tool cannot swing through the table between
                                      two legal endpoints
  8. watchdog during motion         — 10 Hz: live fingertip below floor, XY out
                                      of workspace, or mode ERROR/COLLISION →
                                      immediate Stop + SafetyStop raised
Speed and gripper force are clamped to the Guards caps.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Callable, List, Optional, Sequence

from dobot_ros.policies.guards import Guards
from dobot_ros.policies.types import PolicyAbort, PolicyError, Trace

ConfirmFn = Callable[[str], None]   # raises PolicyAbort on cancel / timeout

MODE_DISABLED, MODE_ENABLE, MODE_RUNNING, MODE_ERROR, MODE_PAUSE, MODE_COLLISION = 4, 5, 7, 9, 10, 11
_BAD_MODES = {MODE_ERROR, MODE_COLLISION}


class SafetyStop(PolicyError):
    """The watchdog halted the robot."""


class RealRobot:
    def __init__(
        self,
        client,
        trace: Trace,
        guards: Guards,
        tool_length_mm: float,
        get_tool_index: Callable[[], int],
        on_tool_change: Optional[Callable[[int], None]] = None,
        confirm_fn: Optional[ConfirmFn] = None,
        confirm_mode: str = "step",          # "step" (every motion) | "once" (caller confirmed up front)
        stop_event: Optional[threading.Event] = None,
        default_speed: int = 30,
        move_timeout_s: float = 30.0,
        watchdog_hz: float = 10.0,
        floor_slack_mm: float = 5.0,
    ):
        self.client = client
        self.trace = trace
        self.guards = guards
        self.tool_length_mm = float(tool_length_mm)
        self._get_tool_index = get_tool_index
        self._on_tool_change = on_tool_change
        self._confirm = confirm_fn
        self._confirm_mode = confirm_mode
        self._stop = stop_event or threading.Event()
        self._speed = guards.clamp_speed(default_speed, default_speed)
        self._timeout = move_timeout_s
        self._wd_period = 1.0 / watchdog_hz
        self._floor_slack = floor_slack_mm
        self._speed_applied: Optional[int] = None
        self._last_sample = 0.0
        self._tripped: Optional[str] = None

    # ── Robot protocol ──────────────────────────────────────────────
    def pose(self) -> List[float]:
        return [float(v) for v in self.client.get_cartesian_pose()]

    def joints(self) -> List[float]:
        return [float(v) for v in self.client.get_joint_angles()]

    def tool_index(self) -> int:
        return int(self._get_tool_index())

    def set_tool(self, index: int) -> None:
        self.trace.call("set_tool", index=index)
        self._check_stop()
        if self.tool_index() != int(index):
            self.client.set_tool(int(index))
            if self._on_tool_change:
                self._on_tool_change(int(index))
            time.sleep(0.3)  # let the feedback stream switch frames
        self.trace.ret("set_tool", tool_index=self.tool_index())

    def move_l(self, pose: Sequence[float], speed: Optional[int] = None) -> None:
        pose = [float(v) for v in pose]
        self.trace.call("move_l", pose=pose, speed=speed)
        self._check_stop()
        self._check_mode()
        tool = self.tool_index()
        start = self.pose()
        self.guards.check(pose, tool)
        self.guards.check_travel(start, pose)
        self.guards.check_line_path(start, pose, tool, seed_joints=self.joints())
        self._ask(f"MovL to X={pose[0]:.0f} Y={pose[1]:.0f} Z={pose[2]:.0f} "
                  f"RZ={pose[5]:.0f} (tool {tool}, tip {self.guards.fingertip_height(pose[2], tool):.0f} mm above table)")
        self._apply_speed(speed)
        self._issue_and_wait(lambda: self.client.move_line(pose), pose, tool)
        self.trace.ret("move_l", joints=self.joints(), pose=self.pose())

    def move_j(self, joints: Sequence[float], speed: Optional[int] = None) -> None:
        joints = [float(v) for v in joints]
        self.trace.call("move_j", joints=joints, speed=speed)
        self._check_stop()
        self._check_mode()
        tool = self.tool_index()
        from dobot_ros.sim import kinematics as K
        if not K.within_limits(joints):
            raise PolicyError(f"joint target outside limits: {joints}")
        goal_pose = K.fk(joints, self.tool_length_mm if tool >= 1 else 0.0)
        self.guards.check(goal_pose, tool)
        self.guards.check_joint_path(self.joints(), joints, tool)
        self._ask("MovJ to joints " + " ".join(f"{v:.0f}" for v in joints))
        self._apply_speed(speed)
        self._issue_and_wait(lambda: self.client.move_joints(joints, wait=False), goal_pose, tool,
                             joint_goal=joints)
        self.trace.ret("move_j", joints=self.joints(), pose=self.pose())

    def gripper(self, pos_0_1000: int, force: int = 20) -> int:
        target = max(0, min(1000, int(pos_0_1000)))
        force_c = self.guards.clamp_force(force)
        self.trace.call("gripper", pos=target, force=force_c)
        self._check_stop()
        self.client.gripper_move(target, force=force_c, speed=50, wait=True, timeout=10.0)
        # The /gripper/state topic lags the action result slightly.
        final = None
        deadline = time.monotonic() + 1.5
        last = None
        while time.monotonic() < deadline:
            p = self.client.gripper_get_position()
            if p is not None and p >= 0:
                if last is not None and abs(p - last) < 2:
                    final = p
                    break
                last = p
            time.sleep(0.1)
        if final is None:
            final = last if last is not None else target
        self._sample(force=True)
        self.trace.ret("gripper", int(final), gripper=int(final))
        return int(final)

    def gripper_pos(self) -> int:
        p = self.client.gripper_get_position()
        return int(p) if p is not None else -1

    def stop(self) -> None:
        self._stop.set()
        try:
            self.client.stop()
        except Exception:
            pass

    # ── internals ───────────────────────────────────────────────────
    def _check_stop(self) -> None:
        if self._stop.is_set():
            raise PolicyAbort("stop requested")

    def _check_mode(self) -> None:
        mode = int(self.client.get_robot_mode())
        if mode in _BAD_MODES:
            raise SafetyStop(f"robot mode {mode} (error/collision) — clear the alarm first")
        if mode == MODE_DISABLED:
            raise SafetyStop("robot is not enabled")

    def _ask(self, msg: str) -> None:
        if self._confirm is not None and self._confirm_mode == "step":
            self._confirm(msg)
            self._check_stop()

    def _apply_speed(self, speed: Optional[int]) -> None:
        s = self.guards.clamp_speed(speed, self._speed)
        if s != self._speed_applied:
            self.client.set_speed_factor(s)
            self._speed_applied = s

    def _issue_and_wait(self, issue, goal_pose: Sequence[float], tool: int,
                        joint_goal: Optional[Sequence[float]] = None) -> None:
        """Issue a motion command and poll until it arrives, with the
        watchdog running the whole time."""
        self._tripped = None
        wd_stop = threading.Event()
        wd = threading.Thread(target=self._watchdog, args=(wd_stop, tool), daemon=True, name="policy-watchdog")
        wd.start()
        try:
            res = issue()
            if res not in (0, None):
                raise PolicyError(f"motion command rejected by controller (res={res})")
            t0 = time.monotonic()
            while True:
                if self._tripped:
                    raise SafetyStop(self._tripped)
                if self._stop.is_set():
                    self.client.stop()
                    raise PolicyAbort("stop requested during motion")
                if self._arrived(goal_pose, joint_goal):
                    break
                if time.monotonic() - t0 > self._timeout:
                    self.client.stop()
                    raise PolicyError(f"motion did not reach goal within {self._timeout:.0f}s")
                self._sample()
                time.sleep(0.1)
            # settle: wait for the controller to leave RUNNING
            for _ in range(20):
                if int(self.client.get_robot_mode()) != MODE_RUNNING:
                    break
                time.sleep(0.05)
            if self._tripped:
                raise SafetyStop(self._tripped)
        finally:
            wd_stop.set()
            wd.join(timeout=1.0)
            self._sample(force=True)

    def _arrived(self, goal_pose: Sequence[float], joint_goal: Optional[Sequence[float]]) -> bool:
        if joint_goal is not None:
            cur = self.joints()
            return max(abs(a - b) for a, b in zip(cur, joint_goal)) < 0.5
        cur = self.pose()
        if math.dist(cur[:3], goal_pose[:3]) > 2.0:
            return False
        for i in range(3, 6):
            d = abs((cur[i] - goal_pose[i] + 180.0) % 360.0 - 180.0)
            if d > 3.0:
                return False
        return True

    def _watchdog(self, stop: threading.Event, tool: int) -> None:
        """Live fail-safe: halt the arm the moment the feedback shows it
        somewhere the guards forbid, or the controller reports an alarm."""
        z_min = self.guards.min_pose_z(tool) - self._floor_slack
        ws = self.guards.workspace
        while not stop.is_set():
            try:
                pose = self.pose()
                mode = int(self.client.get_robot_mode())
                reason = None
                if pose[2] < z_min:
                    reason = f"watchdog: live Z={pose[2]:.1f} below floor {z_min:.1f}"
                elif ws is not None and not ws.contains(pose[0], pose[1], slack_mm=15.0):
                    reason = f"watchdog: live XY=({pose[0]:.0f},{pose[1]:.0f}) left the workspace"
                elif mode in _BAD_MODES:
                    reason = f"watchdog: robot mode {mode} (error/collision)"
                if reason:
                    self._tripped = reason
                    try:
                        self.client.stop()
                    except Exception:
                        pass
                    self.trace.log(reason)
                    return
            except Exception as e:  # feedback hiccup — keep watching
                self.trace.log(f"watchdog read failed: {e}")
            stop.wait(self._wd_period)

    def _sample(self, force: bool = False) -> None:
        now = time.monotonic()
        if not force and now - self._last_sample < 0.2:
            return
        self._last_sample = now
        try:
            self.trace.sample(joints=self.joints(), pose=self.pose(),
                              gripper=self.gripper_pos(), tool_index=self.tool_index())
        except Exception:
            pass
