"""SimRobot — kinematic CR5 + AG-105 implementing the Robot protocol.

Motion is cartesian (move_l) or joint (move_j) interpolation with the
shared guards applied to every sample, IK seeded from the previous sample
so the arm stays on one configuration branch (what a real MovL does).
The gripper is a stroke with a geometric grasp rule; a grasped object is
attached to the fingertip frame and moves with it.
"""

from __future__ import annotations

import math
import threading
import time
from typing import List, Optional, Sequence

import numpy as np

from dobot_ros.policies.guards import Guards
from dobot_ros.policies.types import Collision, PolicyAbort, Trace, Unreachable, WorldObject
from dobot_ros.sim import kinematics as K
from dobot_ros.sim.world import SimWorld, _footprint_contains

GRIPPER_STROKE_MM = 105.0
FINGER_W = 10.0                 # finger pad thickness along the jaw axis (mm)
MAX_TCP_SPEED_MM_S = 1000.0     # at speed factor 100
MAX_ROT_SPEED_DEG_S = 180.0
MAX_JOINT_SPEED_DEG_S = 180.0
GRIPPER_FULL_STROKE_S = 0.7


class SimClock:
    """Virtual time. ``pace`` > 0 also sleeps real time (1.0 = real speed)."""

    def __init__(self, pace: float = 0.0):
        self.t = 0.0
        self.pace = pace

    def advance(self, dt: float) -> None:
        self.t += dt
        if self.pace > 0 and dt > 0:
            time.sleep(dt / self.pace)

    def __call__(self) -> float:
        return self.t


class SimRobot:
    def __init__(
        self,
        world: SimWorld,
        trace: Optional[Trace] = None,
        start_joints: Optional[Sequence[float]] = None,
        tool_index: int = 0,
        gripper_pos: int = 1000,
        default_speed: int = 30,
        clock: Optional[SimClock] = None,
        guards: Optional[Guards] = None,
        step_mm: float = 5.0,
        step_deg: float = 3.0,
        sample_every_s: float = 0.1,
        stop_event: Optional[threading.Event] = None,
    ):
        self.world = world
        self.tool_length_mm = world.tool_length_mm
        self.clock = clock or SimClock()
        self.trace = trace or Trace(clock=self.clock)
        self.guards = guards or Guards(world.table_z, world.tool_length_mm,
                                       world.min_clearance_mm, world.workspace)
        self._tool_index = int(tool_index)
        self._jaw_pos = int(gripper_pos)
        self._speed = int(default_speed)
        self._step_mm = step_mm
        self._step_deg = step_deg
        self._sample_every = sample_every_s
        self._last_sample_t = -1.0
        self._stop = stop_event or threading.Event()
        self._attached: Optional[WorldObject] = None
        self._attach_offset = np.zeros(3)   # object centre in fingertip frame (m)
        self._attach_yaw_offset = 0.0
        self.motion_count = 0
        self.total_path_mm = 0.0
        if start_joints is None:
            cx = (world.workspace.x_min + world.workspace.x_max) / 2.0
            cy = (world.workspace.y_min + world.workspace.y_max) / 2.0
            start_joints = K.ik([cx, cy, world.surface_z + 250.0, 180.0, 0.0, 0.0],
                                tool_length_mm=world.tool_length_mm)
        self._joints = [float(v) for v in start_joints]
        self._sample(force=True)

    # ── Robot protocol ──────────────────────────────────────────────
    def _active_tool_len(self) -> float:
        return self.tool_length_mm if self._tool_index >= 1 else 0.0

    def pose(self) -> List[float]:
        return K.fk(self._joints, self._active_tool_len())

    def joints(self) -> List[float]:
        return list(self._joints)

    def tool_index(self) -> int:
        return self._tool_index

    def set_tool(self, index: int) -> None:
        self.trace.call("set_tool", index=index)
        self._tool_index = int(index)
        self.clock.advance(0.05)
        self.trace.ret("set_tool", tool_index=self._tool_index)

    def gripper_pos(self) -> int:
        return self._jaw_pos

    def stop(self) -> None:
        self._stop.set()

    def move_l(self, pose: Sequence[float], speed: Optional[int] = None) -> None:
        pose = [float(v) for v in pose]
        self.trace.call("move_l", pose=pose, speed=speed)
        start = self.pose()
        self.guards.check(pose, self._tool_index)
        self.guards.check_travel(start, pose)
        goal = list(pose)
        # Unwrap angles toward the start so 180 → -180 isn't a full spin.
        for i in range(3, 6):
            d = (goal[i] - start[i] + 180.0) % 360.0 - 180.0
            goal[i] = start[i] + d
        dist = math.dist(start[:3], goal[:3])
        rot = max(abs(goal[i] - start[i]) for i in range(3, 6))
        n = max(1, int(math.ceil(max(dist / self._step_mm, rot / self._step_deg))))
        sp = self.guards.clamp_speed(speed, self._speed)
        v = MAX_TCP_SPEED_MM_S * sp / 100.0
        w = MAX_ROT_SPEED_DEG_S * sp / 100.0
        dt = max(dist / v if v > 0 else 0.0, rot / w if w > 0 else 0.0) / n
        tl = self._active_tool_len()
        q = list(self._joints)
        for k in range(1, n + 1):
            self._check_stop()
            a = k / n
            p = [start[i] + a * (goal[i] - start[i]) for i in range(6)]
            try:
                q = K.ik(p, seed_deg=q, tool_length_mm=tl)
            except K.IKError as e:
                self.trace.error("move_l", e, joints=list(self._joints), pose=self.pose())
                raise Unreachable(str(e)) from None
            self._joints = q
            self._after_step()
            self.clock.advance(dt)
            self._sample()
        self.motion_count += 1
        self.total_path_mm += dist
        self._sample(force=True)
        self.trace.ret("move_l", joints=list(self._joints), pose=self.pose())

    def move_j(self, joints: Sequence[float], speed: Optional[int] = None) -> None:
        goal = [float(v) for v in joints]
        self.trace.call("move_j", joints=goal, speed=speed)
        if not K.within_limits(goal):
            raise Unreachable(f"joint target outside limits: {goal}")
        start = list(self._joints)
        span = max(abs(g - s) for g, s in zip(goal, start))
        n = max(1, int(math.ceil(span / self._step_deg)))
        w = MAX_JOINT_SPEED_DEG_S * self.guards.clamp_speed(speed, self._speed) / 100.0
        dt = (span / w if w > 0 else 0.0) / n
        for k in range(1, n + 1):
            self._check_stop()
            a = k / n
            self._joints = [s + a * (g - s) for g, s in zip(goal, start)]
            self.guards.check(self.pose(), self._tool_index)
            self._after_step()
            self.clock.advance(dt)
            self._sample()
        self.motion_count += 1
        self._sample(force=True)
        self.trace.ret("move_j", joints=list(self._joints), pose=self.pose())

    def gripper(self, pos_0_1000: int, force: int = 20) -> int:
        target = max(0, min(1000, int(pos_0_1000)))
        force = self.guards.clamp_force(force)
        self.trace.call("gripper", pos=target, force=force)
        w_target = target / 1000.0 * GRIPPER_STROKE_MM
        w_cur = self._jaw_pos / 1000.0 * GRIPPER_STROKE_MM
        self.clock.advance(abs(w_target - w_cur) / GRIPPER_STROKE_MM * GRIPPER_FULL_STROKE_S)
        if w_target < w_cur:
            if self._attached is None:
                hit = self._object_between_fingers(w_cur, w_target)
                if hit is not None:
                    obj, w_obj = hit
                    self._attach(obj)
                    self._jaw_pos = int(round(w_obj / GRIPPER_STROKE_MM * 1000.0))
                else:
                    self._jaw_pos = target
            # else: already holding — jaws can't close further.
        else:
            if self._attached is not None:
                w_obj = self._attached.width_across(self._jaw_yaw_deg())
                if w_target > w_obj + 1.0:
                    self._detach()
            self._jaw_pos = target
        self._sample(force=True)
        self.trace.ret("gripper", self._jaw_pos, gripper=self._jaw_pos)
        return self._jaw_pos

    # ── simulation internals ────────────────────────────────────────
    def _check_stop(self) -> None:
        if self._stop.is_set():
            raise PolicyAbort("stop requested")

    def _tip_matrix(self) -> np.ndarray:
        return K.fk_matrix(self._joints, self.tool_length_mm)

    def _tip_xyz_mm(self) -> np.ndarray:
        return self._tip_matrix()[:3, 3] * 1000.0

    def _jaw_yaw_deg(self) -> float:
        R = self._tip_matrix()[:3, :3]
        return math.degrees(math.atan2(R[1, 0], R[0, 0]))

    def _after_step(self) -> None:
        """Carry the attached object; detect running into free objects."""
        T = self._tip_matrix()
        tip = T[:3, 3] * 1000.0
        if self._attached is not None:
            c = T @ np.array([*self._attach_offset, 1.0])
            self._attached.position = [float(c[0] * 1000.0), float(c[1] * 1000.0), float(c[2] * 1000.0)]
            self._attached.yaw_deg = self._jaw_yaw_deg() + self._attach_yaw_offset
        for o in self.world.truth():
            if o is self._attached or o.support == "gripper":
                continue
            # Each finger is a FINGER_W-wide pad centred FINGER_OFFSET from the
            # tool axis along the jaw direction. A finger inside a free
            # object's volume is a crash — including the target itself when
            # the jaws are narrower than it (descending onto it).
            for fx, fy in self._finger_points_mm():
                if _footprint_contains(o, fx, fy, FINGER_W / 2.0 - 1.0) and tip[2] < o.top_z - 1.0:
                    exc = Collision(f"finger hit {o.color} {o.label} '{o.id}' at Z={tip[2]:.0f} "
                                    f"(top {o.top_z:.0f}); jaw opening {self._jaw_pos / 10:.0f}% "
                                    f"at yaw {self._jaw_yaw_deg():.0f}°")
                    self.trace.error("move", exc, joints=list(self._joints), pose=self.pose())
                    raise exc
            if self._attached is not None and _footprint_contains(o, *self._attached.position[:2], 0.0):
                if self._attached.bottom_z < o.top_z - 1.0 and o.top_z > self.world.surface_z + 1.0:
                    exc = Collision(f"carried object '{self._attached.id}' hit '{o.id}'")
                    self.trace.error("move", exc, joints=list(self._joints), pose=self.pose())
                    raise exc

    def _finger_points_mm(self):
        """World XY of the two finger pads at the current jaw opening."""
        T = self._tip_matrix()
        half = self._jaw_pos / 1000.0 * GRIPPER_STROKE_MM / 2.0 + FINGER_W / 2.0
        out = []
        for sgn in (-1.0, 1.0):
            c = T @ np.array([sgn * half / 1000.0, 0.0, 0.0, 1.0])
            out.append((float(c[0] * 1000.0), float(c[1] * 1000.0)))
        return out

    def _object_between_fingers(self, w_open: float, w_target: float):
        tip = self._tip_xyz_mm()
        jaw_yaw = self._jaw_yaw_deg()
        best = None
        for o in self.world.truth():
            if o.support == "gripper":
                continue
            if not _footprint_contains(o, tip[0], tip[1], 3.0):
                continue
            # Fingertips must be below the object's top and not under its base.
            if not (o.bottom_z - 3.0 <= tip[2] <= o.top_z - 2.0):
                continue
            w_obj = o.width_across(jaw_yaw)
            if w_target < w_obj <= w_open:
                d = math.hypot(tip[0] - o.position[0], tip[1] - o.position[1])
                if best is None or d < best[2]:
                    best = (o, w_obj, d)
        return (best[0], best[1]) if best else None

    def _attach(self, obj: WorldObject) -> None:
        T = self._tip_matrix()
        Tinv = np.linalg.inv(T)
        c = Tinv @ np.array([obj.position[0] / 1000.0, obj.position[1] / 1000.0, obj.position[2] / 1000.0, 1.0])
        self._attach_offset = c[:3]
        self._attach_yaw_offset = obj.yaw_deg - self._jaw_yaw_deg()
        obj.support = "gripper"
        self._attached = obj
        self.trace.log(f"grasped {obj.color} {obj.label} '{obj.id}'")

    def _detach(self) -> None:
        obj = self._attached
        self._attached = None
        x, y = obj.position[0], obj.position[1]
        base = self.world.support_z_at(x, y, exclude=obj.id)
        obj.position[2] = base + obj.size[2] / 2.0
        obj.support = "table" if base <= self.world.surface_z + 1e-6 else self._support_id_at(x, y, obj.id)
        self.trace.log(f"released '{obj.id}' onto {obj.support}")

    def _support_id_at(self, x: float, y: float, exclude: str) -> str:
        best, best_z = "table", self.world.surface_z
        for o in self.world.truth():
            if o.id == exclude or o.support == "gripper":
                continue
            if _footprint_contains(o, x, y, 0.0) and o.top_z > best_z:
                best, best_z = o.id, o.top_z
        return best

    def _sample(self, force: bool = False) -> None:
        t = self.clock.t
        if not force and t - self._last_sample_t < self._sample_every:
            return
        self._last_sample_t = t
        self.trace.sample(joints=list(self._joints), pose=self.pose(), gripper=self._jaw_pos,
                          tool_index=self._tool_index, objects=self.world.snapshot())

    # ── convenience for tests ───────────────────────────────────────
    @property
    def attached(self) -> Optional[WorldObject]:
        return self._attached

    def fingertip_mm(self) -> List[float]:
        return [float(v) for v in self._tip_xyz_mm()]
