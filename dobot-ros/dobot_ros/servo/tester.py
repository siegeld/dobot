"""Manual streaming-servo tester for ServoP.

Additive: does not touch any existing control path. MovJ/MovL/jog/pick remain
fully functional. Use this tool to validate ServoP behavior before wiring VLA
or other high-rate controllers to it.

Design:
- Single fixed-rate thread streams ServoP targets.
- Target is the anchor pose (captured at start) plus a live 6-D offset.
- Offset comes from either (a) jog sliders/keyboard, or (b) a pattern generator.
- Every commanded pose is clamped to workspace bounds via safety.apply().
- ServoP expects monotonic streaming — pauses halt the controller. The loop
  therefore keeps sending the last target even when the user isn't actively
  jogging ("hold pose" mode).
"""

from __future__ import annotations

import collections
import logging
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Deque, List, Optional

from dobot_ros.servo.patterns import Pattern
from dobot_ros.vla.safety import SafetyLimits, clamp_pose, apply as safety_apply


log = logging.getLogger(__name__)


# ── Config / status ────────────────────────────────────────────────────
@dataclass
class ServoConfig:
    """Streaming + ServoP tuning parameters. All live-editable."""
    servo_rate_hz: float = 30.0
    t: float = 0.05
    aheadtime: float = 50.0
    gain: float = 500.0

    # Hard velocity caps applied inside the tick loop. The commanded offset
    # advances toward whatever the caller/pattern sets as the target offset
    # at most max_velocity_*_step per tick (= max_velocity_* × dt). This is
    # the ONLY guaranteed bound on servo-commanded motion: if a slider
    # snaps from +50 mm to 0, or a pattern jumps, the cap still holds.
    # XYZ is clamped as 3-vector magnitude so direction is preserved.
    # RPY is clamped per-axis independently (pitch/roll/yaw are different
    # physical degrees of freedom and don't share a magnitude constraint).
    max_velocity_xyz: float = 100.0  # mm/s
    max_velocity_rpy: float = 45.0   # deg/s

    # How long to keep streaming "hold" after the last target update before
    # giving up and parking. None = forever (stay in servo mode until stop()).
    idle_timeout_s: Optional[float] = None


@dataclass
class ServoStatus:
    running: bool = False
    anchor_pose: List[float] = field(default_factory=list)
    last_target_offset: List[float] = field(default_factory=lambda: [0.0] * 6)
    last_commanded_pose: List[float] = field(default_factory=list)
    step: int = 0
    clamps: int = 0
    overruns: int = 0
    last_latency_ms: float = 0.0
    p50_latency_ms: float = 0.0
    p95_latency_ms: float = 0.0
    last_error: Optional[str] = None
    mode: str = "idle"   # "idle" | "jog" | "pattern"
    pattern_name: Optional[str] = None
    pattern_elapsed_s: float = 0.0


# ── Tester ─────────────────────────────────────────────────────────────
class ServoTester:
    def __init__(
        self,
        ros_client,
        config: Optional[ServoConfig] = None,
        limits: Optional[SafetyLimits] = None,
        table_plane_path: Optional[str] = None,
        log_csv_path: Optional[str] = None,
    ):
        self.ros = ros_client
        self.config = config or ServoConfig()

        if limits is not None:
            self.limits = limits
        elif table_plane_path and Path(table_plane_path).exists():
            self.limits = SafetyLimits.from_table_plane(Path(table_plane_path))
        else:
            self.limits = SafetyLimits()

        self.log_csv_path = log_csv_path

        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        # Reentrant: public methods that return status() may be called while
        # we already hold the lock in internal paths.
        self._lock = threading.RLock()

        # State exposed to the outside.
        self._anchor: List[float] = []
        self._target_offset: List[float] = [0.0] * 6
        self._target_last_updated: float = 0.0
        self._pattern: Optional[Pattern] = None
        self._pattern_start_t: float = 0.0
        # The offset we ACTUALLY command, distinct from _target_offset. The
        # tick loop rate-limits advancement from here toward _target_offset
        # at max_velocity_xyz / max_velocity_rpy. On any target change this
        # lags until the cap has had time to carry us to the new target.
        self._commanded_offset: List[float] = [0.0] * 6
        self._last_tick_time: float = 0.0
        # Rate-control input (SpaceMouse). When non-None, the tick loop
        # integrates this velocity directly into _commanded_offset and
        # ignores _target_offset / _pattern. Zero velocity on any tick
        # means the robot stops that tick — no residual "catch up" motion.
        # Set via set_target_velocity(); cleared when set_target_offset()
        # or set_pattern() is called (those are position-mode inputs).
        self._target_velocity: Optional[List[float]] = None

        self._latencies: Deque[float] = collections.deque(maxlen=200)
        self._status = ServoStatus()

        self._csv_file = None

    # ── Lifecycle ───────────────────────────────────────────────
    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self) -> ServoStatus:
        with self._lock:
            if self.is_running():
                return self.status()
            self._anchor = list(self.ros.get_cartesian_pose())
            self._target_offset = [0.0] * 6
            self._target_last_updated = time.time()
            self._pattern = None
            self._commanded_offset = [0.0] * 6
            self._last_tick_time = 0.0
            self._target_velocity = None
            self._stop.clear()
            self._status = ServoStatus(running=True, anchor_pose=list(self._anchor))
            if self.log_csv_path:
                self._open_csv()
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()
            log.info("servo tester started; anchor=%s", self._anchor)
        return self.status()

    def stop(self, timeout: float = 2.0, call_robot_stop: bool = False):
        """Halt the stream. If `call_robot_stop`, also tell the controller to stop motion."""
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=timeout)
        with self._lock:
            self._status.running = False
            self._status.mode = "idle"
            self._pattern = None
            self._close_csv()
        if call_robot_stop:
            try:
                self.ros.stop()
            except Exception as e:
                log.warning("ros.stop() during servo stop failed: %s", e)
        log.info("servo tester stopped (robot_stop=%s)", call_robot_stop)

    def emergency_stop(self):
        """STOP button handler: halt thread AND call controller Stop."""
        self.stop(timeout=1.0, call_robot_stop=True)

    # ── Live inputs ─────────────────────────────────────────────
    def set_target_offset(self, offset: List[float]) -> ServoStatus:
        """Jog-slider update (position mode). Clears pattern and velocity cmd."""
        if len(offset) != 6:
            raise ValueError("target offset must be 6-D")
        with self._lock:
            self._target_offset = [float(v) for v in offset]
            self._pattern = None
            self._target_velocity = None
            self._target_last_updated = time.time()
            self._status.mode = "jog"
            self._status.pattern_name = None
        return self.status()

    def set_target_velocity(self, velocity: List[float]) -> ServoStatus:
        """Rate-control update (SpaceMouse).

        The tick loop integrates ``velocity`` into ``_commanded_offset`` each
        tick. A zero velocity means no motion that tick — release = stop.
        The velocity itself is capped by ``max_velocity_xyz`` (as 3-vector
        magnitude) and ``max_velocity_rpy`` (per-axis) before integration.

        Clears any active pattern and position target so the rate-control
        path is not fighting a catch-up to a stale target_offset.
        """
        if len(velocity) != 6:
            raise ValueError("target velocity must be 6-D")
        with self._lock:
            self._target_velocity = [float(v) for v in velocity]
            self._pattern = None
            # Freeze the position target at wherever we've actually
            # commanded, so if the caller later flips back to position
            # mode, there's no surprise jump.
            self._target_offset = list(self._commanded_offset)
            self._target_last_updated = time.time()
            self._status.mode = "velocity"
            self._status.pattern_name = None
        return self.status()

    def set_pattern(self, pattern: Pattern, name: str) -> ServoStatus:
        """Start driving the target from a pattern generator (position mode)."""
        with self._lock:
            self._pattern = pattern
            self._pattern_start_t = time.time()
            self._target_offset = [0.0] * 6
            self._target_velocity = None
            self._target_last_updated = time.time()
            self._status.mode = "pattern"
            self._status.pattern_name = name
        return self.status()

    def clear_pattern(self) -> ServoStatus:
        with self._lock:
            self._pattern = None
            self._status.mode = "jog"
            self._status.pattern_name = None
        return self.status()

    _CONFIG_TYPES = {
        "servo_rate_hz": float, "t": float,
        "aheadtime": float, "gain": float,
        "max_velocity_xyz": float, "max_velocity_rpy": float,
        "idle_timeout_s": (float, type(None)),
    }

    def update_config(self, **kwargs) -> ServoStatus:
        """Update tuning parameters live. Validates types and field names."""
        with self._lock:
            for k, v in kwargs.items():
                if k not in self._CONFIG_TYPES:
                    continue
                expected = self._CONFIG_TYPES[k]
                try:
                    if expected is float:
                        v = float(v)
                    elif isinstance(expected, tuple) and type(None) in expected:
                        v = float(v) if v is not None else None
                except (TypeError, ValueError):
                    continue
                setattr(self.config, k, v)
        return self.status()

    def status(self) -> ServoStatus:
        with self._lock:
            # Snapshot so the caller can't mutate internal state.
            return ServoStatus(**self._status.__dict__)

    # ── Main loop ───────────────────────────────────────────────
    def _run(self):
        last_overrun_report = 0.0
        consecutive_failures = 0
        MAX_CONSECUTIVE_FAILURES = 10
        try:
            import math
            while not self._stop.is_set():
                tick_start = time.perf_counter()
                period = 1.0 / max(1.0, self.config.servo_rate_hz)

                # Compute this tick's dt for the velocity cap. First tick falls
                # back to the nominal period since no prior tick exists.
                if self._last_tick_time > 0.0:
                    dt = max(1e-4, tick_start - self._last_tick_time)
                else:
                    dt = period
                self._last_tick_time = tick_start

                max_xyz_step = max(0.0, self.config.max_velocity_xyz) * dt
                max_rpy_step = max(0.0, self.config.max_velocity_rpy) * dt

                # Two input modes:
                #   VELOCITY mode (set_target_velocity) — integrate the
                #     commanded velocity directly into _commanded_offset,
                #     capping the velocity itself. A zero velocity means
                #     no motion this tick: release = stop.
                #   POSITION mode (set_target_offset / patterns) — advance
                #     _commanded_offset toward _target_offset at the same
                #     velocity cap so the robot traverses to the target
                #     position at a safe rate.
                with self._lock:
                    vcmd = self._target_velocity
                if vcmd is not None:
                    vx, vy, vz = vcmd[0], vcmd[1], vcmd[2]
                    vmag = math.sqrt(vx * vx + vy * vy + vz * vz)
                    cap_xyz = max(0.0, self.config.max_velocity_xyz)
                    if vmag > cap_xyz and vmag > 0.0:
                        s = cap_xyz / vmag
                        vx, vy, vz = vx * s, vy * s, vz * s
                    self._commanded_offset[0] += vx * dt
                    self._commanded_offset[1] += vy * dt
                    self._commanded_offset[2] += vz * dt
                    cap_rpy = max(0.0, self.config.max_velocity_rpy)
                    for i in range(3, 6):
                        vi = vcmd[i]
                        if vi > cap_rpy:
                            vi = cap_rpy
                        elif vi < -cap_rpy:
                            vi = -cap_rpy
                        self._commanded_offset[i] += vi * dt
                    # Keep target_offset in sync so a future mode switch
                    # doesn't snap.
                    with self._lock:
                        self._target_offset = list(self._commanded_offset)
                else:
                    target = self._current_offset()
                    dx = target[0] - self._commanded_offset[0]
                    dy = target[1] - self._commanded_offset[1]
                    dz = target[2] - self._commanded_offset[2]
                    mag = math.sqrt(dx * dx + dy * dy + dz * dz)
                    if mag > max_xyz_step and mag > 0.0:
                        scale = max_xyz_step / mag
                        dx *= scale
                        dy *= scale
                        dz *= scale
                    self._commanded_offset[0] += dx
                    self._commanded_offset[1] += dy
                    self._commanded_offset[2] += dz
                    for i in range(3, 6):
                        d = target[i] - self._commanded_offset[i]
                        if d > max_rpy_step:
                            d = max_rpy_step
                        elif d < -max_rpy_step:
                            d = -max_rpy_step
                        self._commanded_offset[i] += d

                offset = list(self._commanded_offset)
                commanded = [self._anchor[i] + offset[i] for i in range(6)]

                # Safety: clamp against workspace bounds.
                # We DON'T clamp per-step delta here because the tester
                # explicitly wants to test arbitrary targets; workspace bounds
                # are the floor we won't cross.
                clamped, reasons = clamp_pose(commanded, self.limits)
                if reasons:
                    with self._lock:
                        self._status.clamps += 1

                # Stream the ServoP call. Time elapsed in the call is our latency.
                call_start = time.perf_counter()
                try:
                    self.ros.servo_p(
                        clamped,
                        t=self.config.t,
                        aheadtime=self.config.aheadtime,
                        gain=self.config.gain,
                    )
                    latency_ms = (time.perf_counter() - call_start) * 1000.0
                    consecutive_failures = 0
                    with self._lock:
                        self._latencies.append(latency_ms)
                        self._status.last_latency_ms = latency_ms
                        self._status.last_commanded_pose = clamped
                        self._status.last_target_offset = list(offset)
                        self._status.step += 1
                        self._refresh_percentiles()
                    self._maybe_log(clamped, offset, latency_ms)
                except Exception as e:
                    consecutive_failures += 1
                    log.warning("servo_p call failed (%d/%d): %s",
                                consecutive_failures, MAX_CONSECUTIVE_FAILURES, e)
                    with self._lock:
                        self._status.last_error = f"servo_p: {e}"
                    if consecutive_failures >= MAX_CONSECUTIVE_FAILURES:
                        log.error("servo tester aborting after %d consecutive failures", consecutive_failures)
                        break

                # Pace the loop.
                elapsed = time.perf_counter() - tick_start
                sleep = period - elapsed
                if sleep > 0:
                    self._stop.wait(timeout=sleep)
                else:
                    with self._lock:
                        self._status.overruns += 1
                    now = time.time()
                    if now - last_overrun_report > 1.0:
                        log.debug("servo tester overrun: %.1f ms over budget", -sleep * 1000)
                        last_overrun_report = now

                # Idle timeout — auto-stop if no one has touched the target in a while.
                if self.config.idle_timeout_s is not None:
                    if time.time() - self._target_last_updated > self.config.idle_timeout_s:
                        log.info("servo tester idle timeout — halting")
                        break

        except Exception as e:
            log.exception("servo tester crashed: %s", e)
            with self._lock:
                self._status.last_error = str(e)
        finally:
            # SAFETY: halt the robot when the tester exits (normal stop or crash).
            # Without this, the robot holds the last ServoP target unsupervised.
            try:
                self.ros.stop()
            except Exception as stop_err:
                log.warning("ros.stop() during tester shutdown failed: %s", stop_err)
            with self._lock:
                self._status.running = False
                self._close_csv()

    def _current_offset(self) -> List[float]:
        """Return the 6-D offset for this tick, evaluating a pattern if active."""
        with self._lock:
            if self._pattern is not None:
                elapsed = time.time() - self._pattern_start_t
                self._status.pattern_elapsed_s = elapsed
                if self._pattern.finished(elapsed):
                    # Pattern ended — drop to hold mode.
                    self._pattern = None
                    self._status.mode = "idle"
                    self._status.pattern_name = None
                    return list(self._target_offset)
                return list(self._pattern.target_at(elapsed))
            return list(self._target_offset)

    def _refresh_percentiles(self):
        if not self._latencies:
            return
        sorted_l = sorted(self._latencies)
        n = len(sorted_l)
        self._status.p50_latency_ms = sorted_l[n // 2]
        self._status.p95_latency_ms = sorted_l[min(n - 1, int(n * 0.95))]

    # ── CSV logging ─────────────────────────────────────────────
    def _open_csv(self):
        try:
            path = Path(self.log_csv_path)
            path.parent.mkdir(parents=True, exist_ok=True)
            self._csv_file = path.open("w", buffering=1)  # line-buffered
            self._csv_file.write(
                "t,ox,oy,oz,orx,ory,orz,"
                "cx,cy,cz,crx,cry,crz,"
                "ax,ay,az,arx,ary,arz,latency_ms\n"
            )
        except Exception as e:
            log.warning("open csv log failed: %s", e)
            self._csv_file = None
            with self._lock:
                self._status.last_error = f"CSV log failed to open: {e}"

    def _close_csv(self):
        if self._csv_file is not None:
            try:
                self._csv_file.close()
            except Exception:
                pass
            self._csv_file = None

    def _maybe_log(self, commanded: List[float], offset: List[float], latency_ms: float):
        if self._csv_file is None:
            return
        try:
            actual = self.ros.get_cartesian_pose()
        except Exception:
            actual = [0.0] * 6
        row = (
            [time.time()] + offset + commanded + list(actual) + [latency_ms]
        )
        try:
            self._csv_file.write(",".join(f"{v:.4f}" for v in row) + "\n")
        except Exception:
            pass
