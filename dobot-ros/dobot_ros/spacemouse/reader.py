"""SpaceMouseReader — bridges a 3Dconnexion SpaceMouse Wireless BT to ServoTester.

Additive: calls ServoTester.set_target_offset() (already existing) to stream
live jog offsets. Two daemon threads:
  1. evdev read loop (blocks on device.read_loop()), decodes axis/button events.
  2. 50 Hz integration tick: deadband + scale + integrate + clamp, forwards
     to ServoTester; debounces buttons → gripper.

See docs/superpowers/specs/2026-04-17-spacemouse-pendant-design.md.
"""
from __future__ import annotations

import copy
import logging
import threading
from typing import Callable, List, Optional, Tuple

from dobot_ros.spacemouse.hid_state import (
    DeviceStatus, HidState, SpaceMouseSettings,
)

try:
    import evdev as _evdev
    from evdev import ecodes as _ecodes
except ImportError:  # pragma: no cover - test environments stub these
    _evdev = None
    _ecodes = None

log = logging.getLogger(__name__)


class SpaceMouseReader:
    """Owner of the SpaceMouse device + integration loop."""

    def __init__(
        self,
        servo_tester,
        ros_client,
        settings: SpaceMouseSettings,
        time_fn: Callable[[], float],
        find_device_fn: Callable[[], Optional[str]],
        open_device_fn: Callable[[str], object],
        battery_fn: Callable[[], Optional[int]],
    ):
        self._servo = servo_tester
        self._ros = ros_client
        self._settings = settings
        self._time = time_fn
        self._find_device = find_device_fn
        self._open_device = open_device_fn
        self._read_battery = battery_fn

        self._lock = threading.RLock()
        self._state = HidState()
        self._offset: List[float] = [0.0] * 6
        self._last_raw: List[float] = [0.0] * 6
        self._buttons: List[bool] = [False, False]
        # -inf so the first edge never debounces against a pretend "previous" edge.
        self._last_button_edge: List[float] = [float("-inf"), float("-inf")]
        self._last_nonidle_t: float = 0.0
        self._device_path: Optional[str] = None
        self._armed: bool = False
        self._abs_max: Optional[List[int]] = None

    # ── Public API ──────────────────────────────────────────────────
    def snapshot(self) -> HidState:
        with self._lock:
            return copy.deepcopy(self._state)

    def update_settings(self, settings: SpaceMouseSettings):
        with self._lock:
            self._settings = settings

    def arm(self) -> Tuple[bool, Optional[str]]:
        """Check gates and arm the reader. Returns (ok, reason)."""
        with self._lock:
            if self._armed:
                return True, None
            path = self._find_device()
            if path is None:
                self._state.device_status = DeviceStatus.DISCONNECTED
                self._state.error = "SpaceMouse not connected"
                return False, "SpaceMouse not connected — check Bluetooth"
            self._device_path = path
            try:
                status = self._servo.start()
                anchor = list(getattr(status, "anchor_pose", []) or [])
            except Exception as e:
                return False, f"failed to start servo stream: {e}"
            self._offset = [0.0] * 6
            self._last_nonidle_t = self._time()
            self._state = HidState(
                armed=True,
                device_status=DeviceStatus.CONNECTED,
                anchor=anchor,
                battery_pct=self._safe_battery(),
            )
            self._armed = True
            log.info("SpaceMouse armed; anchor=%s device=%s", anchor, path)
        return True, None

    def disarm(self):
        """Graceful disarm: stop the servo stream, hold pose."""
        with self._lock:
            if not self._armed:
                return
            self._armed = False
            self._offset = [0.0] * 6
            try:
                self._servo.stop()
            except Exception as e:
                log.warning("servo.stop() during disarm failed: %s", e)
            self._state.armed = False
            self._state.offset = [0.0] * 6
            log.info("SpaceMouse disarmed")

    def emergency_stop(self):
        """E-stop: halt the servo stream AND call ros.stop()."""
        with self._lock:
            self._armed = False
            self._offset = [0.0] * 6
            try:
                self._servo.emergency_stop()
            except Exception as e:
                log.warning("servo.emergency_stop() failed: %s", e)
            self._state.armed = False
            self._state.offset = [0.0] * 6
            self._state.error = "emergency stop"
            log.warning("SpaceMouse E-stop")

    # ── Event ingestion (driven by evdev thread OR by tests) ────────
    def set_axis_absmax(self, absmax_per_axis: List[int]):
        """Record the raw range for normalization. Called once after device open."""
        if len(absmax_per_axis) != 6:
            raise ValueError("absmax_per_axis must have 6 entries")
        with self._lock:
            self._abs_max = list(absmax_per_axis)

    def on_raw_axis(self, axis_index: int, raw_value: int):
        """Ingest one axis event. axis_index in [0..5]: 0..2 = TX/TY/TZ, 3..5 = RX/RY/RZ."""
        if not 0 <= axis_index <= 5:
            return
        with self._lock:
            amax = self._abs_max[axis_index] if self._abs_max else 350
            norm = raw_value / float(amax) if amax else 0.0
            if norm > 1.0:
                norm = 1.0
            elif norm < -1.0:
                norm = -1.0
            self._last_raw[axis_index] = norm
            self._state.axes = list(self._last_raw)

    def on_button(self, button_index: int, pressed: bool):
        """Ingest one button event. button_index in {0, 1}."""
        if button_index not in (0, 1):
            return
        with self._lock:
            was = self._buttons[button_index]
            self._buttons[button_index] = bool(pressed)
            self._state.buttons = list(self._buttons)
            if pressed and not was:
                now = self._time()
                debounce_s = self._settings.button_debounce_ms / 1000.0
                if now - self._last_button_edge[button_index] < debounce_s:
                    return
                self._last_button_edge[button_index] = now
                self._last_nonidle_t = now
                if self._armed:
                    self._fire_gripper(button_index)

    def effective_velocity(self) -> List[float]:
        """Compute the current 6-D velocity command (with deadband + sign map).

        Units: mm/s for indices 0..2, deg/s for 3..5.
        """
        with self._lock:
            s = self._settings
            v = [0.0] * 6
            for i in range(6):
                a = self._last_raw[i]
                if abs(a) < s.deadband:
                    continue
                scale = s.max_velocity_xyz if i < 3 else s.max_velocity_rpy
                v[i] = a * scale * s.sign_map[i]
            return v

    def _fire_gripper(self, button_index: int):
        position = 0 if button_index == 0 else 1000
        force = self._settings.gripper_force
        try:
            self._ros.gripper_move(position, force=force, speed=50, wait=False)
        except Exception as e:
            log.warning("gripper_move(%d) failed: %s", position, e)

    # ── Tick loop ───────────────────────────────────────────────────
    def tick_once(self, dt: float):
        """One iteration of the 50 Hz integration loop. Tests drive this directly;
        in production the threaded wrapper calls it every `dt` seconds."""
        with self._lock:
            s = self._settings
            v = [0.0] * 6
            any_active = False
            for i in range(6):
                a = self._last_raw[i]
                if abs(a) >= s.deadband:
                    scale = s.max_velocity_xyz if i < 3 else s.max_velocity_rpy
                    v[i] = a * scale * s.sign_map[i]
                    any_active = True

            new_offset = list(self._offset)
            for i in range(6):
                new_offset[i] += v[i] * dt

            mx = s.max_excursion_xyz
            mr = s.max_excursion_rpy
            for i in range(3):
                if new_offset[i] > mx:
                    new_offset[i] = mx
                elif new_offset[i] < -mx:
                    new_offset[i] = -mx
            for i in range(3, 6):
                if new_offset[i] > mr:
                    new_offset[i] = mr
                elif new_offset[i] < -mr:
                    new_offset[i] = -mr

            self._offset = new_offset
            self._state.offset = list(new_offset)

            now = self._time()
            if any_active:
                self._last_nonidle_t = now
            idle_s = now - self._last_nonidle_t
            self._state.idle_seconds = idle_s

            armed = self._armed
            idle_timeout = self._settings.idle_auto_disarm_s
            offset_snapshot = list(self._offset)

        if armed:
            try:
                self._servo.set_target_offset(offset_snapshot)
            except Exception as e:
                log.warning("servo.set_target_offset failed: %s", e)

            if idle_s >= idle_timeout:
                log.info("SpaceMouse idle %.1fs → auto-disarm", idle_s)
                self.disarm()

    def notify_device_lost(self, reason: str):
        """Called by the evdev thread (or tests) when the device goes away."""
        with self._lock:
            was_armed = self._armed
            self._armed = False
            self._state.armed = False
            self._state.device_status = DeviceStatus.LOST
            self._state.error = f"device lost: {reason}"
            self._device_path = None
        if was_armed:
            try:
                self._servo.emergency_stop()
            except Exception as e:
                log.warning("emergency_stop on device loss failed: %s", e)
        log.warning("SpaceMouse device lost: %s", reason)

    # ── Threads ─────────────────────────────────────────────────────
    def start_threads(self, tick_hz: float = 50.0):
        """Start the evdev read thread and the integration tick thread."""
        if getattr(self, "_stop_event", None) is not None:
            return
        self._stop_event = threading.Event()
        self._tick_hz = tick_hz
        self._open_device_obj = None
        self._tick_thread = threading.Thread(
            target=self._tick_loop, name="spacemouse-tick", daemon=True)
        self._read_thread = threading.Thread(
            target=self._read_loop, name="spacemouse-read", daemon=True)
        self._tick_thread.start()
        self._read_thread.start()

    def stop_threads(self, timeout: float = 2.0):
        """Stop both threads. Idempotent."""
        ev = getattr(self, "_stop_event", None)
        if ev is None:
            return
        ev.set()
        try:
            self.disarm()
        except Exception:
            pass
        try:
            dev = getattr(self, "_open_device_obj", None)
            if dev is not None:
                try:
                    dev.close()
                except Exception:
                    pass
        finally:
            self._tick_thread.join(timeout=timeout)
            self._read_thread.join(timeout=timeout)
            self._stop_event = None
            self._tick_thread = None
            self._read_thread = None
            self._open_device_obj = None

    # Background loops ------------------------------------------------

    def _tick_loop(self):
        period = 1.0 / max(1.0, self._tick_hz)
        last = self._time()
        while not self._stop_event.is_set():
            now = self._time()
            dt = max(1e-4, now - last)
            last = now
            try:
                self.tick_once(dt=dt)
            except Exception:
                log.exception("spacemouse tick failed")
            self._stop_event.wait(timeout=period)

    def _read_loop(self):
        while not self._stop_event.is_set():
            path = self._find_device()
            if path is None:
                self._stop_event.wait(timeout=1.0)
                continue
            try:
                dev = self._open_device(path)
            except PermissionError:
                with self._lock:
                    self._state.device_status = DeviceStatus.PERMISSION_DENIED
                    self._state.error = "permission denied on /dev/input (check input group)"
                return
            except Exception as e:
                log.warning("open_device(%s) failed: %s", path, e)
                self._stop_event.wait(timeout=1.0)
                continue
            self._open_device_obj = dev
            self._discover_absinfo(dev)
            with self._lock:
                self._state.device_status = DeviceStatus.CONNECTED
                self._state.error = None
                self._device_path = path
            try:
                self._pump_events(dev)
            except OSError as e:
                self.notify_device_lost(str(e))
            except Exception as e:
                log.exception("read loop crashed: %s", e)
                self.notify_device_lost(str(e))
            finally:
                try:
                    dev.close()
                except Exception:
                    pass

    def _pump_events(self, dev):
        if _ecodes is None:
            return
        axis_code_map = {
            _ecodes.ABS_X: 0, _ecodes.ABS_Y: 1, _ecodes.ABS_Z: 2,
            _ecodes.ABS_RX: 3, _ecodes.ABS_RY: 4, _ecodes.ABS_RZ: 5,
        }
        button_code_map = {_ecodes.BTN_0: 0, _ecodes.BTN_1: 1}
        ev_abs = _ecodes.EV_ABS
        ev_key = _ecodes.EV_KEY
        for ev in dev.read_loop():
            if self._stop_event.is_set():
                return
            etype = ev.type
            if etype == ev_abs:
                idx = axis_code_map.get(ev.code)
                if idx is not None:
                    self.on_raw_axis(idx, ev.value)
            elif etype == ev_key:
                idx = button_code_map.get(ev.code)
                if idx is not None:
                    self.on_button(idx, pressed=bool(ev.value))

    def _discover_absinfo(self, dev):
        if _ecodes is None:
            if not self._abs_max:
                self.set_axis_absmax([350] * 6)
            return
        axes = [_ecodes.ABS_X, _ecodes.ABS_Y, _ecodes.ABS_Z,
                _ecodes.ABS_RX, _ecodes.ABS_RY, _ecodes.ABS_RZ]
        maxes: List[int] = []
        for code in axes:
            try:
                info = dev.absinfo(code)
                maxes.append(int(info.max) if info and info.max else 350)
            except Exception:
                maxes.append(350)
        self.set_axis_absmax(maxes)

    # ── Internals ───────────────────────────────────────────────────
    def _safe_battery(self) -> Optional[int]:
        try:
            return self._read_battery()
        except Exception:
            return None
