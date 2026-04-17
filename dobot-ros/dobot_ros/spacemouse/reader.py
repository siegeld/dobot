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
        self._last_button_edge: List[float] = [0.0, 0.0]
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

    # ── Internals ───────────────────────────────────────────────────
    def _safe_battery(self) -> Optional[int]:
        try:
            return self._read_battery()
        except Exception:
            return None
