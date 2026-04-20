"""Dataclasses and constants for the SpaceMouse pendant."""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional


# 3Dconnexion SpaceMouse Wireless BT (confirmed via `bluetoothctl info`).
VENDOR_ID = 0x256F
PRODUCT_ID = 0xC63A


class DeviceStatus(str, Enum):
    DISCONNECTED = "disconnected"
    CONNECTED = "connected"
    LOST = "lost"
    PERMISSION_DENIED = "permission_denied"


DEFAULT_SETTINGS = {
    "max_velocity_xyz": 80.0,
    "max_velocity_rpy": 30.0,
    "deadband": 0.10,
    "sign_map": [1, 1, 1, 1, 1, 1],
    "max_excursion_xyz": 300.0,
    "max_excursion_rpy": 90.0,
    "idle_auto_disarm_s": 30.0,
    "button_debounce_ms": 150,
    "gripper_force": 20,
    # First-order IIR smoothing applied to each axis value before it
    # becomes a velocity command. smoothed[i] = alpha*raw + (1-alpha)*prev.
    # 1.0 = no filtering (raw); 0.0 = frozen. 0.35 at 50Hz ≈ 45ms time const.
    "axis_lpf_alpha": 0.35,
}


@dataclass
class SpaceMouseSettings:
    """Persisted, live-editable tuning for the reader.

    Validated on construction; callers should catch ValueError when loading
    from untrusted input (e.g., a settings POST).
    """
    max_velocity_xyz: float = 80.0
    max_velocity_rpy: float = 30.0
    deadband: float = 0.10
    sign_map: List[int] = field(default_factory=lambda: [1, 1, 1, 1, 1, 1])
    max_excursion_xyz: float = 300.0
    max_excursion_rpy: float = 90.0
    idle_auto_disarm_s: float = 30.0
    button_debounce_ms: int = 150
    gripper_force: int = 20
    # IIR smoothing on axis values to remove HID micro-jitter before the
    # velocity command is emitted. 1.0 disables (raw); 0.35 at 50Hz is
    # a good default.
    axis_lpf_alpha: float = 0.35

    def __post_init__(self):
        if len(self.sign_map) != 6:
            raise ValueError(f"sign_map must have 6 elements, got {len(self.sign_map)}")
        for v in self.sign_map:
            if v not in (-1, 1):
                raise ValueError(f"sign_map values must be ±1, got {v}")
        if not 0.0 <= self.deadband < 1.0:
            raise ValueError("deadband must be in [0, 1)")
        if self.max_velocity_xyz <= 0 or self.max_velocity_rpy <= 0:
            raise ValueError("max_velocity_* must be > 0")
        if self.max_excursion_xyz <= 0 or self.max_excursion_rpy <= 0:
            raise ValueError("max_excursion_* must be > 0")
        if self.idle_auto_disarm_s <= 0:
            raise ValueError("idle_auto_disarm_s must be > 0")
        if not 0 <= self.gripper_force <= 100:
            raise ValueError("gripper_force must be in [0, 100]")
        if not 0.0 < self.axis_lpf_alpha <= 1.0:
            raise ValueError("axis_lpf_alpha must be in (0, 1]")


@dataclass
class HidState:
    """Snapshot of the pendant state pushed to the browser at ~10 Hz."""
    axes: List[float] = field(default_factory=lambda: [0.0] * 6)
    buttons: List[bool] = field(default_factory=lambda: [False, False])
    device_status: DeviceStatus = DeviceStatus.DISCONNECTED
    armed: bool = False
    battery_pct: Optional[int] = None
    offset: List[float] = field(default_factory=lambda: [0.0] * 6)
    anchor: Optional[List[float]] = None
    last_event_age_ms: float = 0.0
    idle_seconds: float = 0.0
    error: Optional[str] = None

    def to_dict(self) -> dict:
        return {
            "axes": list(self.axes),
            "buttons": list(self.buttons),
            "device_status": self.device_status.value,
            "armed": self.armed,
            "battery_pct": self.battery_pct,
            "offset": list(self.offset),
            "anchor": list(self.anchor) if self.anchor is not None else None,
            "last_event_age_ms": self.last_event_age_ms,
            "idle_seconds": self.idle_seconds,
            "error": self.error,
        }
