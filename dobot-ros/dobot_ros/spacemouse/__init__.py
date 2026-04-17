"""SpaceMouse pendant module.

Additive: this module provides a Bluetooth SpaceMouse Wireless BT jogging
pendant. It does not modify any existing motion path (MovJ/MovL/jog/pick
remain fully functional). See docs/superpowers/specs/2026-04-17-spacemouse-pendant-design.md.
"""

from dobot_ros.spacemouse.hid_state import (
    DEFAULT_SETTINGS,
    DeviceStatus,
    HidState,
    PRODUCT_ID,
    SpaceMouseSettings,
    VENDOR_ID,
)

__all__ = [
    "DEFAULT_SETTINGS",
    "DeviceStatus",
    "HidState",
    "PRODUCT_ID",
    "SpaceMouseSettings",
    "VENDOR_ID",
]
