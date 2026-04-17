"""Unit tests for spacemouse dataclasses and constants."""
from __future__ import annotations

from dobot_ros.spacemouse.hid_state import (
    HidState, SpaceMouseSettings, DeviceStatus,
    DEFAULT_SETTINGS, VENDOR_ID, PRODUCT_ID,
)


def test_vendor_product_match_spacemouse_wireless_bt():
    assert VENDOR_ID == 0x256F
    assert PRODUCT_ID == 0xC63A


def test_default_settings_are_safe():
    s = SpaceMouseSettings(**DEFAULT_SETTINGS)
    assert s.max_velocity_xyz == 80.0
    assert s.max_velocity_rpy == 30.0
    assert s.deadband == 0.10
    assert s.sign_map == [1, 1, 1, 1, 1, 1]
    assert s.max_excursion_xyz == 300.0
    assert s.max_excursion_rpy == 90.0
    assert s.idle_auto_disarm_s == 30.0
    assert s.button_debounce_ms == 150
    assert s.gripper_force == 20


def test_hid_state_defaults_are_neutral():
    st = HidState()
    assert st.axes == [0.0] * 6
    assert st.buttons == [False, False]
    assert st.device_status == DeviceStatus.DISCONNECTED
    assert st.armed is False
    assert st.battery_pct is None


def test_hid_state_to_dict_roundtrips():
    st = HidState(
        axes=[0.1, -0.2, 0.3, 0.0, 0.0, 0.0],
        buttons=[True, False],
        device_status=DeviceStatus.CONNECTED,
        armed=True,
        battery_pct=87,
        offset=[5.0, -1.0, 0.0, 0.0, 0.0, 0.0],
        anchor=[400.0, 0.0, 300.0, 180.0, 0.0, 0.0],
        last_event_age_ms=12.5,
    )
    d = st.to_dict()
    assert d["axes"] == [0.1, -0.2, 0.3, 0.0, 0.0, 0.0]
    assert d["buttons"] == [True, False]
    assert d["device_status"] == "connected"
    assert d["armed"] is True
    assert d["battery_pct"] == 87


def test_signmap_validation_length():
    import pytest
    with pytest.raises(ValueError):
        SpaceMouseSettings(**{**DEFAULT_SETTINGS, "sign_map": [1, 1, 1]})


def test_signmap_validation_values():
    import pytest
    with pytest.raises(ValueError):
        SpaceMouseSettings(**{**DEFAULT_SETTINGS, "sign_map": [1, 1, 1, 1, 1, 2]})
