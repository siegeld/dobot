"""Unit tests for SpaceMouseReader lifecycle and axis decode."""
from __future__ import annotations

from dobot_ros.spacemouse.hid_state import (
    DEFAULT_SETTINGS, DeviceStatus, SpaceMouseSettings,
)
from dobot_ros.spacemouse.reader import SpaceMouseReader


def _make_reader(mock_servo, mock_ros, **overrides):
    """Shared constructor used across reader tests."""
    settings = SpaceMouseSettings(**{**DEFAULT_SETTINGS, **overrides})
    clock = {"t": 0.0}

    def time_fn():
        return clock["t"]

    def advance(dt):
        clock["t"] += dt

    reader = SpaceMouseReader(
        servo_tester=mock_servo,
        ros_client=mock_ros,
        settings=settings,
        time_fn=time_fn,
        find_device_fn=lambda: "/dev/input/event21",
        open_device_fn=lambda path: None,
        battery_fn=lambda: None,
    )
    return reader, advance


def test_reader_starts_disarmed_and_disconnected(mock_servo, mock_ros):
    reader, _ = _make_reader(mock_servo, mock_ros)
    state = reader.snapshot()
    assert state.armed is False
    assert state.device_status == DeviceStatus.DISCONNECTED
    assert state.offset == [0.0] * 6


def test_arm_requires_device(mock_servo, mock_ros):
    settings = SpaceMouseSettings(**DEFAULT_SETTINGS)
    reader = SpaceMouseReader(
        servo_tester=mock_servo, ros_client=mock_ros, settings=settings,
        time_fn=lambda: 0.0,
        find_device_fn=lambda: None,
        open_device_fn=lambda p: None,
        battery_fn=lambda: None,
    )
    ok, reason = reader.arm()
    assert ok is False
    assert "not found" in reason.lower() or "not connected" in reason.lower()
    assert mock_servo.start_calls == 0


def test_arm_starts_servo_and_records_anchor(mock_servo, mock_ros):
    reader, _ = _make_reader(mock_servo, mock_ros)
    ok, reason = reader.arm()
    assert ok is True, reason
    assert mock_servo.start_calls == 1
    state = reader.snapshot()
    assert state.armed is True
    assert state.anchor is not None


def test_disarm_stops_servo_and_resets_offset(mock_servo, mock_ros):
    reader, _ = _make_reader(mock_servo, mock_ros)
    reader.arm()
    reader.disarm()
    assert mock_servo.stop_calls == 1
    state = reader.snapshot()
    assert state.armed is False
    assert state.offset == [0.0] * 6


def test_double_arm_is_noop(mock_servo, mock_ros):
    reader, _ = _make_reader(mock_servo, mock_ros)
    reader.arm()
    reader.arm()
    assert mock_servo.start_calls == 1


def test_emergency_stop_uses_tester_estop(mock_servo, mock_ros):
    reader, _ = _make_reader(mock_servo, mock_ros)
    reader.arm()
    reader.emergency_stop()
    assert mock_servo.emergency_stop_calls == 1
    state = reader.snapshot()
    assert state.armed is False
