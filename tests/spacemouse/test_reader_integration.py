"""Integration tests for SpaceMouseReader tick → ServoTester + idle disarm."""
from __future__ import annotations

import pytest

from dobot_ros.spacemouse.hid_state import DEFAULT_SETTINGS, SpaceMouseSettings
from dobot_ros.spacemouse.reader import SpaceMouseReader


def _reader_with_clock(mock_servo, mock_ros, **overrides):
    settings = SpaceMouseSettings(**{**DEFAULT_SETTINGS, **overrides})
    clock = {"t": 0.0}

    def time_fn():
        return clock["t"]

    def advance(dt):
        clock["t"] += dt

    r = SpaceMouseReader(
        servo_tester=mock_servo, ros_client=mock_ros, settings=settings,
        time_fn=time_fn,
        find_device_fn=lambda: "/dev/input/event21",
        open_device_fn=lambda p: None,
        battery_fn=lambda: 87,
    )
    r.set_axis_absmax([350] * 6)
    return r, advance


def test_tick_does_nothing_when_disarmed(mock_servo, mock_ros):
    r, advance = _reader_with_clock(mock_servo, mock_ros)
    r.on_raw_axis(0, 350)
    advance(0.02)
    r.tick_once(dt=0.02)
    assert mock_servo.target_offsets == []


def test_tick_integrates_offset_when_armed(mock_servo, mock_ros):
    r, advance = _reader_with_clock(mock_servo, mock_ros, max_velocity_xyz=80.0)
    r.arm()
    r.on_raw_axis(0, 350)
    advance(0.02)
    r.tick_once(dt=0.02)
    assert len(mock_servo.target_offsets) == 1
    off = mock_servo.target_offsets[-1]
    assert abs(off[0] - 80.0 * 0.02) < 1e-6
    assert off[1:] == [0.0, 0.0, 0.0, 0.0, 0.0]


def test_tick_accumulates_offset_over_multiple_ticks(mock_servo, mock_ros):
    r, advance = _reader_with_clock(mock_servo, mock_ros, max_velocity_xyz=80.0)
    r.arm()
    r.on_raw_axis(0, 350)
    for _ in range(10):
        advance(0.02)
        r.tick_once(dt=0.02)
    off = mock_servo.target_offsets[-1]
    assert abs(off[0] - 16.0) < 1e-3


def test_tick_respects_deadband(mock_servo, mock_ros):
    r, advance = _reader_with_clock(mock_servo, mock_ros, deadband=0.1)
    r.arm()
    r.on_raw_axis(0, int(0.05 * 350))
    for _ in range(10):
        advance(0.02)
        r.tick_once(dt=0.02)
    off = mock_servo.target_offsets[-1]
    assert off == [0.0] * 6


def test_tick_clamps_offset_to_max_excursion(mock_servo, mock_ros):
    r, advance = _reader_with_clock(
        mock_servo, mock_ros, max_velocity_xyz=1000.0, max_excursion_xyz=50.0)
    r.arm()
    r.on_raw_axis(0, 350)
    for _ in range(20):
        advance(0.02)
        r.tick_once(dt=0.02)
    off = mock_servo.target_offsets[-1]
    assert off[0] == pytest.approx(50.0)


def test_tick_clamps_negative_excursion(mock_servo, mock_ros):
    r, advance = _reader_with_clock(
        mock_servo, mock_ros, max_velocity_xyz=1000.0, max_excursion_xyz=50.0)
    r.arm()
    r.on_raw_axis(0, -350)
    for _ in range(20):
        advance(0.02)
        r.tick_once(dt=0.02)
    off = mock_servo.target_offsets[-1]
    assert off[0] == pytest.approx(-50.0)


def test_idle_auto_disarm_fires_after_timeout(mock_servo, mock_ros):
    r, advance = _reader_with_clock(mock_servo, mock_ros, idle_auto_disarm_s=1.0)
    r.arm()
    for _ in range(60):
        advance(0.02)
        r.tick_once(dt=0.02)
    assert r.snapshot().armed is False
    assert mock_servo.stop_calls == 1


def test_button_press_resets_idle_timer(mock_servo, mock_ros):
    r, advance = _reader_with_clock(mock_servo, mock_ros, idle_auto_disarm_s=1.0)
    r.arm()
    for _ in range(30):
        advance(0.02)
        r.tick_once(dt=0.02)
    r.on_button(1, pressed=True)
    r.on_button(1, pressed=False)
    for _ in range(30):
        advance(0.02)
        r.tick_once(dt=0.02)
    assert r.snapshot().armed is True


def test_axis_motion_resets_idle_timer(mock_servo, mock_ros):
    r, advance = _reader_with_clock(mock_servo, mock_ros, idle_auto_disarm_s=1.0)
    r.arm()
    for _ in range(30):
        advance(0.02)
        r.tick_once(dt=0.02)
    r.on_raw_axis(0, 350)
    for _ in range(5):
        advance(0.02)
        r.tick_once(dt=0.02)
    r.on_raw_axis(0, 0)
    for _ in range(30):
        advance(0.02)
        r.tick_once(dt=0.02)
    assert r.snapshot().armed is True


def test_disconnect_disarms_and_calls_servo_estop(mock_servo, mock_ros):
    r, _ = _reader_with_clock(mock_servo, mock_ros)
    r.arm()
    r.notify_device_lost("read failed")
    assert r.snapshot().armed is False
    assert r.snapshot().device_status.value == "lost"
    assert mock_servo.emergency_stop_calls == 1


def test_snapshot_offset_matches_last_commanded(mock_servo, mock_ros):
    r, advance = _reader_with_clock(mock_servo, mock_ros, max_velocity_xyz=80.0)
    r.arm()
    r.on_raw_axis(0, 350)
    for _ in range(5):
        advance(0.02)
        r.tick_once(dt=0.02)
    s = r.snapshot()
    assert abs(s.offset[0] - 80.0 * 0.02 * 5) < 1e-3
