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
    # Reader is rate-control now: nothing should go to the tester when
    # disarmed — neither set_target_velocity nor set_target_offset.
    assert mock_servo.target_velocities == []
    assert mock_servo.target_offsets == []


def test_tick_emits_velocity_when_armed(mock_servo, mock_ros):
    """Full axis push → one velocity command per tick at max_velocity_xyz."""
    r, advance = _reader_with_clock(mock_servo, mock_ros, max_velocity_xyz=80.0)
    r.arm()
    r.on_raw_axis(0, 350)
    advance(0.02)
    r.tick_once(dt=0.02)
    assert len(mock_servo.target_velocities) == 1
    vel = mock_servo.target_velocities[-1]
    assert abs(vel[0] - 80.0) < 1e-6
    assert vel[1:] == [0.0, 0.0, 0.0, 0.0, 0.0]


def test_release_stops_motion_immediately(mock_servo, mock_ros):
    """Axis returns to zero → the very next tick emits zero velocity.

    This is the whole point of rate control: release = stop. No residual
    motion, no catch-up lag.
    """
    r, advance = _reader_with_clock(mock_servo, mock_ros, max_velocity_xyz=80.0)
    r.arm()
    # Push for 5 ticks.
    r.on_raw_axis(0, 350)
    for _ in range(5):
        advance(0.02)
        r.tick_once(dt=0.02)
    # Release.
    r.on_raw_axis(0, 0)
    advance(0.02)
    r.tick_once(dt=0.02)
    # Very next emitted velocity must be zero on every axis.
    assert mock_servo.target_velocities[-1] == [0.0] * 6


def test_tick_respects_deadband(mock_servo, mock_ros):
    r, advance = _reader_with_clock(mock_servo, mock_ros, deadband=0.1)
    r.arm()
    r.on_raw_axis(0, int(0.05 * 350))  # inside deadband (5% of full scale)
    for _ in range(10):
        advance(0.02)
        r.tick_once(dt=0.02)
    # Every emitted velocity should be all zeros.
    for vel in mock_servo.target_velocities:
        assert vel == [0.0] * 6


def test_excursion_limit_zeros_outward_velocity(mock_servo, mock_ros):
    """Once the mock's commanded offset hits max_excursion_xyz in +X,
    the reader must stop pushing +X (v[0] = 0). Pulling back is still
    allowed (v[0] < 0 would pass through)."""
    r, advance = _reader_with_clock(
        mock_servo, mock_ros, max_velocity_xyz=1000.0, max_excursion_xyz=50.0)
    r.arm()
    r.on_raw_axis(0, 350)
    for _ in range(20):
        advance(0.02)
        r.tick_once(dt=0.02)
    # The mock integrates velocity × dt; commanded offset should be clamped
    # by the reader's excursion guard once it reaches 50 mm.
    assert mock_servo._commanded_offset[0] <= 50.0 + 1e-3
    # And the LAST few emitted velocities (when already at excursion) are
    # zero on X because the reader zeros outward velocity there.
    assert mock_servo.target_velocities[-1][0] == 0.0


def test_excursion_limit_negative_direction(mock_servo, mock_ros):
    r, advance = _reader_with_clock(
        mock_servo, mock_ros, max_velocity_xyz=1000.0, max_excursion_xyz=50.0)
    r.arm()
    r.on_raw_axis(0, -350)
    for _ in range(20):
        advance(0.02)
        r.tick_once(dt=0.02)
    assert mock_servo._commanded_offset[0] >= -50.0 - 1e-3
    assert mock_servo.target_velocities[-1][0] == 0.0


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


import time as real_time


def test_run_spins_both_threads_and_stops_cleanly(mock_servo, mock_ros, fake_evdev_with_spacemouse, monkeypatch):
    from dobot_ros.spacemouse import reader as reader_mod

    monkeypatch.setattr(reader_mod, "_evdev", fake_evdev_with_spacemouse)
    monkeypatch.setattr(reader_mod, "_ecodes", fake_evdev_with_spacemouse.ecodes)

    r = reader_mod.SpaceMouseReader(
        servo_tester=mock_servo, ros_client=mock_ros,
        settings=SpaceMouseSettings(**DEFAULT_SETTINGS),
        time_fn=real_time.monotonic,
        find_device_fn=lambda: "/dev/input/event21",
        open_device_fn=lambda path: fake_evdev_with_spacemouse.InputDevice(path),
        battery_fn=lambda: None,
    )
    r.start_threads(tick_hz=100.0)
    try:
        r.arm()
        real_time.sleep(0.15)
        r.disarm()
    finally:
        r.stop_threads(timeout=1.0)

    assert mock_servo.start_calls == 1
    assert mock_servo.stop_calls == 1
