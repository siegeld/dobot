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


def test_factory_defers_servo_tester_until_arm(mock_servo, mock_ros):
    """Reader accepts a zero-arg factory; it shouldn't invoke it until arm()."""
    calls = {"n": 0}

    def factory():
        calls["n"] += 1
        return mock_servo

    reader = SpaceMouseReader(
        servo_tester=factory, ros_client=mock_ros,
        settings=SpaceMouseSettings(**DEFAULT_SETTINGS),
        time_fn=lambda: 0.0,
        find_device_fn=lambda: "/dev/input/event21",
        open_device_fn=lambda p: None,
        battery_fn=lambda: None,
    )
    assert calls["n"] == 0        # factory not called during construction
    assert reader.snapshot().armed is False
    reader.arm()
    assert calls["n"] == 1


def test_factory_failure_surfaces_as_arm_error(mock_servo, mock_ros):
    def bad_factory():
        raise RuntimeError("boom")

    reader = SpaceMouseReader(
        servo_tester=bad_factory, ros_client=mock_ros,
        settings=SpaceMouseSettings(**DEFAULT_SETTINGS),
        time_fn=lambda: 0.0,
        find_device_fn=lambda: "/dev/input/event21",
        open_device_fn=lambda p: None,
        battery_fn=lambda: None,
    )
    ok, reason = reader.arm()
    assert ok is False
    assert "boom" in reason


# ── Decode / deadband / sign-map tests ──────────────────────────────


def test_normalize_by_absinfo_max(mock_servo, mock_ros):
    reader, _ = _make_reader(mock_servo, mock_ros)
    reader.set_axis_absmax([350] * 6)
    reader.on_raw_axis(axis_index=0, raw_value=175)
    s = reader.snapshot()
    assert abs(s.axes[0] - 0.5) < 1e-6


def test_normalize_clips_above_one(mock_servo, mock_ros):
    reader, _ = _make_reader(mock_servo, mock_ros)
    reader.set_axis_absmax([350] * 6)
    reader.on_raw_axis(axis_index=2, raw_value=10_000)
    s = reader.snapshot()
    assert s.axes[2] == 1.0


def test_normalize_clips_below_minus_one(mock_servo, mock_ros):
    reader, _ = _make_reader(mock_servo, mock_ros)
    reader.set_axis_absmax([350] * 6)
    reader.on_raw_axis(axis_index=2, raw_value=-10_000)
    s = reader.snapshot()
    assert s.axes[2] == -1.0


def test_deadband_zeroes_small_deflections(mock_servo, mock_ros):
    reader, _ = _make_reader(mock_servo, mock_ros, deadband=0.10)
    reader.set_axis_absmax([350] * 6)
    reader.on_raw_axis(axis_index=0, raw_value=int(0.05 * 350))
    assert reader.effective_velocity()[0] == 0.0
    reader.on_raw_axis(axis_index=0, raw_value=int(0.20 * 350))
    assert reader.effective_velocity()[0] > 0.0


def test_sign_map_flips_velocity(mock_servo, mock_ros):
    reader, _ = _make_reader(mock_servo, mock_ros, sign_map=[-1, 1, 1, 1, 1, 1])
    reader.set_axis_absmax([350] * 6)
    reader.on_raw_axis(axis_index=0, raw_value=350)
    v = reader.effective_velocity()
    assert v[0] < 0.0


def test_velocity_uses_max_velocity_from_settings(mock_servo, mock_ros):
    reader, _ = _make_reader(mock_servo, mock_ros, max_velocity_xyz=80.0, max_velocity_rpy=30.0)
    reader.set_axis_absmax([350] * 6)
    reader.on_raw_axis(axis_index=0, raw_value=350)
    reader.on_raw_axis(axis_index=5, raw_value=350)
    v = reader.effective_velocity()
    assert abs(v[0] - 80.0) < 1e-6
    assert abs(v[5] - 30.0) < 1e-6


def test_button_press_edge_armed_triggers_gripper(mock_servo, mock_ros):
    reader, advance = _make_reader(mock_servo, mock_ros)
    reader.arm()
    reader.on_button(0, pressed=True)
    assert len(mock_ros.gripper_moves) == 1
    assert mock_ros.gripper_moves[0]["position"] == 0


def test_button_press_edge_disarmed_is_echoed_but_no_action(mock_servo, mock_ros):
    reader, _ = _make_reader(mock_servo, mock_ros)
    reader.on_button(0, pressed=True)
    assert mock_ros.gripper_moves == []
    s = reader.snapshot()
    assert s.buttons[0] is True


def test_button_debounce_drops_rapid_repeats(mock_servo, mock_ros):
    reader, advance = _make_reader(mock_servo, mock_ros, button_debounce_ms=150)
    reader.arm()
    reader.on_button(0, pressed=True)
    advance(0.050)
    reader.on_button(0, pressed=False)
    reader.on_button(0, pressed=True)
    assert len(mock_ros.gripper_moves) == 1
    advance(0.200)
    reader.on_button(0, pressed=False)
    reader.on_button(0, pressed=True)
    assert len(mock_ros.gripper_moves) == 2


def test_button_open_maps_to_1000(mock_servo, mock_ros):
    reader, _ = _make_reader(mock_servo, mock_ros)
    reader.arm()
    reader.on_button(1, pressed=True)
    assert mock_ros.gripper_moves[-1]["position"] == 1000


def test_button_close_maps_to_0(mock_servo, mock_ros):
    reader, _ = _make_reader(mock_servo, mock_ros)
    reader.arm()
    reader.on_button(0, pressed=True)
    assert mock_ros.gripper_moves[-1]["position"] == 0
