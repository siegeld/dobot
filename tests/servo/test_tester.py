"""Integration tests for the ServoTester against a mock ROS client."""

from __future__ import annotations

import time

from dobot_ros.servo.tester import ServoTester, ServoConfig
from dobot_ros.servo.patterns import CirclePattern, SineWavePattern
from dobot_ros.vla.safety import SafetyLimits


def _run(tester, duration=0.3):
    tester.start()
    time.sleep(duration)
    tester.stop()


def test_tester_streams_hold_pose(mock_ros):
    """With no jog or pattern, the tester should still stream the anchor pose."""
    tester = ServoTester(mock_ros, ServoConfig(servo_rate_hz=50.0))
    _run(tester, duration=0.3)
    assert len(mock_ros.servo_p_calls) >= 8
    # Every call targets the anchor (equal to the initial pose).
    anchor = tester.status().anchor_pose
    for c in mock_ros.servo_p_calls:
        for i in range(6):
            assert abs(c["pose"][i] - anchor[i]) < 1e-6


def test_tester_applies_jog_offset(mock_ros):
    tester = ServoTester(mock_ros, ServoConfig(servo_rate_hz=50.0))
    tester.start()
    tester.set_target_offset([5.0, 0, 0, 0, 0, 0])
    time.sleep(0.2)
    tester.stop()

    # We should see calls with X offset by ~+5 mm eventually.
    anchor_x = tester.status().anchor_pose[0]
    xs = [c["pose"][0] for c in mock_ros.servo_p_calls[-5:]]
    assert any(abs(x - (anchor_x + 5.0)) < 1e-6 for x in xs)


def test_tester_runs_circle_pattern(mock_ros):
    tester = ServoTester(mock_ros, ServoConfig(servo_rate_hz=60.0))
    tester.start()
    tester.set_pattern(CirclePattern(radius_mm=20.0, period_s=2.0), name="circle")
    time.sleep(0.4)
    tester.stop()

    anchor = tester.status().anchor_pose
    # Across many samples, we should see variance in both X and Y commands.
    xs = [c["pose"][0] - anchor[0] for c in mock_ros.servo_p_calls]
    ys = [c["pose"][1] - anchor[1] for c in mock_ros.servo_p_calls]
    assert max(xs) - min(xs) > 5.0
    assert max(ys) - min(ys) > 5.0


def test_tester_emergency_stop_calls_ros_stop(mock_ros):
    # Track calls to ros.stop() by wrapping the mock.
    calls = []
    orig = getattr(mock_ros, "stop", None)
    mock_ros.stop = lambda: calls.append("stop") or 0

    tester = ServoTester(mock_ros, ServoConfig(servo_rate_hz=30.0))
    tester.start()
    time.sleep(0.1)
    tester.emergency_stop()

    assert "stop" in calls
    assert not tester.is_running()
    if orig is not None:
        mock_ros.stop = orig


def test_tester_workspace_clamp(mock_ros):
    # Tight bounds around the anchor; any offset > 2 mm in X is clamped.
    limits = SafetyLimits(
        x_min=mock_ros._pose[0] - 2.0, x_max=mock_ros._pose[0] + 2.0,
        y_min=-1e6, y_max=1e6, z_min=-1e6, z_max=1e6,
    )
    tester = ServoTester(mock_ros, ServoConfig(servo_rate_hz=50.0), limits=limits)
    tester.start()
    tester.set_target_offset([50.0, 0, 0, 0, 0, 0])  # wildly out of bounds
    time.sleep(0.15)
    tester.stop()

    anchor_x = tester.status().anchor_pose[0]
    for c in mock_ros.servo_p_calls[-3:]:
        # X should be clamped to anchor + 2.0 at most.
        assert c["pose"][0] <= anchor_x + 2.0 + 1e-6
    assert tester.status().clamps > 0


def test_tester_live_config_update(mock_ros):
    tester = ServoTester(mock_ros, ServoConfig(servo_rate_hz=20.0, gain=500.0))
    tester.start()
    tester.update_config(gain=800.0, aheadtime=70.0)
    time.sleep(0.15)
    tester.stop()

    # Config update is reflected in subsequent ServoP calls.
    gains = {c["gain"] for c in mock_ros.servo_p_calls[-3:]}
    assert 800.0 in gains


def test_tester_cannot_start_twice(mock_ros):
    tester = ServoTester(mock_ros, ServoConfig(servo_rate_hz=10.0))
    tester.start()
    # Second start is a no-op; returns current status without error.
    s = tester.start()
    assert s.running
    tester.stop()
