"""Verify the ServoTester's rate-control input path (set_target_velocity).

The tester has two input modes. Position mode (``set_target_offset`` or a
``set_pattern``) advances the commanded offset toward a target position at
the velocity cap. Rate mode (``set_target_velocity``) integrates a caller-
supplied velocity directly. These tests cover the rate-mode path — the
critical property is that a zero velocity stops motion immediately with
no residual "catch-up" to a stale target.
"""

from __future__ import annotations

import math
import time

from dobot_ros.servo.tester import ServoTester, ServoConfig


def test_velocity_integrated_into_commanded_offset(mock_ros):
    """Constant velocity × time → expected total displacement."""
    tester = ServoTester(
        mock_ros,
        ServoConfig(servo_rate_hz=50.0, max_velocity_xyz=200.0, max_velocity_rpy=360.0),
    )
    tester.start()
    tester.set_target_velocity([50.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # 50 mm/s in +X
    time.sleep(0.4)
    tester.stop()
    anchor_x = tester.status().anchor_pose[0]
    last_x = mock_ros.servo_p_calls[-1]["pose"][0]
    # Over ~0.4s at 50 mm/s we expect ~20 mm displacement. Generous slack
    # for loop-entry jitter and end-tick timing.
    assert 15.0 < (last_x - anchor_x) < 25.0


def test_zero_velocity_stops_immediately(mock_ros):
    """After setting velocity=0, subsequent ticks must not advance offset."""
    tester = ServoTester(
        mock_ros,
        ServoConfig(servo_rate_hz=50.0, max_velocity_xyz=200.0, max_velocity_rpy=360.0),
    )
    tester.start()
    tester.set_target_velocity([60.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    time.sleep(0.2)
    # Freeze motion.
    tester.set_target_velocity([0.0] * 6)
    # Take a snapshot just after the stop command.
    frozen_x = mock_ros.servo_p_calls[-1]["pose"][0]
    time.sleep(0.2)
    tester.stop()
    # After zero-velocity, X should not advance by more than a small tick.
    post = [c["pose"][0] for c in mock_ros.servo_p_calls[-5:]]
    max_drift = max(abs(x - frozen_x) for x in post)
    assert max_drift < 2.0, f"drifted {max_drift:.2f}mm after zero velocity"


def test_velocity_itself_is_capped_by_config(mock_ros):
    """A velocity larger than max_velocity_xyz is clamped before integration."""
    rate = 50.0
    cap = 40.0  # mm/s — tight
    tester = ServoTester(
        mock_ros,
        ServoConfig(servo_rate_hz=rate, max_velocity_xyz=cap, max_velocity_rpy=360.0),
    )
    tester.start()
    # Ask for 500 mm/s in +X — should be clamped to 40 mm/s.
    tester.set_target_velocity([500.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    time.sleep(0.25)
    tester.stop()
    anchor_x = tester.status().anchor_pose[0]
    # Maximum possible X displacement at the cap over 0.25s is 10 mm.
    # Tolerate small overshoot for timing jitter.
    last_x = mock_ros.servo_p_calls[-1]["pose"][0]
    assert (last_x - anchor_x) <= 12.0


def test_switch_from_velocity_to_position_no_jump(mock_ros):
    """After a velocity session, switching to position mode must not snap
    the robot back to anchor — target_offset is synced to the commanded
    position on each velocity tick, so mode change is smooth."""
    tester = ServoTester(
        mock_ros,
        ServoConfig(servo_rate_hz=50.0, max_velocity_xyz=200.0, max_velocity_rpy=360.0),
    )
    tester.start()
    tester.set_target_velocity([50.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    time.sleep(0.2)
    pre_switch_x = mock_ros.servo_p_calls[-1]["pose"][0]
    # Switch to position mode with the SAME offset the velocity path has
    # drifted to. The mode change alone (without a new target) shouldn't
    # move the robot; check by giving it the current offset as target.
    current_offset = tester.status().last_target_offset
    tester.set_target_offset(list(current_offset))
    time.sleep(0.1)
    tester.stop()
    post_switch_x = mock_ros.servo_p_calls[-1]["pose"][0]
    # Robot may have advanced a tiny bit due to in-flight cap, but it
    # should not have JUMPED backward to anchor.
    assert post_switch_x >= pre_switch_x - 0.5


def test_pattern_clears_velocity_mode(mock_ros):
    """Starting a pattern must clear any prior velocity command."""
    from dobot_ros.servo.patterns import CirclePattern
    tester = ServoTester(
        mock_ros,
        ServoConfig(servo_rate_hz=50.0, max_velocity_xyz=200.0, max_velocity_rpy=360.0),
    )
    tester.start()
    tester.set_target_velocity([50.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    time.sleep(0.1)
    # Now start a pattern — should take over, no residual linear velocity.
    tester.set_pattern(CirclePattern(radius_mm=20.0, period_s=2.0), name="circle")
    time.sleep(0.2)
    tester.stop()
    # Mode should be "pattern" at stop-time.
    # (We can't check status().mode directly after stop since it's set to
    # "idle" — but confirm the velocity path is not still integrating by
    # observing that after pattern start, poses no longer march linearly.)
    # Pattern positions vary with sin/cos, so the last few X values should
    # NOT be monotonically increasing the way pure velocity integration
    # would give us.
    xs = [c["pose"][0] for c in mock_ros.servo_p_calls[-6:]]
    deltas = [xs[i+1] - xs[i] for i in range(len(xs) - 1)]
    # At least one delta should be non-positive if a circle pattern is
    # running (X oscillates).
    assert any(d <= 0.01 for d in deltas), f"xs looks like linear drive: {xs}"
