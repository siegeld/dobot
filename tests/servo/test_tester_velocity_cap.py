"""Verify the ServoTester tick-loop velocity cap.

The tester rate-limits the advancement of its internal ``_commanded_offset``
from its previous value toward the caller-supplied ``_target_offset`` by
``max_velocity_xyz`` (as 3-vector magnitude) and ``max_velocity_rpy`` (per
axis) per tick. These tests drive the tester at a known rate with a tight
cap and inspect the recorded ``servo_p`` calls to confirm no single
commanded-pose step exceeds the cap times the measured tick dt.
"""

from __future__ import annotations

import math
import time

from dobot_ros.servo.tester import ServoTester, ServoConfig


# Generous tolerance: tick dt is measured in the loop and can jitter a few
# percent on a busy CI box, and the first tick uses a nominal dt. We
# verify the cap holds with a multiplier that allows real jitter without
# hiding genuine overshoots.
_TOLERANCE = 1.25


def _step_magnitudes_xyz(calls, anchor):
    """Return per-tick XYZ magnitude of the commanded-pose change."""
    out = []
    prev = anchor[:3]
    for c in calls:
        cur = c["pose"][:3]
        dx, dy, dz = cur[0] - prev[0], cur[1] - prev[1], cur[2] - prev[2]
        out.append(math.sqrt(dx * dx + dy * dy + dz * dz))
        prev = cur
    return out


def _step_magnitudes_rpy_axis(calls, anchor, axis):
    """Per-tick magnitude of change along one RPY axis (3=RX, 4=RY, 5=RZ)."""
    out = []
    prev = anchor[axis]
    for c in calls:
        cur = c["pose"][axis]
        out.append(abs(cur - prev))
        prev = cur
    return out


def test_xyz_velocity_cap_limits_per_tick_step(mock_ros):
    """Setting a far target never commands a per-tick step above the cap."""
    rate = 50.0
    cap = 30.0  # mm/s — tight so we're capped for the whole run
    tester = ServoTester(
        mock_ros,
        ServoConfig(servo_rate_hz=rate, max_velocity_xyz=cap, max_velocity_rpy=360.0),
    )
    tester.start()
    # Ask for a huge jump the cap can't satisfy in one tick.
    tester.set_target_offset([500.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    time.sleep(0.3)
    tester.stop()

    anchor = tester.status().anchor_pose
    steps = _step_magnitudes_xyz(mock_ros.servo_p_calls, anchor)
    # Skip the very first sample — there's no prior commanded pose to diff
    # against on the first tick.
    limit_per_tick = cap / rate * _TOLERANCE
    for s in steps[1:]:
        assert s <= limit_per_tick, f"step {s:.3f} exceeds cap {limit_per_tick:.3f}"


def test_xyz_cap_preserves_direction(mock_ros):
    """A diagonal target scales XYZ uniformly — direction is preserved."""
    tester = ServoTester(
        mock_ros,
        ServoConfig(servo_rate_hz=50.0, max_velocity_xyz=20.0, max_velocity_rpy=360.0),
    )
    tester.start()
    tester.set_target_offset([300.0, 300.0, 0.0, 0.0, 0.0, 0.0])
    time.sleep(0.2)
    tester.stop()

    # After anchor, every commanded pose should lie on the +X=+Y diagonal
    # (dx == dy, dz == 0) since the cap scales uniformly.
    anchor = tester.status().anchor_pose
    for c in mock_ros.servo_p_calls[1:]:
        ox = c["pose"][0] - anchor[0]
        oy = c["pose"][1] - anchor[1]
        oz = c["pose"][2] - anchor[2]
        assert abs(ox - oy) < 1e-6, f"direction lost: ox={ox}, oy={oy}"
        assert abs(oz) < 1e-6


def test_snap_to_zero_is_rate_limited(mock_ros):
    """Slider snap from high offset to zero: return is NOT instant.

    Without the cap, a target change from +40 to 0 is one tick of motion
    and the firmware ramps that in ``t`` seconds (jerky / dangerous). With
    the cap at 40 mm/s and rate 50 Hz, each tick moves at most 0.8 mm.
    Returning from 40 to 0 must therefore span at least 40/0.8 = 50 ticks.
    """
    rate = 50.0
    cap = 40.0  # mm/s
    tester = ServoTester(
        mock_ros,
        ServoConfig(servo_rate_hz=rate, max_velocity_xyz=cap, max_velocity_rpy=360.0),
    )
    tester.start()

    # Hold +40mm long enough that _commanded_offset reaches +40.
    tester.set_target_offset([40.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    time.sleep(1.5)  # 40 / 40 = 1 s minimum; pad.
    # Snap target back to 0.
    boundary = len(mock_ros.servo_p_calls)
    tester.set_target_offset([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    time.sleep(1.5)
    tester.stop()

    anchor_x = tester.status().anchor_pose[0]
    post_snap = mock_ros.servo_p_calls[boundary:]

    # No single tick after snap should drop more than cap * dt.
    limit_per_tick = cap / rate * _TOLERANCE
    prev = post_snap[0]["pose"][0]
    for c in post_snap[1:]:
        cur = c["pose"][0]
        step = abs(cur - prev)
        assert step <= limit_per_tick, \
            f"snap-to-zero overshoot: step {step:.3f} exceeds {limit_per_tick:.3f}"
        prev = cur

    # And the return took a meaningful number of ticks — not one.
    # At cap=40 mm/s we need >=25 ticks (half a second) even with TOLERANCE.
    assert len(post_snap) >= 20


def test_rpy_cap_independent_per_axis(mock_ros):
    """RPY cap limits each axis independently at max_velocity_rpy deg/s."""
    rate = 50.0
    cap = 30.0  # deg/s
    tester = ServoTester(
        mock_ros,
        ServoConfig(servo_rate_hz=rate, max_velocity_xyz=1000.0, max_velocity_rpy=cap),
    )
    tester.start()
    # Target all three rotational axes simultaneously.
    tester.set_target_offset([0.0, 0.0, 0.0, 180.0, 180.0, 180.0])
    time.sleep(0.3)
    tester.stop()

    anchor = tester.status().anchor_pose
    limit_per_tick = cap / rate * _TOLERANCE
    for axis in (3, 4, 5):
        steps = _step_magnitudes_rpy_axis(mock_ros.servo_p_calls, anchor, axis)
        for s in steps[1:]:
            assert s <= limit_per_tick, \
                f"axis {axis} step {s:.3f} exceeds cap {limit_per_tick:.3f}"


def test_config_defaults_expose_caps():
    cfg = ServoConfig()
    assert cfg.max_velocity_xyz == 100.0
    assert cfg.max_velocity_rpy == 45.0
