"""Edge-case tests for the safety clamp layer — NaN, Inf, extreme values."""

import math

from dobot_ros.vla.safety import SafetyLimits, clamp_delta, clamp_pose, apply


def test_clamp_delta_nan_returns_zero():
    lims = SafetyLimits()
    delta = [float('nan'), 1.0, 2.0, 0.0, 0.0, 0.0]
    clamped, reasons = clamp_delta(delta, lims)
    assert clamped == [0.0] * 6
    assert any("non-finite" in r for r in reasons)


def test_clamp_delta_inf_returns_zero():
    lims = SafetyLimits()
    delta = [0.0, float('inf'), 0.0, 0.0, 0.0, 0.0]
    clamped, reasons = clamp_delta(delta, lims)
    assert clamped == [0.0] * 6


def test_clamp_pose_nan_reported():
    lims = SafetyLimits()
    pose = [100, 200, float('nan'), 0, 0, 0]
    clamped, reasons = clamp_pose(pose, lims)
    assert any("non-finite" in r for r in reasons)


def test_apply_with_nan_delta():
    lims = SafetyLimits(max_xyz_step_mm=50.0)
    current = [100, 100, 300, 0, 0, 0]
    delta = [float('nan'), 0, 0, 0, 0, 0]
    result = apply(current, delta, lims)
    assert result.clamped
    # Delta should be zeroed out (NaN guard in clamp_delta).
    assert all(math.isfinite(v) for v in result.delta)


def test_clamp_delta_large_values():
    lims = SafetyLimits(max_xyz_step_mm=10.0, max_rot_step_deg=5.0)
    delta = [1000.0, -1000.0, 500.0, 90.0, -90.0, 45.0]
    clamped, reasons = clamp_delta(delta, lims)
    # XYZ should be scaled down proportionally.
    xyz_norm = math.sqrt(sum(c**2 for c in clamped[:3]))
    assert abs(xyz_norm - 10.0) < 0.01
    # Rotations capped at 5.
    assert all(abs(clamped[i]) <= 5.0 for i in range(3, 6))
    assert len(reasons) > 0


def test_clamp_pose_all_axes_out_of_bounds():
    lims = SafetyLimits(x_min=-100, x_max=100, y_min=-100, y_max=100,
                        z_min=50, z_max=500)
    pose = [999, -999, -10, 0, 0, 0]
    clamped, reasons = clamp_pose(pose, lims)
    assert clamped[0] == 100
    assert clamped[1] == -100
    assert clamped[2] == 50
    assert len(reasons) >= 3


def test_apply_preserves_current_when_delta_is_zero():
    lims = SafetyLimits()
    current = [200, -100, 300, 10, 20, 30]
    result = apply(current, [0, 0, 0, 0, 0, 0], lims)
    assert not result.clamped
    assert result.pose == current
