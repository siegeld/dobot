"""Test safety clamps: per-step caps + workspace bounds."""

from __future__ import annotations

from dobot_ros.vla.safety import SafetyLimits, clamp_delta, clamp_pose, apply


def test_xyz_step_cap_preserves_direction():
    lims = SafetyLimits(max_xyz_step_mm=10.0)
    delta = [30.0, 40.0, 0.0, 0.0, 0.0, 0.0]  # norm=50
    clamped, reasons = clamp_delta(delta, lims)
    dx, dy = clamped[0], clamped[1]
    assert abs(dx * dx + dy * dy - 100.0) < 1e-6  # norm == 10
    assert dx > 0 and dy > 0 and dx / dy == 30 / 40
    assert any("xyz_step_norm" in r for r in reasons)


def test_rot_step_cap():
    lims = SafetyLimits(max_rot_step_deg=2.0)
    delta = [0, 0, 0, 10.0, -5.0, 1.0]
    clamped, reasons = clamp_delta(delta, lims)
    assert clamped[3] == 2.0
    assert clamped[4] == -2.0
    assert clamped[5] == 1.0
    assert any("drx" in r for r in reasons)


def test_pose_clamp_to_workspace():
    lims = SafetyLimits(x_min=-100, x_max=100, z_min=50, z_max=500)
    pose, reasons = clamp_pose([150, 0, 20, 0, 0, 0], lims)
    assert pose[0] == 100
    assert pose[2] == 50
    assert any("x=" in r for r in reasons)
    assert any("z=" in r for r in reasons)


def test_apply_full_pipeline_floor():
    lims = SafetyLimits(
        max_xyz_step_mm=50.0, max_rot_step_deg=5.0,
        x_min=-500, x_max=500, y_min=-500, y_max=500,
        z_min=100, z_max=800,
    )
    current = [0, 0, 110, 0, 0, 0]
    # Request a -50 mm step in Z, which would drop below floor (Z_min=100).
    res = apply(current, [0, 0, -50, 0, 0, 0], lims)
    assert res.clamped
    assert res.pose[2] == 100  # clamped to floor
    # Effective delta after clamping.
    assert res.delta[2] == -10


def test_apply_noop_when_in_bounds():
    lims = SafetyLimits(max_xyz_step_mm=50.0, max_rot_step_deg=10.0)
    current = [100, 100, 300, 0, 0, 0]
    res = apply(current, [1, 2, 3, 0.1, 0.2, 0.3], lims)
    assert not res.clamped
    assert res.reasons == []
    assert res.pose == [101, 102, 303, 0.1, 0.2, 0.3]
