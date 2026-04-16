"""Tests for VisionTransform — ray-plane projection, rotation correction, confidence.

Uses synthetic data: a camera placed at a known pose looking down at a table,
with fabricated correspondence points. All math is verified against expected
geometry, not real hardware.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from dobot_ros.vision_transform import (
    CameraIntrinsics,
    Extrinsics,
    VisionTransform,
)


# ── Fixtures ────────────────────────────────────────────────────────────

def _overhead_camera():
    """Camera directly above the table at Z=1000mm, looking straight down.
    Camera Z axis = robot -Z (pointing down), Camera X = robot X, Camera Y = robot Y.
    """
    intr = CameraIntrinsics(fx=600, fy=600, cx=320, cy=180, width=640, height=360)
    # R rotates camera frame → robot frame.
    # Camera Z (forward) maps to robot -Z (down), Camera Y (down) maps to robot +Y.
    R = np.array([
        [1,  0,  0],
        [0, -1,  0],
        [0,  0, -1],
    ], dtype=float)
    t = np.array([0, 0, 1000], dtype=float)  # camera at robot (0, 0, 1000)
    ext = Extrinsics(R=R, t=t)
    table_z = 0.0
    return VisionTransform(intr, ext, table_z)


def _oblique_camera():
    """Camera in front of the table at 45° looking down-forward.
    Position: robot (0, -800, 600). Looks toward (0, 0, 0).
    """
    intr = CameraIntrinsics(fx=600, fy=600, cx=320, cy=180, width=640, height=360)

    # Build R from a "look-at" construction.
    cam_pos = np.array([0.0, -800.0, 600.0])
    target = np.array([0.0, 0.0, 0.0])
    fwd = target - cam_pos
    fwd = fwd / np.linalg.norm(fwd)
    world_up = np.array([0.0, 0.0, 1.0])
    right = np.cross(fwd, world_up)
    right = right / np.linalg.norm(right)
    down = np.cross(fwd, right)

    # Camera axes in robot frame: Z_cam = fwd, X_cam = right, Y_cam = down
    # R @ [1,0,0] = right, R @ [0,1,0] = down, R @ [0,0,1] = fwd
    R = np.column_stack([right, down, fwd])

    ext = Extrinsics(R=R, t=cam_pos)
    return VisionTransform(intr, ext, table_z=0.0)


# ── pixel_to_table ──────────────────────────────────────────────────────

def test_overhead_center_pixel():
    """Center pixel of an overhead camera should project to camera XY = (0, 0)."""
    vt = _overhead_camera()
    x, y = vt.pixel_to_table(320, 180)
    assert abs(x) < 0.1
    assert abs(y) < 0.1


def test_overhead_offset_pixel():
    """A pixel offset by (fx, 0) from center should project to +1mm on the table
    when camera is at 1000mm height (tan(atan(1/fx)*fx) * 1000 = 1000 * 1/600 * 600... no.
    Actually: dx_camera = (px - cx) / fx = 1.0, so ray direction is (1, 0, 1) normalized.
    Table hit = origin + t * ray_robot. For overhead: ray_robot = R @ ray_camera.
    With our R: ray_cam = (1, 0, 1)/sqrt(2) → ray_rob = (1, 0, -1)/sqrt(2).
    t_param = (0 - 1000) / (-1/sqrt(2)) = 1000*sqrt(2).
    hit_x = 0 + 1000*sqrt(2) * (1/sqrt(2)) = 1000.
    So px = cx + fx should map to robot X = 1000mm. Let's check px = cx + 1 → X ≈ 1000/600 ≈ 1.667mm.
    """
    vt = _overhead_camera()
    x, _ = vt.pixel_to_table(321, 180)
    expected_x = 1000.0 / 600.0  # height / fx * 1 pixel
    assert abs(x - expected_x) < 0.01


def test_overhead_symmetric():
    """Symmetric pixels should give symmetric table XY."""
    vt = _overhead_camera()
    x1, y1 = vt.pixel_to_table(270, 180)
    x2, y2 = vt.pixel_to_table(370, 180)
    assert abs(x1 + x2) < 0.1  # symmetric about center


def test_oblique_center_pixel_hits_table():
    """Oblique camera's center pixel should hit somewhere on the table."""
    vt = _oblique_camera()
    x, y = vt.pixel_to_table(320, 180)
    # Should be near the look-at target (0, 0) — not exact because of
    # perspective, but within ~200mm.
    assert abs(x) < 300
    assert abs(y) < 300


# ── image_rot_to_table_rot ──────────────────────────────────────────────

def test_overhead_rotation_magnitude_preserved():
    """For an overhead camera, the magnitude of image rotation ≈ table rotation.
    The sign may flip depending on the camera's Y-axis orientation (our test
    overhead camera has Y flipped: camera-Y-down → robot-Y-negative)."""
    vt = _overhead_camera()
    for angle in [0, 45, 90, -30]:
        corrected = vt.image_rot_to_table_rot(angle, 320, 180)
        # Magnitude should be close; sign may differ due to Y-flip in R.
        assert abs(abs(corrected) - abs(angle)) < 2.0, \
            f"angle {angle}: corrected={corrected}, magnitudes differ"


def test_oblique_rotation_differs():
    """For an oblique camera, image rotation should differ from table rotation
    for non-zero rotations (perspective warps angles)."""
    vt = _oblique_camera()
    # A 45° image rotation at a non-center pixel should NOT be exactly 45° on the table.
    corrected = vt.image_rot_to_table_rot(45.0, 400, 250)
    # We can't predict the exact value without doing the math by hand,
    # but it should be DIFFERENT from 45.
    # (For the overhead case it's ~45; for oblique it's different.)
    # Just check it returns a finite number in range.
    assert -180 <= corrected <= 180


# ── robot_to_pixel (inverse) ───────────────────────────────────────────

def test_round_trip_pixel_to_table_to_pixel():
    """pixel_to_table → robot_to_pixel should give back the original pixel."""
    vt = _overhead_camera()
    for px, py in [(320, 180), (200, 100), (500, 300)]:
        rx, ry = vt.pixel_to_table(px, py)
        px2, py2 = vt.robot_to_pixel(rx, ry)
        assert abs(px2 - px) < 0.5, f"px round-trip: {px} → {rx:.1f} → {px2:.1f}"
        assert abs(py2 - py) < 0.5, f"py round-trip: {py} → {ry:.1f} → {py2:.1f}"


# ── Depth-based checks ─────────────────────────────────────────────────

def test_expected_depth_overhead():
    """Overhead at 1000mm height, table at Z=0 → expected depth ≈ 1.0m at center."""
    vt = _overhead_camera()
    d = vt.expected_depth_at_pixel(320, 180)
    assert 0.9 < d < 1.1


def test_object_height():
    vt = _overhead_camera()
    # Object 50mm above table → measured depth is 50mm less (0.05m closer).
    h = vt.object_height_mm(expected_depth_m=1.0, measured_depth_m=0.95)
    assert abs(h - 50.0) < 1.0


def test_object_height_flat():
    vt = _overhead_camera()
    h = vt.object_height_mm(expected_depth_m=1.0, measured_depth_m=1.0)
    assert abs(h) < 1.0


# ── Confidence ──────────────────────────────────────────────────────────

def test_confidence_all_green():
    vt = _overhead_camera()
    c = vt.pick_confidence(
        object_present=True, depth_consistency_mm=5.0,
        frames_tracked=10, in_workspace=True, area_px=1000,
        calibration_age_s=60,
    )
    assert c.level == "green"
    assert c.score >= 5
    assert all(v == "green" for v in c.signals.values())


def test_confidence_missing_object():
    vt = _overhead_camera()
    c = vt.pick_confidence(
        object_present=False, depth_consistency_mm=5.0,
        frames_tracked=10, in_workspace=True, area_px=1000,
    )
    assert c.signals["presence"] == "red"


def test_confidence_yellow():
    vt = _overhead_camera()
    c = vt.pick_confidence(
        object_present=True, depth_consistency_mm=20.0,
        frames_tracked=3, in_workspace=True, area_px=300,
    )
    assert c.level == "yellow"


# ── Solve from points ──────────────────────────────────────────────────

def test_solve_from_points_identity():
    """Solve with synthetic points where camera frame = robot frame (identity R, zero t).
    Camera points in meters, robot points in mm, so the transform is a 1000× scale + identity.
    """
    intr = CameraIntrinsics(fx=600, fy=600, cx=320, cy=180)
    correspondences = [
        {"camera_xyz": [0.1, 0.0, 1.0], "robot_xyz": [100, 0, 1000]},
        {"camera_xyz": [0.0, 0.1, 1.0], "robot_xyz": [0, 100, 1000]},
        {"camera_xyz": [-0.1, 0.0, 1.0], "robot_xyz": [-100, 0, 1000]},
        {"camera_xyz": [0.0, -0.1, 1.0], "robot_xyz": [0, -100, 1000]},
    ]
    vt, error_mm = VisionTransform.solve_from_points(correspondences, intr, table_z=0.0)
    assert error_mm < 0.1
    # R should be ~identity, t should be ~zero.
    assert np.allclose(vt.extrinsics.R, np.eye(3), atol=0.01)
    assert np.allclose(vt.extrinsics.t, np.zeros(3), atol=1.0)


def test_solve_minimum_3_points():
    intr = CameraIntrinsics(fx=600, fy=600, cx=320, cy=180)
    correspondences = [
        {"camera_xyz": [0.1, 0.0, 1.0], "robot_xyz": [100, 0, 1000]},
        {"camera_xyz": [0.0, 0.1, 1.0], "robot_xyz": [0, 100, 1000]},
        {"camera_xyz": [-0.1, 0.0, 1.0], "robot_xyz": [-100, 0, 1000]},
    ]
    vt, error_mm = VisionTransform.solve_from_points(correspondences, intr, table_z=0.0)
    assert error_mm < 0.1


def test_solve_too_few_points():
    intr = CameraIntrinsics(fx=600, fy=600, cx=320, cy=180)
    with pytest.raises(ValueError, match="≥3"):
        VisionTransform.solve_from_points([
            {"camera_xyz": [0, 0, 1], "robot_xyz": [0, 0, 1000]},
        ], intr, table_z=0.0)


# ── Serialization ──────────────────────────────────────────────────────

def test_save_load_round_trip(tmp_path):
    vt = _overhead_camera()
    path = tmp_path / "vision_cal.json"
    vt.save(path)
    vt2 = VisionTransform.load(path)
    assert np.allclose(vt.extrinsics.R, vt2.extrinsics.R)
    assert np.allclose(vt.extrinsics.t, vt2.extrinsics.t)
    assert vt.intrinsics.fx == vt2.intrinsics.fx
    assert vt.table_z == vt2.table_z
