"""Edge-case tests for VisionTransform — boundary conditions, degenerate inputs."""

import math
import numpy as np
import pytest

from dobot_ros.vision_transform import (
    CameraIntrinsics, Extrinsics, VisionTransform,
)


def _overhead_vt():
    intr = CameraIntrinsics(fx=600, fy=600, cx=320, cy=180)
    R = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], dtype=float)
    t = np.array([0, 0, 1000], dtype=float)
    return VisionTransform(intr, Extrinsics(R=R, t=t), table_z=0.0)


def test_pixel_at_principal_point():
    """Ray through the principal point should hit (0, 0) on the table."""
    vt = _overhead_vt()
    x, y = vt.pixel_to_table(320, 180)  # cx, cy
    assert abs(x) < 0.1
    assert abs(y) < 0.1


def test_pixel_to_table_rejects_parallel_ray():
    """If the ray is parallel to the table, raise ValueError."""
    intr = CameraIntrinsics(fx=600, fy=600, cx=320, cy=180)
    # Camera looking sideways (Z axis parallel to table).
    R = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], dtype=float)
    t = np.array([0, 0, 500], dtype=float)
    vt = VisionTransform(intr, Extrinsics(R=R, t=t), table_z=0.0)
    # Center pixel ray goes along +Y in robot frame → parallel to table (Z=const).
    with pytest.raises(ValueError, match="parallel"):
        vt.pixel_to_table(320, 180)


def test_pixel_to_table_rejects_backward_ray():
    """If the table is behind the camera, raise ValueError."""
    intr = CameraIntrinsics(fx=600, fy=600, cx=320, cy=180)
    # Camera at Z=500 looking UP (+Z in robot frame). Table at Z=0 is behind it.
    # R maps camera +Z to robot +Z (upward), so the ray goes away from the table.
    R = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]], dtype=float)
    t = np.array([0, 0, 500], dtype=float)  # camera above table, looking up
    vt = VisionTransform(intr, Extrinsics(R=R, t=t), table_z=0.0)
    # Center ray: camera Z→robot +Z. Origin at Z=500, table at Z=0.
    # t_param = (0 - 500) / ray_rob_z. ray_rob_z > 0 → t_param < 0 → behind camera.
    with pytest.raises(ValueError, match="behind"):
        vt.pixel_to_table(320, 180)


def test_table_z_zero_is_valid():
    """table_z=0 should work (table at the robot base plane)."""
    vt = _overhead_vt()
    vt.table_z = 0.0
    x, y = vt.pixel_to_table(350, 200)
    assert math.isfinite(x) and math.isfinite(y)


def test_expected_depth_at_center():
    vt = _overhead_vt()
    d = vt.expected_depth_at_pixel(320, 180)
    assert 0.9 < d < 1.1  # camera at 1000mm → ~1.0m


def test_object_height_negative_means_below_table():
    vt = _overhead_vt()
    h = vt.object_height_mm(expected_depth_m=1.0, measured_depth_m=1.05)
    assert h < 0  # object is "below" table → shouldn't happen in practice


def test_solve_collinear_points_rejected():
    """Points on a straight line should be rejected."""
    intr = CameraIntrinsics(fx=600, fy=600, cx=320, cy=180)
    # All points on the X axis.
    correspondences = [
        {"camera_xyz": [0.1, 0, 1], "robot_xyz": [100, 0, 1000]},
        {"camera_xyz": [0.2, 0, 1], "robot_xyz": [200, 0, 1000]},
        {"camera_xyz": [0.3, 0, 1], "robot_xyz": [300, 0, 1000]},
    ]
    with pytest.raises(ValueError, match="collinear"):
        VisionTransform.solve_from_points(correspondences, intr, table_z=0.0)


def test_oblique_pixel_to_table_accuracy():
    """Oblique camera should project pixels accurately after solve."""
    intr = CameraIntrinsics(fx=600, fy=600, cx=320, cy=180)
    # Simulate a camera at (0, -800, 600) looking at origin.
    cam_pos = np.array([0.0, -800.0, 600.0])
    target = np.array([0.0, 0.0, 0.0])
    fwd = target - cam_pos
    fwd = fwd / np.linalg.norm(fwd)
    world_up = np.array([0.0, 0.0, 1.0])
    right = np.cross(fwd, world_up)
    right = right / np.linalg.norm(right)
    down = np.cross(fwd, right)
    R = np.column_stack([right, down, fwd])
    ext = Extrinsics(R=R, t=cam_pos)
    vt = VisionTransform(intr, ext, table_z=0.0)

    # Project a known robot point to pixel and back.
    robot_x, robot_y = 100.0, -50.0
    px, py = vt.robot_to_pixel(robot_x, robot_y)
    rx, ry = vt.pixel_to_table(px, py)
    assert abs(rx - robot_x) < 0.5
    assert abs(ry - robot_y) < 0.5
