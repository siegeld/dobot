"""Tests for motion validation functions — the last-resort safety net.

These test the standalone validation module (no ROS2 deps needed).
"""

import math
import pytest

from dobot_ros.validation import validate_finite, validate_pose_bounds, validate_joint_bounds


class TestValidateFinite:
    def test_rejects_nan(self):
        with pytest.raises(ValueError, match="NaN"):
            validate_finite([1.0, float('nan'), 3.0], "test")

    def test_rejects_inf(self):
        with pytest.raises(ValueError, match="Inf"):
            validate_finite([float('inf'), 0, 0], "test")

    def test_rejects_negative_inf(self):
        with pytest.raises(ValueError, match="Inf"):
            validate_finite([0, float('-inf'), 0], "test")

    def test_accepts_normal_values(self):
        validate_finite([1.0, -2.5, 0.0, 999.9, -0.001, 360.0], "test")

    def test_accepts_zero(self):
        validate_finite([0.0, 0.0, 0.0], "test")


class TestValidatePoseBounds:
    def test_rejects_x_too_large(self):
        with pytest.raises(ValueError, match="X="):
            validate_pose_bounds([2000, 0, 100, 0, 0, 0])

    def test_rejects_y_too_large(self):
        with pytest.raises(ValueError, match="Y="):
            validate_pose_bounds([0, -2000, 100, 0, 0, 0])

    def test_rejects_z_below_floor(self):
        with pytest.raises(ValueError, match="Z=.*below"):
            validate_pose_bounds([0, 0, -500, 0, 0, 0])

    def test_rejects_z_above_ceiling(self):
        with pytest.raises(ValueError, match="Z=.*ceiling"):
            validate_pose_bounds([0, 0, 2000, 0, 0, 0])

    def test_rejects_rotation_too_large(self):
        with pytest.raises(ValueError, match="RX="):
            validate_pose_bounds([0, 0, 100, 500, 0, 0])

    def test_accepts_normal_pose(self):
        validate_pose_bounds([400, -200, 300, 180, 0, 45])

    def test_accepts_negative_z_within_floor(self):
        validate_pose_bounds([0, 0, -100, 0, 0, 0])

    def test_accepts_boundary_values(self):
        validate_pose_bounds([1500, -1500, 1500, 360, -360, 360])
        validate_pose_bounds([0, 0, -200, 0, 0, 0])


class TestValidateJointBounds:
    def test_rejects_j3_out_of_range(self):
        with pytest.raises(ValueError, match="J3="):
            validate_joint_bounds([0, 0, 200, 0, 0, 0])

    def test_accepts_normal_joints(self):
        validate_joint_bounds([10, -30, 90, 0, 45, -120])

    def test_accepts_boundary_values(self):
        validate_joint_bounds([360, 360, 160, 360, 360, 360])
        validate_joint_bounds([-360, -360, -160, -360, -360, -360])

    def test_rejects_j1_out_of_range(self):
        with pytest.raises(ValueError, match="J1="):
            validate_joint_bounds([400, 0, 0, 0, 0, 0])
