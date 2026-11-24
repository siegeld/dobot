"""CR5 FK/IK model tests. No robot."""
import math
import numpy as np
import pytest

from dobot_ros.sim import kinematics as K


def test_fk_zero_pose_is_arm_straight_up():
    p = K.fk([0, 0, 0, 0, 0, 0])
    # 147 + 427 + 357 + 116 = 1047 mm tall; lateral offset 141 + 105 = 246.
    assert abs(p[2] - 1047.0) < 0.5
    assert abs(p[0]) < 0.5 and abs(p[1] + 246.0) < 0.5


def test_tool_offset_moves_along_flange_normal():
    q = [10, -20, -100, 30, 80, -15]
    p0 = K.fk(q, 0.0)
    p1 = K.fk(q, 203.0)
    assert abs(math.dist(p0[:3], p1[:3]) - 203.0) < 1e-6
    assert p0[3:] == pytest.approx(p1[3:], abs=1e-9)


def test_vertical_wrist_points_tool_down():
    # RX=180, RY=0 must make tool Z = -Z base under the assumed convention.
    R = K.rpy_to_matrix(180, 0, 37)
    assert np.allclose(R @ np.array([0, 0, 1.0]), [0, 0, -1.0], atol=1e-9)
    # jaw axis (tool X) yaw equals RZ
    tx = R @ np.array([1.0, 0, 0])
    assert abs(math.degrees(math.atan2(tx[1], tx[0])) - 37) < 1e-6


def test_rpy_roundtrip():
    for rx, ry, rz in [(180, 0, 0), (170, -10, 45), (-90, 0, -180), (90, 30, 60), (180, 0, 179.9)]:
        R = K.rpy_to_matrix(rx, ry, rz)
        back = K.matrix_to_rpy(R)
        assert np.allclose(K.rpy_to_matrix(*back), R, atol=1e-9)


@pytest.mark.parametrize("q", [
    [0, 0, -90, 0, 90, 0],
    [10, -20, -100, 30, 80, -15],
    [-120, -40, -110, 50, 95, 120],
    [45, 30, -140, -20, 60, 0],
])
def test_ik_roundtrip(q):
    p = K.fk(q)
    q2 = K.ik(p, seed_deg=[v + 5 for v in q])
    assert K.fk(q2) == pytest.approx(p, abs=0.1)


def test_ik_stays_on_seed_branch():
    q = [-100, -50, -80, 40, 90, 170]
    p = K.fk(q)
    q2 = K.ik(p, seed_deg=q)
    assert max(abs(a - b) for a, b in zip(q, q2)) < 1.0


def test_ik_unreachable_raises():
    with pytest.raises(K.IKError):
        K.ik([1500, 0, 300, 180, 0, 0])


def test_vertical_reach_over_workspace():
    # Three of the four calibrated table corners are reachable with a
    # vertical wrist 50 mm up; the far corner (945 mm radius) is not.
    ok = [K.reachable([x, y, 28 + 50, 180, 0, 0]) for x, y in
          [(-676.7, 80.0), (-281.9, 85.7), (-271.4, -678.9)]]
    assert all(ok)
    assert not K.reachable([-661.1, -676.3, 78, 180, 0, 0])


def test_joint_positions_chain_length():
    pts = K.joint_positions([0, 0, 0, 0, 0, 0])
    assert len(pts) == 7
    assert pts[0] == pytest.approx([0, 0, 147.0], abs=1e-6)
