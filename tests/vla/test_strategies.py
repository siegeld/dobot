"""Tests for pick strategies — parameter validation, edge cases, NaN handling."""

import math
import pytest

from dobot_ros.strategies.base import ParameterDef, PickContext, PickWaypoint
from dobot_ros.strategies.simple_top_down import SimpleTopDown
from dobot_ros.strategies.angled_approach import AngledApproach


def _make_context(**overrides):
    defaults = dict(
        robot_x=300.0, robot_y=-100.0, table_z=28.0, min_clearance_mm=25.0,
        rotation_deg=15.0, current_pose=[300, -100, 300, 180, 0, 0],
        object_height_mm=0.0, object_present=False, object_data=None,
        params={},
    )
    defaults.update(overrides)
    return PickContext(**defaults)


class TestParameterDef:
    def test_validate_float_clamped(self):
        p = ParameterDef("x", "float", 50.0, "test", min=0, max=100)
        assert p.validate(150) == 100.0
        assert p.validate(-10) == 0.0
        assert p.validate(50) == 50.0

    def test_validate_int(self):
        p = ParameterDef("x", "int", 5, "test", min=1, max=10)
        assert p.validate(20) == 10
        assert p.validate(0) == 1

    def test_validate_bool(self):
        p = ParameterDef("x", "bool", True, "test")
        assert p.validate(False) is False
        assert p.validate(1) is True

    def test_validate_choice_fallback(self):
        p = ParameterDef("x", "choice", "a", "test", choices=["a", "b", "c"])
        assert p.validate("b") == "b"
        assert p.validate("invalid") == "a"  # falls back to default


class TestSimpleTopDown:
    def test_basic_plan_has_4_waypoints(self):
        ctx = _make_context(params={"approach_height_mm": 100, "grasp_clearance_mm": 5,
                                     "lift_height_mm": 150, "gripper_open_pos": 800,
                                     "gripper_close_pos": 200, "move_speed": 30})
        plan = SimpleTopDown().plan(ctx)
        assert len(plan.waypoints) == 4
        labels = [w.label for w in plan.waypoints]
        assert any("approach" in l.lower() for l in labels)
        assert any("grasp" in l.lower() for l in labels)
        assert any("retract" in l.lower() for l in labels)

    def test_grasp_z_never_below_min_clearance(self):
        ctx = _make_context(
            params={"grasp_clearance_mm": 0, "use_object_height": True},
            object_present=True, object_height_mm=5.0, min_clearance_mm=25.0,
        )
        plan = SimpleTopDown().plan(ctx)
        grasp_wp = [w for w in plan.waypoints if "grasp" in w.label.lower() and w.gripper_action != "close"][0]
        assert grasp_wp.pose[2] >= ctx.table_z + ctx.min_clearance_mm

    def test_object_height_raises_grasp_z(self):
        ctx = _make_context(
            params={"grasp_clearance_mm": 5, "use_object_height": True},
            object_present=True, object_height_mm=60.0, min_clearance_mm=5.0,
        )
        plan = SimpleTopDown().plan(ctx)
        grasp_wp = [w for w in plan.waypoints if "grasp" in w.label.lower() and w.gripper_action != "close"][0]
        # Height/2 = 30, which is > clearance (5), so grasp_z should be table_z + 30.
        assert grasp_wp.pose[2] >= ctx.table_z + 30.0

    def test_empty_params_uses_defaults(self):
        ctx = _make_context(params={})
        plan = SimpleTopDown().plan(ctx)
        assert len(plan.waypoints) == 4

    def test_rotation_applied_to_rz(self):
        ctx = _make_context(rotation_deg=45.0, params={})
        plan = SimpleTopDown().plan(ctx)
        for wp in plan.waypoints:
            assert wp.pose[5] == 45.0

    def test_nan_rotation_propagates_to_pose(self):
        """NaN rotation from a bad detection should propagate so
        _validate_finite in servo_p/move_pose catches it."""
        ctx = _make_context(rotation_deg=float('nan'), params={})
        plan = SimpleTopDown().plan(ctx)
        # The pose should contain NaN — it's the caller's job to validate.
        assert any(math.isnan(wp.pose[5]) for wp in plan.waypoints)


class TestAngledApproach:
    def test_has_5_waypoints(self):
        ctx = _make_context(params={"approach_direction": "from_back"})
        plan = AngledApproach().plan(ctx)
        assert len(plan.waypoints) == 5

    def test_hover_waypoint_is_offset(self):
        ctx = _make_context(
            params={"approach_offset_mm": 80, "approach_direction": "from_back"},
        )
        plan = AngledApproach().plan(ctx)
        hover = plan.waypoints[0]
        # "from_back" means dx=-1, so hover X should be robot_x - 80.
        assert abs(hover.pose[0] - (ctx.robot_x - 80)) < 1.0

    def test_target_near_base_uses_fallback(self):
        ctx = _make_context(
            robot_x=0.0, robot_y=0.0,
            params={"approach_direction": "auto", "approach_offset_mm": 50},
        )
        plan = AngledApproach().plan(ctx)
        hover = plan.waypoints[0]
        # Fallback direction is from_back: dx=-1, dy=0.
        assert hover.pose[0] < ctx.robot_x  # X is offset negative
