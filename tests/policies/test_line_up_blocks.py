import math

import pytest

from dobot_ros.policies.registry import PolicyRegistry
from dobot_ros.policies.runner import PolicyRunner, RunRequest
from dobot_ros.policies.line_up_blocks import LineUpBlocks, _yaw_err
from dobot_ros.policies.guards import WorkspaceBounds
from dobot_ros.policies.types import WorldObject

QUIET = {"enabled": False}
WS = WorkspaceBounds(-676, -271, -679, 86)


@pytest.fixture
def runner(tmp_path):
    return PolicyRunner(PolicyRegistry(params_dir=tmp_path / "p"), tmp_path / "t")


def params(**over):
    base = {p.name: p.default for p in LineUpBlocks.parameter_defs()}
    base.update(over)
    return base


def _row_ok(objs, axis, tol_mm=2.0, tol_deg=1.0):
    """Every object square to the axes, all on one line, evenly spaced."""
    cross = [o["position"][1 - axis] for o in objs]
    assert max(cross) - min(cross) < tol_mm, cross
    along = sorted(o["position"][axis] for o in objs)
    gaps = [b - a for a, b in zip(along, along[1:])]
    assert max(gaps) - min(gaps) < tol_mm, gaps
    assert all(_yaw_err(o["yaw_deg"], 0.0) < tol_deg for o in objs), [o["yaw_deg"] for o in objs]
    return gaps[0] if gaps else None


def test_layout_along_y_is_centred_and_fits():
    slots, pitch, yaw = LineUpBlocks.layout(8, 40.0, WS, params())
    assert pitch == 60 and yaw == 0.0
    assert all(round(s[0], 1) == -341.0 for s in slots)
    ys = [s[1] for s in slots]
    assert round((ys[0] + ys[-1]) / 2.0, 1) == round((WS.y_min + WS.y_max) / 2.0, 1)
    assert round(ys[1] - ys[0]) == 60
    with pytest.raises(ValueError):
        LineUpBlocks.layout(8, 40.0, WS, params(row_axis="x"))      # 420 mm does not fit in 325
    slots_x, _, yaw_x = LineUpBlocks.layout(3, 60.0, WS, params(row_axis="x"))
    assert yaw_x == 90.0 and all(round(s[1], 1) == 16.0 for s in slots_x)


def test_layout_avoids_a_doubtful_block_on_the_row_line():
    flat = WorldObject(id="flat", label="block", color="orange", position=[-341.0, -300.0, 0.0],
                       size=[50, 50, 20], yaw_deg=0.0)
    base, _, _ = LineUpBlocks.layout(6, 60.0, WS, params())
    shifted, _, _ = LineUpBlocks.layout_avoiding(6, 60.0, WS, params(), [flat], 8.0)
    assert shifted != base and shifted[0][0] < base[0][0] - 60      # moved deeper into the table
    same, _, _ = LineUpBlocks.layout_avoiding(6, 60.0, WS, params(), [], 8.0)
    assert same == base


def test_assignment_orders():
    def blk(i, color, size, y):
        return WorldObject(id=f"b{i}", label="block", color=color, position=[-500.0, y, 20.0], size=size, yaw_deg=0.0)
    objs = [blk(0, "red", [40, 40, 40], -100), blk(1, "blue", [60, 30, 30], -400), blk(2, "red", [40, 40, 40], -250)]
    slots = [[-341.0, -400.0], [-341.0, -300.0], [-341.0, -200.0]]
    by_size = LineUpBlocks._assign(objs, slots, 1, 25.0, 0.0, 15.0, "size")
    assert by_size["b1"] == 0 and by_size["b2"] == 1 and by_size["b0"] == 2      # largest first, ties by position
    by_color = LineUpBlocks._assign(objs, slots, 1, 25.0, 0.0, 15.0, "color")
    assert by_color["b1"] == 0 and by_color["b2"] == 1 and by_color["b0"] == 2
    seated = blk(3, "green", [40, 40, 40], -300)
    seated.position[0] = -341.0
    any_ = LineUpBlocks._assign(objs + [seated], slots + [[-341.0, -100.0]], 1, 25.0, 0.0, 15.0, "any")
    assert any_["b3"] == 1                                                      # already seated keeps its slot
    assert sorted(any_.values()) == [0, 1, 2, 3]
    marg = LineUpBlocks._assign(objs, slots, 1, 25.0, 0.0, 15.0, "size", marginal={"b1": 0.0, "b0": 10.0})
    assert marg["b2"] == 0 and marg["b1"] == 1 and marg["b0"] == 2             # doubtful grips last, most doubtful at the end


def test_lines_up_mixed_scene_square_and_even(runner):
    rec = runner.run(RunRequest(slug="line_up_blocks", scene="mixed_colors", seed=1, pace=0, noise=QUIET))
    res = rec["result"]
    assert res["ok"], res
    assert res["observations"]["moves"] == 8 and res["observations"]["in_row"] == 8
    assert round(_row_ok(rec["final_objects"], axis=1)) == 60


def test_color_order_groups_colours(runner):
    rec = runner.run(RunRequest(slug="line_up_blocks", scene="mixed_colors", seed=2, random_positions=True, pace=0,
                                noise=QUIET, params={"order": "color"}))
    assert rec["result"]["ok"], rec["result"]
    by_y = sorted(rec["final_objects"], key=lambda o: o["position"][1])
    assert [o["color"] for o in by_y] == ["blue"] * 3 + ["green"] * 2 + ["red"] * 3


def test_row_along_x(runner):
    rec = runner.run(RunRequest(slug="line_up_blocks", scene="three_colors", seed=4, random_positions=True, pace=0,
                                noise=QUIET, params={"row_axis": "x"}))
    assert rec["result"]["ok"], rec["result"]
    assert round(_row_ok(rec["final_objects"], axis=0)) == 80


def test_already_lined_up_makes_no_moves(runner):
    slots, _, _ = LineUpBlocks.layout(3, 40.0, WS, params())
    scene = {"name": "done", "objects": [
        {"label": "block", "color": c, "size": [40, 40, 40], "position": [round(s[0], 1), round(s[1], 1)], "yaw_deg": 0}
        for c, s in zip(("red", "green", "blue"), slots)]}
    rec = runner.run(RunRequest(slug="line_up_blocks", scene=scene, pace=0, noise=QUIET, params={"order": "any"}))
    assert rec["result"]["ok"] and rec["result"]["observations"]["moves"] == 0


def test_crooked_block_on_its_slot_is_squared(runner):
    slots, _, _ = LineUpBlocks.layout(2, 40.0, WS, params())
    scene = {"name": "crooked", "objects": [
        {"label": "block", "color": "red", "size": [40, 40, 40], "position": [round(slots[0][0], 1), round(slots[0][1], 1)], "yaw_deg": 35},
        {"label": "block", "color": "blue", "size": [40, 40, 40], "position": [-550, -200], "yaw_deg": 0}]}
    rec = runner.run(RunRequest(slug="line_up_blocks", scene=scene, pace=0, noise=QUIET, params={"order": "any"}))
    assert rec["result"]["ok"], rec["result"]
    assert rec["result"]["observations"]["moves"] == 2
    _row_ok(rec["final_objects"], axis=1)


def test_block_sitting_on_another_slot_is_moved_aside_first(runner):
    slots, _, _ = LineUpBlocks.layout(2, 40.0, WS, params())
    # Big block (slot 0 by size) is far away; a small block squats on slot 0.
    scene = {"name": "squat", "objects": [
        {"id": "big", "label": "block", "color": "red", "size": [60, 60, 40], "position": [-600, -100], "yaw_deg": 0},
        {"id": "small", "label": "block", "color": "blue", "size": [40, 40, 40], "position": [round(slots[0][0], 1), round(slots[0][1], 1) + 10], "yaw_deg": 20}]}
    rec = runner.run(RunRequest(slug="line_up_blocks", scene=scene, pace=0, noise=QUIET))
    res = rec["result"]
    assert res["ok"], res
    logs = [e["message"] for e in rec["events"] if e["kind"] == "log" and e["message"].startswith("move 1")]
    assert "small" in logs[0] and "is on slot 0" in logs[0]
    assert res["observations"]["row"] == ["big", "small"]


def test_flat_block_left_out_with_reason_rest_lined_up(runner):
    rec = runner.run(RunRequest(slug="line_up_blocks", scene="clutter", seed=3, random_positions=True, pace=0, noise=QUIET))
    res = rec["result"]
    assert res["ok"] is False
    assert "orange" in res["reason"] and "clearance floor" in res["reason"]
    assert res["observations"]["in_row"] == 5
    rest = [o for o in rec["final_objects"] if not o["id"].startswith("orange")]
    assert round(_row_ok(rest, axis=1)) == 80


def test_row_shifts_away_from_an_immovable_block_on_the_line(runner):
    slots, _, _ = LineUpBlocks.layout(3, 50.0, WS, params())
    scene = {"name": "flat_on_line", "objects": [
        {"id": "flat", "label": "block", "color": "orange", "size": [50, 50, 20], "position": [round(slots[1][0], 1), round(slots[1][1], 1)], "yaw_deg": 0},
        {"label": "block", "color": "red", "size": [40, 40, 40], "position": [-550, -150], "yaw_deg": 10},
        {"label": "block", "color": "blue", "size": [40, 40, 40], "position": [-600, -450], "yaw_deg": -30}]}
    rec = runner.run(RunRequest(slug="line_up_blocks", scene=scene, pace=0, noise=QUIET))
    res = rec["result"]
    assert res["ok"] is False and "flat" in res["reason"]
    rest = [o for o in rec["final_objects"] if o["id"] != "flat"]
    _row_ok(rest, axis=1)
    assert all(o["position"][0] < slots[0][0] - 40 for o in rest)      # the row moved off the flat block


def test_batch_default_noise(runner):
    s = runner.batch(RunRequest(slug="line_up_blocks", scene="mixed_colors", seed=300), n=10)
    assert s["success_rate"] >= 0.9, s["reasons"]


def test_terminates_under_heavy_dropout(runner):
    heavy = {"xy_sigma_mm": 10, "yaw_sigma_deg": 12, "miss_prob": 0.3, "height_sigma_mm": 25}
    rec = runner.run(RunRequest(slug="line_up_blocks", scene="mixed_colors", seed=7, random_positions=True, pace=0, noise=heavy))
    assert rec["result"] is not None
    assert rec["elapsed_wall_s"] < 60
