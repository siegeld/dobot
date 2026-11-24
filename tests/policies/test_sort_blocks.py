import pytest

from dobot_ros.policies.registry import PolicyRegistry
from dobot_ros.policies.runner import PolicyRunner, RunRequest
from dobot_ros.policies.sort_blocks_by_color import SortBlocksByColor
from dobot_ros.policies.guards import WorkspaceBounds

QUIET = {"enabled": False}


@pytest.fixture
def runner(tmp_path):
    return PolicyRunner(PolicyRegistry(params_dir=tmp_path / "p"), tmp_path / "t")


def params(**over):
    base = {p.name: p.default for p in SortBlocksByColor.parameter_defs()}
    base.update(over)
    return base


def test_layout_zones_along_y_slots_along_x():
    ws = WorkspaceBounds(-676, -271, -679, 86)
    z = SortBlocksByColor.layout(["blue", "green", "red"], ws, params())
    assert [round(z[c].cy) for c in ("blue", "green", "red")] == [-609, -489, -369]
    assert [round(s[0]) for s in z["red"].slots] == [-341, -401, -461]
    assert z["red"].on_slot([-343, -366], 15) and not z["red"].on_slot([-371, -369], 15)
    with pytest.raises(ValueError):
        SortBlocksByColor.layout([f"c{i}" for i in range(8)], ws, params())


def test_sorts_fixed_mixed_scene(runner):
    rec = runner.run(RunRequest(slug="sort_blocks_by_color", scene="mixed_colors", seed=1, pace=0, noise=QUIET))
    res = rec["result"]
    assert res["ok"], res
    assert res["observations"]["moves"] == 8 and res["observations"]["per_color"] == {"blue": 3, "green": 2, "red": 3}
    ws = WorkspaceBounds(-676, -271, -679, 86)
    zones = SortBlocksByColor.layout(["blue", "green", "red"], ws, params())
    for o in rec["final_objects"]:
        assert zones[o["color"]].on_slot(o["position"][:2], 2.0), o


def test_already_sorted_scene_makes_no_moves(runner):
    # Two colours → zones blue (y=-609) and red (y=-489).
    scene = {"name": "done", "objects": [
        {"label": "block", "color": "red", "size": [40, 40, 40], "position": [-341, -489]},
        {"label": "block", "color": "red", "size": [40, 40, 40], "position": [-401, -489]},
        {"label": "block", "color": "blue", "size": [40, 40, 40], "position": [-341, -609]},
    ]}
    rec = runner.run(RunRequest(slug="sort_blocks_by_color", scene=scene, pace=0, noise=QUIET))
    assert rec["result"]["ok"] and rec["result"]["observations"]["moves"] == 0


def test_wrong_colour_on_a_slot_gets_moved_first(runner):
    scene = {"name": "swap", "objects": [
        {"label": "block", "color": "blue", "size": [40, 40, 40], "position": [-341, -489]},   # sits on red's slot (2 colours: red zone y=-489)
        {"label": "block", "color": "red", "size": [40, 40, 40], "position": [-520, -200]},
    ]}
    rec = runner.run(RunRequest(slug="sort_blocks_by_color", scene=scene, pace=0, noise=QUIET))
    assert rec["result"]["ok"], rec["result"]
    logs = [e["message"] for e in rec["events"] if e["kind"] == "log" and e["message"].startswith("move 1")]
    assert "blue" in logs[0]


def test_flat_block_is_given_up_with_reason_and_rest_sorted(runner):
    rec = runner.run(RunRequest(slug="sort_blocks_by_color", scene="clutter", seed=3, random_positions=True,
                                pace=0, noise=QUIET))
    res = rec["result"]
    assert res["ok"] is False
    assert "orange" in res["reason"] and "clearance floor" in res["reason"]
    assert res["observations"]["unsorted"] == 1 and res["observations"]["moves"] >= 4


def test_batch_default_noise(runner):
    s = runner.batch(RunRequest(slug="sort_blocks_by_color", scene="three_colors", seed=200), n=15)
    assert s["success_rate"] >= 0.9, s["reasons"]


def test_terminates_under_heavy_dropout(runner):
    heavy = {"xy_sigma_mm": 10, "yaw_sigma_deg": 12, "miss_prob": 0.3, "height_sigma_mm": 25}
    rec = runner.run(RunRequest(slug="sort_blocks_by_color", scene="mixed_colors", seed=7, random_positions=True,
                                pace=0, noise=heavy))
    assert rec["result"] is not None
    assert rec["elapsed_wall_s"] < 60
