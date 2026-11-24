import pytest

from dobot_ros.policies.registry import PolicyRegistry
from dobot_ros.policies.runner import PolicyRunner, RunRequest

QUIET = {"enabled": False}


@pytest.fixture
def runner(tmp_path):
    return PolicyRunner(PolicyRegistry(params_dir=tmp_path / "p"), tmp_path / "t")


def test_builds_full_tower_from_mixed_scene(runner):
    rec = runner.run(RunRequest(slug="stack_all_blocks", scene="mixed_colors", seed=1, pace=0, noise=QUIET))
    res = rec["result"]
    assert res["ok"], res
    assert res["observations"]["tower_blocks"] == 8 and res["observations"]["tower_height_mm"] == 320
    assert res["observations"]["moves"] == 7
    base_id = res["observations"]["order"][0]
    base = next(o for o in rec["final_objects"] if o["id"] == base_id)
    others = [o for o in rec["final_objects"] if o["id"] != base_id]
    assert all(abs(o["position"][0] - base["position"][0]) < 1 and abs(o["position"][1] - base["position"][1]) < 1 for o in others)
    assert all(o["support"] != "table" for o in others)


def test_flat_block_becomes_the_base_in_clutter(runner):
    rec = runner.run(RunRequest(slug="stack_all_blocks", scene="clutter", seed=2, random_positions=True, pace=0, noise=QUIET))
    res = rec["result"]
    assert res["ok"], res
    assert res["observations"]["order"][0].startswith("orange")        # 50×50×20: largest footprint, ungraspable
    assert res["observations"]["tower_blocks"] == 6


def test_crowded_base_is_moved_or_replaced(runner):
    # Big block hemmed in on both jaw sides: cannot be released onto in place.
    scene = {"name": "crowd", "objects": [
        {"label": "block", "color": "red", "size": [60, 60, 40], "position": [-480, -250]},
        {"label": "block", "color": "blue", "size": [40, 40, 40], "position": [-480, -200]},
        {"label": "block", "color": "green", "size": [40, 40, 40], "position": [-480, -300]},
    ]}
    rec = runner.run(RunRequest(slug="stack_all_blocks", scene=scene, pace=0, noise=QUIET))
    res = rec["result"]
    assert res["ok"], res
    assert res["observations"]["tower_blocks"] == 3
    logs = " ".join(e["message"] for e in rec["events"] if e["kind"] == "log")
    assert "no clear base" in logs or "base: blue" in logs or "base: green" in logs


def test_height_cap_is_respected(runner):
    rec = runner.run(RunRequest(slug="stack_all_blocks", scene="three_colors", pace=0, noise=QUIET,
                                params={"max_tower_mm": 100}))
    res = rec["result"]
    assert res["ok"] is False and "exceed" in res["reason"]
    assert res["observations"]["tower_blocks"] == 2


def test_batch_default_noise(runner):
    s = runner.batch(RunRequest(slug="stack_all_blocks", scene="three_colors", seed=300), n=15)
    assert s["success_rate"] >= 0.9, s["reasons"]
