import pytest

from dobot_ros.policies.registry import PolicyRegistry
from dobot_ros.policies.runner import PolicyRunner, RunRequest


@pytest.fixture
def runner(tmp_path):
    return PolicyRunner(PolicyRegistry(params_dir=tmp_path / "params"), tmp_path / "traces")


def test_registry_discovers_policy():
    reg = PolicyRegistry()
    assert "pick_colored_block" in reg.slugs()
    meta = reg.list_policies()[0]
    assert meta["slug"] == "pick_colored_block"
    assert any(p["name"] == "color" for p in meta["parameters"])
    assert "class PickColoredBlock" in reg.source("pick_colored_block")


def test_params_validate_and_persist(tmp_path):
    reg = PolicyRegistry(params_dir=tmp_path)
    saved = reg.set_params("pick_colored_block", {"color": "blue", "max_attempts": 99, "bogus": 1})
    assert saved["color"] == "blue" and saved["max_attempts"] == 5 and "bogus" not in saved
    assert reg.get_params("pick_colored_block")["color"] == "blue"


def test_pick_red_block_succeeds(runner):
    rec = runner.run(RunRequest(slug="pick_colored_block", scene="two_blocks", seed=1, pace=0.0,
                                params={"color": "red"}))
    res = rec["result"]
    assert res["ok"], res
    assert res["attempts"] == 1
    assert 38 < res["observations"]["jaw_width_mm"] < 60
    red = next(o for o in rec["final_objects"] if o["id"] == "red_block")
    assert red["support"] == "gripper"
    assert red["position"][2] > rec["initial_objects"][0]["position"][2] + 100
    names = [e.get("name") for e in rec["events"] if e["kind"] == "call"]
    assert names[:2] == ["set_tool", "look"]
    assert "grasp" in names and names[-1] == "move_l"


def test_missing_color_fails_gracefully(runner):
    rec = runner.run(RunRequest(slug="pick_colored_block", scene="two_blocks", seed=1, pace=0.0,
                                params={"color": "green"}))
    assert rec["result"]["ok"] is False
    assert "no match" in rec["result"]["reason"]
    assert rec["result"]["attempts"] == 0


def test_zero_noise_batch_is_perfect(runner):
    s = runner.batch(RunRequest(slug="pick_colored_block", scene="three_colors", seed=100, pace=0.0,
                                params={"color": "blue"}, noise={"enabled": False}), n=10)
    assert s["success_rate"] == 1.0, s["reasons"]


def test_default_noise_batch_success_rate(runner):
    s = runner.batch(RunRequest(slug="pick_colored_block", scene="clutter", seed=0, pace=0.0,
                                params={"color": "red"}), n=30)
    assert s["success_rate"] >= 0.9, s["reasons"]


def test_trace_store_roundtrip(runner):
    rec = runner.run(RunRequest(slug="pick_colored_block", scene="two_blocks", seed=2, pace=0.0))
    listed = runner.list_traces()
    assert listed[0]["id"] == rec["id"] and listed[0]["ok"] is True
    assert runner.get_trace(rec["id"])["events"]
    assert runner.delete_trace(rec["id"]) and runner.get_trace(rec["id"]) is None


def test_stack_policy_registered_and_succeeds(runner):
    assert "stack_colored_blocks" in runner.registry.slugs()
    rec = runner.run(RunRequest(slug="stack_colored_blocks", scene="two_blocks", seed=4, pace=0.0,
                                params={"top_color": "red", "base_color": "blue"}))
    assert rec["result"]["ok"], rec["result"]
    red = next(o for o in rec["final_objects"] if o["id"] == "red_block")
    assert red["support"] == "blue_block"


def test_stack_policy_batch(runner):
    s = runner.batch(RunRequest(slug="stack_colored_blocks", scene="three_colors", seed=10, pace=0.0,
                                params={"top_color": "green", "base_color": "blue"}), n=20)
    assert s["success_rate"] >= 0.85, s["reasons"]
