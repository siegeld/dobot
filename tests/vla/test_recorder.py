"""Test the episode recorder: on-disk schema, action computation, start/stop."""

from __future__ import annotations

import json
import time
from pathlib import Path

from dobot_ros.vla.recorder import EpisodeRecorder, list_episodes


def test_record_one_episode_end_to_end(mock_ros, mock_camera, tmp_episodes):
    rec = EpisodeRecorder(
        ros_client=mock_ros, camera=mock_camera,
        episodes_dir=str(tmp_episodes), rate_hz=20.0,  # fast for tests
    )

    ep_dir = rec.start("pick up the red block", episode_name="test")
    assert rec.is_recording()

    # Let a few frames accumulate, nudging the robot state between them so
    # actions are non-zero.
    for _ in range(5):
        time.sleep(0.08)
        mock_ros._pose[0] += 1.5  # X drifts +1.5 mm between captures

    sealed = rec.stop()
    assert sealed == ep_dir
    assert not rec.is_recording()

    # Structure.
    assert (sealed / "meta.json").exists()
    assert (sealed / "steps.jsonl").exists()
    assert (sealed / "frames").is_dir()
    frames = sorted((sealed / "frames").iterdir())
    assert len(frames) >= 3
    for f in frames:
        assert f.name.endswith(".jpg")
        assert f.stat().st_size > 0  # non-empty JPEG

    # Meta.
    meta = json.loads((sealed / "meta.json").read_text())
    assert meta["instruction"] == "pick up the red block"
    assert meta["num_steps"] == len(frames)
    assert meta["rate_hz"] == 20.0
    assert meta["image_shape"] == [48, 64, 3]
    assert meta["schema_version"] == 1

    # Steps.
    with (sealed / "steps.jsonl").open() as f:
        steps = [json.loads(line) for line in f if line.strip()]
    assert len(steps) == meta["num_steps"]

    first, last = steps[0], steps[-1]
    assert first["is_first"] is True and first["is_last"] is False
    assert last["is_last"] is True and last["is_terminal"] is True

    # Sum of per-step dx across non-terminal steps should equal the total
    # displacement we nudged the robot through (5 nudges × 1.5 mm = 7.5 mm).
    # Individual per-step dx may be 0 when the recorder captures two frames
    # between nudges; that's fine, the sum is what matters.
    total_dx = sum(s["action"]["ee_delta"][0] for s in steps[:-1])
    assert 5.0 < total_dx < 10.0, f"expected ~7.5mm total dx, got {total_dx:.2f}"

    # Each action must be non-negative in X (robot only moved in +X direction).
    for s in steps[:-1]:
        assert s["action"]["ee_delta"][0] >= 0.0

    # Last-step action must be zero.
    assert last["action"]["ee_delta"] == [0.0] * 6

    # State fields present.
    for s in steps:
        assert len(s["state"]["ee_pose"]) == 6
        assert len(s["state"]["joints"]) == 6
        assert "gripper_pos" in s["state"]

    # list_episodes sees it.
    eps = list_episodes(str(tmp_episodes))
    assert any(e["name"] == sealed.name for e in eps)


def test_cannot_start_twice(mock_ros, mock_camera, tmp_episodes):
    rec = EpisodeRecorder(mock_ros, mock_camera, str(tmp_episodes), rate_hz=10.0)
    rec.start("t")
    try:
        import pytest
        with pytest.raises(RuntimeError):
            rec.start("t2")
    finally:
        rec.stop()
