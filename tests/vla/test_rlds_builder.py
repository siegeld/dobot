"""Test the pre-TFDS standalone reader functions of the RLDS builder.

TFDS itself is heavy and skipped in CI; the shape-preserving reader functions
are pure-python and can be exercised here.
"""

from __future__ import annotations

import json
import time
from pathlib import Path

import numpy as np

from dobot_ros.vla.recorder import EpisodeRecorder
from dobot_ros.vla.rlds_builder import load_episode, step_to_rlds_dict, iter_episodes


def _record_tiny_episode(mock_ros, mock_camera, tmp_episodes, instruction="do x"):
    rec = EpisodeRecorder(mock_ros, mock_camera, str(tmp_episodes), rate_hz=20.0)
    rec.start(instruction)
    for _ in range(4):
        time.sleep(0.08)
        mock_ros._pose[0] += 2.0
    return rec.stop()


def test_load_episode_roundtrip(mock_ros, mock_camera, tmp_episodes):
    ep_dir = _record_tiny_episode(mock_ros, mock_camera, tmp_episodes)
    episode = load_episode(ep_dir)

    assert episode["meta"]["instruction"] == "do x"
    assert episode["meta"]["num_steps"] == len(episode["steps"])
    assert episode["steps"][0]["is_first"]
    assert episode["steps"][-1]["is_last"]
    assert episode["steps"][-1]["is_terminal"]


def test_step_to_rlds_dict_shapes(mock_ros, mock_camera, tmp_episodes):
    ep_dir = _record_tiny_episode(mock_ros, mock_camera, tmp_episodes)
    episode = load_episode(ep_dir)

    step = episode["steps"][0]
    rlds = step_to_rlds_dict(step, ep_dir, "do x")

    assert isinstance(rlds["state"], np.ndarray)
    assert rlds["state"].shape == (7,)
    assert rlds["state"].dtype == np.float32
    assert isinstance(rlds["action"], np.ndarray)
    assert rlds["action"].shape == (7,)
    assert rlds["action"].dtype == np.float32

    assert Path(rlds["image_path"]).exists()
    assert rlds["language_instruction"] == "do x"
    assert isinstance(rlds["is_first"], bool)
    assert isinstance(rlds["reward"], np.floating)


def test_iter_episodes_skips_malformed(mock_ros, mock_camera, tmp_episodes):
    _record_tiny_episode(mock_ros, mock_camera, tmp_episodes, instruction="a")
    _record_tiny_episode(mock_ros, mock_camera, tmp_episodes, instruction="b")
    # A stray directory without meta.json/steps.jsonl must be ignored.
    (tmp_episodes / "not_an_episode").mkdir()

    names = [n for n, _ in iter_episodes(tmp_episodes)]
    assert len(names) == 2
    assert "not_an_episode" not in names


def test_gripper_normalization(mock_ros, mock_camera, tmp_episodes):
    mock_ros._gripper_pos = 500
    ep_dir = _record_tiny_episode(mock_ros, mock_camera, tmp_episodes)
    episode = load_episode(ep_dir)
    rlds = step_to_rlds_dict(episode["steps"][0], ep_dir, "x")
    # Gripper is in slot 6 and normalized to [0, 1].
    assert 0.0 <= rlds["state"][6] <= 1.0
    assert abs(rlds["state"][6] - 0.5) < 1e-6
