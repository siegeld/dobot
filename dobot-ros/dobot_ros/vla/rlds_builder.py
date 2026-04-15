"""Convert recorded episode directories into an RLDS-compatible TFDS dataset.

Uses the kpertsch/rlds_dataset_builder convention: define a single
DatasetBuilder with the required feature schema and a _generate_examples
generator that iterates over episode dirs.

This module is intentionally split so the conversion can run standalone
without the rest of the robot stack installed — only numpy + tensorflow-datasets.
"""

from __future__ import annotations

import json
import logging
from pathlib import Path
from typing import Dict, Iterator, List, Optional, Tuple

import numpy as np


log = logging.getLogger(__name__)


# ── Standalone reader ───────────────────────────────────────────────────
# These functions read our on-disk episode format without needing TFDS. They
# are also used by the tests and by any ad-hoc training script.

def load_episode(episode_dir: Path) -> Dict:
    """Load an episode directory into an in-memory dict.

    Returns:
        {
          "meta": {...},
          "steps": [ {step dict}, ... ],
          "episode_path": str,
        }
    """
    episode_dir = Path(episode_dir)
    meta = json.loads((episode_dir / "meta.json").read_text())
    steps: List[dict] = []
    with (episode_dir / "steps.jsonl").open("r") as f:
        for line in f:
            line = line.strip()
            if line:
                steps.append(json.loads(line))
    return {"meta": meta, "steps": steps, "episode_path": str(episode_dir)}


def step_to_rlds_dict(step: dict, episode_dir: Path, instruction: str) -> dict:
    """Convert one episode-format step into an RLDS-format step dict.

    Image is loaded lazily by the caller via the returned `image_path`.
    """
    state = step["state"]
    action = step["action"]

    state_vec = np.asarray([
        *state["ee_pose"],                 # 6
        state["gripper_pos"] / 1000.0,     # 1
    ], dtype=np.float32)

    action_vec = np.asarray([
        *action["ee_delta"],               # 6
        action["gripper_target"] / 1000.0, # 1
    ], dtype=np.float32)

    return {
        "image_path": str(episode_dir / step["image_path"]),
        "state": state_vec,
        "action": action_vec,
        "language_instruction": instruction,
        "is_first": bool(step.get("is_first", False)),
        "is_last": bool(step.get("is_last", False)),
        "is_terminal": bool(step.get("is_terminal", False)),
        "reward": np.float32(1.0 if step.get("is_terminal") else 0.0),
        "discount": np.float32(1.0),
    }


def iter_episodes(episodes_dir: Path) -> Iterator[Tuple[str, Path]]:
    """Yield (episode_id, path) for each episode directory."""
    root = Path(episodes_dir)
    if not root.exists():
        return
    for ep in sorted(root.iterdir()):
        if ep.is_dir() and (ep / "meta.json").exists() and (ep / "steps.jsonl").exists():
            yield ep.name, ep


# ── TFDS builder (import lazily — tensorflow is heavy) ─────────────────
def make_tfds_builder(name: str, episodes_dir: Path, image_shape=(360, 640, 3),
                     instruction_override: Optional[str] = None):
    """Return an instance of a DatasetBuilder for these episodes.

    tensorflow-datasets is imported lazily so this module can be loaded
    on hosts that don't have it installed. Call `builder.download_and_prepare()`
    to materialize the TFDS shards.

    `image_shape` is the target shape for decoded frames. Frames in episodes
    are stored as JPEGs; TFDS will decode and validate against this shape.
    """
    import tensorflow as tf
    import tensorflow_datasets as tfds

    H, W, C = image_shape

    class DobotEpisodes(tfds.core.GeneratorBasedBuilder):
        """RLDS dataset of Dobot CR5 manipulation episodes."""

        VERSION = tfds.core.Version("1.0.0")

        def _info(self) -> tfds.core.DatasetInfo:
            return self.dataset_info_from_configs(
                features=tfds.features.FeaturesDict({
                    "steps": tfds.features.Dataset({
                        "observation": tfds.features.FeaturesDict({
                            "image": tfds.features.Image(
                                shape=(H, W, C), dtype=np.uint8,
                                encoding_format="jpeg",
                                doc="Third-person RGB frame.",
                            ),
                            "state": tfds.features.Tensor(
                                shape=(7,), dtype=np.float32,
                                doc="[X,Y,Z,RX,RY,RZ,gripper_norm] — mm, deg, 0-1.",
                            ),
                        }),
                        "action": tfds.features.Tensor(
                            shape=(7,), dtype=np.float32,
                            doc="[dX,dY,dZ,dRX,dRY,dRZ,gripper_norm] — mm, deg, 0-1.",
                        ),
                        "discount": tfds.features.Scalar(dtype=np.float32),
                        "reward": tfds.features.Scalar(dtype=np.float32),
                        "is_first": tfds.features.Scalar(dtype=np.bool_),
                        "is_last": tfds.features.Scalar(dtype=np.bool_),
                        "is_terminal": tfds.features.Scalar(dtype=np.bool_),
                        "language_instruction": tfds.features.Text(),
                    }),
                    "episode_metadata": tfds.features.FeaturesDict({
                        "file_path": tfds.features.Text(),
                    }),
                }),
            )

        def _split_generators(self, dl_manager):
            return {"train": self._generate_examples(Path(episodes_dir))}

        def _generate_examples(self, path: Path):
            for ep_id, ep_path in iter_episodes(path):
                episode = load_episode(ep_path)
                instruction = instruction_override or episode["meta"].get("instruction", "")

                steps_out = []
                for step in episode["steps"]:
                    rlds_step = step_to_rlds_dict(step, ep_path, instruction)
                    steps_out.append({
                        "observation": {
                            "image": rlds_step["image_path"],   # TFDS loads from path
                            "state": rlds_step["state"],
                        },
                        "action": rlds_step["action"],
                        "discount": rlds_step["discount"],
                        "reward": rlds_step["reward"],
                        "is_first": rlds_step["is_first"],
                        "is_last": rlds_step["is_last"],
                        "is_terminal": rlds_step["is_terminal"],
                        "language_instruction": rlds_step["language_instruction"],
                    })

                yield ep_id, {
                    "steps": steps_out,
                    "episode_metadata": {"file_path": str(ep_path)},
                }

    DobotEpisodes.__name__ = name
    return DobotEpisodes(data_dir=str(Path(episodes_dir).parent / "rlds"))


# ── CLI entry (used by `dobot-ros vla convert-rlds`) ───────────────────
def convert(episodes_dir: str, dataset_name: str = "dobot_cr5_episodes",
            image_shape=(360, 640, 3), instruction_override: Optional[str] = None) -> str:
    """Convert on-disk episodes into a TFDS dataset. Returns the output dir."""
    builder = make_tfds_builder(
        name=dataset_name,
        episodes_dir=Path(episodes_dir),
        image_shape=image_shape,
        instruction_override=instruction_override,
    )
    builder.download_and_prepare()
    return str(builder.data_dir)
