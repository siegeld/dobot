"""Episode recorder — captures synced (RGB, state, action) at a fixed rate.

Architecture:
- Caller supplies a DobotRosClient (for state) and a CameraFrameSource (for RGB).
- Caller calls `start(instruction)` to open a new episode directory.
- A background thread ticks at `rate_hz` and writes one frame + one step.
- Caller calls `stop()` to finalize (computes per-step actions from the sequence
  of states) and seal the episode.
"""

from __future__ import annotations

import json
import logging
import threading
import time
from dataclasses import asdict
from pathlib import Path
from typing import Callable, List, Optional

from dobot_ros.vla.types import Step, Action, State, EpisodeMeta


log = logging.getLogger(__name__)


# ── Camera source abstraction ───────────────────────────────────────────
class CameraFrameSource:
    """Returns JPEG bytes of the current RGB frame.

    Default implementation polls the existing camera server's /api/frame/color
    endpoint. Swap for a ROS image-topic source later if needed.
    """

    def __init__(self, camera_url: str = "http://10.11.6.65:8080", timeout: float = 2.0):
        self.base_url = camera_url.rstrip("/")
        self.timeout = timeout

    def get_jpeg(self) -> bytes:
        import requests
        r = requests.get(f"{self.base_url}/api/frame/color", timeout=self.timeout)
        r.raise_for_status()
        return r.content


# ── Recorder ────────────────────────────────────────────────────────────
def _slugify(s: str, maxlen: int = 40) -> str:
    out = []
    for c in s.strip().lower():
        if c.isalnum():
            out.append(c)
        elif c in " -_":
            out.append("_")
    slug = "".join(out).strip("_")
    return (slug or "episode")[:maxlen]


class EpisodeRecorder:
    def __init__(
        self,
        ros_client,
        camera: Optional[CameraFrameSource] = None,
        episodes_dir: str = "/data/episodes",
        rate_hz: float = 10.0,
    ):
        self.ros = ros_client
        self.camera = camera or CameraFrameSource()
        self.episodes_dir = Path(episodes_dir)
        self.rate_hz = rate_hz

        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._lock = threading.Lock()

        # Active-episode state.
        self._episode_dir: Optional[Path] = None
        self._meta: Optional[EpisodeMeta] = None
        self._states: List[State] = []  # accumulate; actions computed at stop()
        self._step_times: List[float] = []
        self._image_paths: List[str] = []

    # ── Lifecycle ───────────────────────────────────────────────────
    def is_recording(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self, instruction: str, episode_name: Optional[str] = None) -> Path:
        """Begin a new episode. Returns the episode directory."""
        with self._lock:
            if self.is_recording():
                raise RuntimeError("already recording; call stop() first")

            ts = int(time.time())
            slug = episode_name or _slugify(instruction)
            ep_dir = self.episodes_dir / f"{slug}_{ts}"
            (ep_dir / "frames").mkdir(parents=True, exist_ok=True)

            self._episode_dir = ep_dir
            self._meta = EpisodeMeta(
                instruction=instruction,
                started_at=time.time(),
                rate_hz=self.rate_hz,
                camera_url=self.camera.base_url,
            )
            self._states = []
            self._step_times = []
            self._image_paths = []
            self._stop.clear()

            self._thread = threading.Thread(target=self._loop, daemon=True)
            self._thread.start()
            log.info("Recording started: %s (instruction=%r)", ep_dir, instruction)
            return ep_dir

    def stop(self) -> Optional[Path]:
        """Finalize and seal the episode. Returns the sealed directory."""
        with self._lock:
            if not self.is_recording():
                log.warning("stop() called but no episode is recording")
                return None
            self._stop.set()

        self._thread.join(timeout=5.0)

        with self._lock:
            if self._meta is None or self._episode_dir is None:
                return None
            self._meta.ended_at = time.time()
            self._meta.num_steps = len(self._states)
            self._write_steps()
            self._write_meta()
            ep_dir = self._episode_dir
            self._episode_dir = None
            self._meta = None
            log.info("Recording stopped: %s (%d steps)", ep_dir, len(self._states))
            self._states = []
            self._step_times = []
            self._image_paths = []
            return ep_dir

    # ── Main loop ───────────────────────────────────────────────────
    def _loop(self):
        period = 1.0 / self.rate_hz
        next_t = time.time()
        idx = 0
        while not self._stop.is_set():
            start_t = time.time()
            try:
                self._capture_step(idx)
                idx += 1
            except Exception as e:
                log.exception("recorder step %d failed: %s", idx, e)

            next_t += period
            sleep = next_t - time.time()
            if sleep > 0:
                self._stop.wait(timeout=sleep)
            else:
                # Fell behind — skip catching up, just reset schedule.
                next_t = time.time()

    def _capture_step(self, idx: int):
        # State snapshot.
        pose = self.ros.get_cartesian_pose()
        joints = self.ros.get_joint_angles()
        g_pos = self.ros.gripper_get_position() or 0
        g_state = self.ros.gripper_get_state() or 0

        state = State(
            ee_pose=list(pose),
            joints=list(joints),
            gripper_pos=int(g_pos),
            gripper_state=int(g_state),
        )

        # Image.
        jpeg = self.camera.get_jpeg()
        rel_path = f"frames/{idx:06d}.jpg"
        abs_path = self._episode_dir / rel_path
        abs_path.write_bytes(jpeg)

        if idx == 0 and not self._meta.image_shape:
            try:
                from PIL import Image
                import io
                with Image.open(io.BytesIO(jpeg)) as im:
                    w, h = im.size
                    self._meta.image_shape = [h, w, 3]
            except Exception:
                pass

        with self._lock:
            self._states.append(state)
            self._step_times.append(time.time())
            self._image_paths.append(rel_path)

    # ── Finalization ────────────────────────────────────────────────
    def _compute_actions(self) -> List[Action]:
        """Compute per-step actions as (state[i+1] - state[i])."""
        actions: List[Action] = []
        for i in range(len(self._states)):
            if i + 1 >= len(self._states):
                actions.append(Action.zero())
                continue
            cur = self._states[i]
            nxt = self._states[i + 1]
            ee_delta = [nxt.ee_pose[k] - cur.ee_pose[k] for k in range(6)]
            actions.append(Action(ee_delta=ee_delta, gripper_target=nxt.gripper_pos))
        return actions

    def _write_steps(self):
        actions = self._compute_actions()
        path = self._episode_dir / "steps.jsonl"
        with path.open("w") as f:
            n = len(self._states)
            for i in range(n):
                step = Step(
                    step=i,
                    t=self._step_times[i],
                    image_path=self._image_paths[i],
                    state=self._states[i],
                    action=actions[i],
                    is_first=(i == 0),
                    is_last=(i == n - 1),
                    is_terminal=(i == n - 1),
                )
                f.write(json.dumps(step.to_json()) + "\n")

    def _write_meta(self):
        path = self._episode_dir / "meta.json"
        path.write_text(json.dumps(asdict(self._meta), indent=2))


# ── Helpers for list / load ─────────────────────────────────────────────
def list_episodes(episodes_dir: str = "/data/episodes") -> List[dict]:
    """List recorded episodes with their metadata, newest first."""
    root = Path(episodes_dir)
    if not root.exists():
        return []
    out = []
    for ep in sorted(root.iterdir(), reverse=True):
        if not ep.is_dir():
            continue
        meta_path = ep / "meta.json"
        if not meta_path.exists():
            continue
        try:
            meta = json.loads(meta_path.read_text())
        except json.JSONDecodeError:
            continue
        out.append({"name": ep.name, "path": str(ep), **meta})
    return out
