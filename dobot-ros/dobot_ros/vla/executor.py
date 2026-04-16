"""Closed-loop VLA executor.

Runs the control loop:
  capture frame → query OFT → receive action chunk →
    interpolate to higher rate → clamp to safety limits →
      stream ServoP + gripper actions

Designed to run in a background thread alongside the web dashboard.
"""

from __future__ import annotations

import base64
import logging
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional

from dobot_ros.vla.client import OFTClient, VLAPrediction
from dobot_ros.vla.recorder import CameraFrameSource
from dobot_ros.vla.safety import SafetyLimits, apply as safety_apply


log = logging.getLogger(__name__)


# ── Config ──────────────────────────────────────────────────────────────
@dataclass
class ExecutorConfig:
    instruction: str
    server_url: str = "http://gpurbr2:7071"

    # Model-side rate (how the action chunk maps to time). OFT chunks are
    # typically at 10 Hz model rate.
    model_rate_hz: float = 10.0

    # Servo streaming rate. We upsample the model output to smooth motion.
    servo_rate_hz: float = 30.0

    # How many actions of each chunk to execute before re-querying. Smaller =
    # more reactive, more compute. chunk_actions_to_execute < chunk_size means
    # receding-horizon.
    chunk_actions_to_execute: int = 4

    # ServoP tuning.
    servo_aheadtime: float = 50.0
    servo_gain: float = 500.0

    # Gripper threshold to issue a new action (0..1 normalized distance).
    gripper_threshold: float = 0.1

    # Safety.
    table_plane_path: Optional[str] = None
    unnorm_key: Optional[str] = None


@dataclass
class ExecutorStatus:
    running: bool = False
    step: int = 0
    last_latency_ms: float = 0.0
    last_error: Optional[str] = None
    last_chunk_size: int = 0
    clamps: int = 0
    recent_clamp_reasons: List[str] = field(default_factory=list)
    model: str = ""


# ── Executor ────────────────────────────────────────────────────────────
class VLAExecutor:
    def __init__(
        self,
        ros_client,
        config: ExecutorConfig,
        camera: Optional[CameraFrameSource] = None,
        oft_client: Optional[OFTClient] = None,
    ):
        self.ros = ros_client
        self.config = config
        self.camera = camera or CameraFrameSource()
        self.client = oft_client or OFTClient(base_url=config.server_url)

        self.limits = (
            SafetyLimits.from_table_plane(Path(config.table_plane_path))
            if config.table_plane_path else SafetyLimits()
        )

        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._status = ExecutorStatus()
        self._last_gripper_norm: Optional[float] = None

    # ── Lifecycle ───────────────────────────────────────────────
    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self):
        with self._lock:
            if self.is_running():
                raise RuntimeError("executor already running")
            self._stop.clear()
            self._status = ExecutorStatus(running=True)
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()
            log.info("VLA executor started: %r", self.config.instruction)

    def stop(self, timeout: float = 3.0):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=timeout)
        with self._lock:
            self._status.running = False
        log.info("VLA executor stopped")

    def status(self) -> ExecutorStatus:
        with self._lock:
            # Return a shallow copy so callers can't mutate it.
            return ExecutorStatus(**self._status.__dict__)

    # ── Main loop ───────────────────────────────────────────────
    def _run(self):
        cfg = self.config
        period = 1.0 / cfg.servo_rate_hz

        try:
            while not self._stop.is_set():
                # Query the model.
                pred = self._query_model()
                if pred is None:
                    time.sleep(0.1)
                    continue

                chunk = pred.actions[: cfg.chunk_actions_to_execute]
                if not chunk:
                    continue

                # Validate action dimensions — model must emit 7-D actions.
                bad = [i for i, a in enumerate(chunk) if len(a) < 7]
                if bad:
                    log.warning("model returned wrong action dims at indices %s (expected 7-D); skipping chunk", bad)
                    with self._lock:
                        self._status.last_error = f"bad action dims: {[len(chunk[i]) for i in bad]}"
                    continue

                # Upsample chunk to servo rate. Each model action represents
                # 1/model_rate_hz seconds; we divide it into M sub-steps of
                # 1/servo_rate_hz each with linear interpolation.
                ratio = max(1, int(round(cfg.servo_rate_hz / cfg.model_rate_hz)))
                interpolated = self._upsample(chunk, ratio)

                # Execute each interpolated action via ServoP.
                for action in interpolated:
                    if self._stop.is_set():
                        break
                    self._execute_one(action, period)

                with self._lock:
                    self._status.last_chunk_size = len(chunk)

        except Exception as e:
            log.exception("executor loop crashed: %s", e)
            with self._lock:
                self._status.last_error = str(e)
                self._status.running = False
        finally:
            # SAFETY: always halt the robot when the executor exits (normal or crash).
            # Without this, the robot continues toward the last ServoP target unsupervised.
            try:
                self.ros.stop()
            except Exception as e:
                log.warning("ros.stop() during executor shutdown failed: %s", e)
            with self._lock:
                self._status.running = False

    def _query_model(self) -> Optional[VLAPrediction]:
        try:
            jpeg = self.camera.get_jpeg()
            img_b64 = base64.b64encode(jpeg).decode("ascii")
            pose = self.ros.get_cartesian_pose()
            g_pos = self.ros.gripper_get_position() or 0
            state = [*pose, g_pos / 1000.0]
            pred = self.client.predict(
                image_b64=img_b64,
                instruction=self.config.instruction,
                state=state,
                unnorm_key=self.config.unnorm_key,
            )
            with self._lock:
                self._status.last_latency_ms = pred.latency_ms
                self._status.last_error = None
                self._status.model = pred.model
            return pred
        except Exception as e:
            log.warning("predict failed: %s", e)
            with self._lock:
                self._status.last_error = f"predict: {e}"
            return None

    def _upsample(self, chunk: List[List[float]], ratio: int) -> List[List[float]]:
        """Linearly interpolate a list of 7-D actions into `ratio` sub-steps each.

        Each original action is a full delta over one model step. We divide it
        into `ratio` smaller sub-deltas of equal magnitude so that summing them
        reproduces the original delta.
        """
        if ratio <= 1:
            return chunk
        out: List[List[float]] = []
        for a in chunk:
            # 6-D delta is split evenly; gripper target stays (absolute).
            sub_delta = [a[i] / ratio for i in range(6)]
            for k in range(ratio):
                # Interleave gripper: hold current target until last sub-step
                # of each chunk element (so gripper commands fire once per
                # model step, not per sub-step).
                gripper = a[6] if k == ratio - 1 else None
                out.append([*sub_delta, gripper])
        return out

    def _execute_one(self, action: List[float], period: float):
        """Apply one interpolated action: safety → ServoP → maybe gripper."""
        t0 = time.perf_counter()

        # Action layout: [dx, dy, dz, drx, dry, drz, gripper_target_or_None]
        delta = list(action[:6])
        gripper = action[6]

        current_pose = self.ros.get_cartesian_pose()
        clamp = safety_apply(current_pose, delta, self.limits)

        if clamp.clamped:
            with self._lock:
                self._status.clamps += 1
                # Keep a rolling tail of reasons (last 10).
                self._status.recent_clamp_reasons = (
                    self._status.recent_clamp_reasons + clamp.reasons
                )[-10:]

        # Stream pose.
        try:
            self.ros.servo_p(
                clamp.pose,
                t=period * 1.2,  # slightly longer than period so interpolator overlaps
                aheadtime=self.config.servo_aheadtime,
                gain=self.config.servo_gain,
            )
        except Exception as e:
            log.warning("servo_p failed: %s", e)
            with self._lock:
                self._status.last_error = f"servo_p: {e}"

        # Gripper (fire-and-forget; only if threshold crossed).
        if gripper is not None:
            last = self._last_gripper_norm
            if last is None or abs(gripper - last) >= self.config.gripper_threshold:
                target = int(max(0, min(1000, round(gripper * 1000))))
                try:
                    self.ros.gripper_move(target, wait=False)
                    self._last_gripper_norm = gripper
                except Exception as e:
                    log.warning("gripper_move failed: %s", e)

        with self._lock:
            self._status.step += 1

        # Pace the loop.
        elapsed = time.perf_counter() - t0
        if elapsed < period:
            self._stop.wait(timeout=period - elapsed)
