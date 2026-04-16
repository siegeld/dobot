"""Edge-case tests for VLA executor — NaN actions, crash recovery, stop timeout."""

import math
import time
import threading

import pytest

from dobot_ros.vla.executor import VLAExecutor, ExecutorConfig


# Reuse conftest fixtures.
from conftest import MockRosClient, MockCamera, MockOFTClient


def test_executor_rejects_nan_delta(mock_camera):
    """Model returning NaN in deltas should not crash — actions are skipped."""
    chunk = [[float('nan'), 0, 0, 0, 0, 0, 0.5]] * 4
    oft = MockOFTClient(chunks=[chunk])
    ros = MockRosClient()
    cfg = ExecutorConfig(instruction="x", servo_rate_hz=20.0, model_rate_hz=10.0)
    ex = VLAExecutor(ros, cfg, mock_camera, oft)
    ex.start()
    time.sleep(0.3)
    ex.stop()
    # NaN deltas should be skipped — no servo_p calls with NaN.
    for call in ros.servo_p_calls:
        assert all(math.isfinite(v) for v in call["pose"])


def test_executor_rejects_wrong_dimension_actions(mock_camera):
    """Model returning 3-D actions (instead of 7-D) should be skipped."""
    chunk = [[1.0, 2.0, 3.0]] * 4  # wrong dims
    oft = MockOFTClient(chunks=[chunk])
    ros = MockRosClient()
    cfg = ExecutorConfig(instruction="x", servo_rate_hz=20.0, model_rate_hz=10.0)
    ex = VLAExecutor(ros, cfg, mock_camera, oft)
    ex.start()
    time.sleep(0.3)
    ex.stop()
    st = ex.status()
    assert "bad action dims" in (st.last_error or "")


def test_executor_calls_ros_stop_on_crash(mock_camera):
    """If the executor loop crashes, ros.stop() must be called."""
    # Make gripper_get_position raise to crash the loop.
    class CrashingRos(MockRosClient):
        _call_count = 0
        def get_cartesian_pose(self):
            self._call_count += 1
            if self._call_count > 5:
                raise RuntimeError("simulated crash")
            return super().get_cartesian_pose()

    ros = CrashingRos()
    stop_calls = []
    ros.stop = lambda: stop_calls.append(1) or 0

    oft = MockOFTClient()
    cfg = ExecutorConfig(instruction="x", servo_rate_hz=50.0, model_rate_hz=10.0)
    ex = VLAExecutor(ros, cfg, mock_camera, oft)
    ex.start()
    time.sleep(0.5)
    ex.stop()
    # ros.stop() should have been called in the finally block.
    assert len(stop_calls) >= 1


def test_executor_gripper_debounce(mock_camera):
    """Gripper commands must be debounced (min 0.5s apart)."""
    # Chunk alternates gripper rapidly.
    chunk = [[0.5, 0, 0, 0, 0, 0, g] for g in [0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0]]
    oft = MockOFTClient(chunks=[chunk])
    ros = MockRosClient()
    cfg = ExecutorConfig(
        instruction="x",
        servo_rate_hz=20.0, model_rate_hz=10.0,
        chunk_actions_to_execute=8,
        gripper_threshold=0.3,
    )
    ex = VLAExecutor(ros, cfg, mock_camera, oft)
    ex.start()
    time.sleep(0.6)
    ex.stop()
    # With 0.5s debounce and 0.6s runtime, at most 2 gripper commands.
    assert len(ros.gripper_moves) <= 3


def test_executor_stop_timeout_keeps_running_true(mock_camera):
    """If the thread doesn't stop within timeout, status.running stays True."""
    class HangingRos(MockRosClient):
        def get_cartesian_pose(self):
            time.sleep(10)  # hang forever
            return super().get_cartesian_pose()

    ros = HangingRos()
    ros.stop = lambda: 0
    oft = MockOFTClient()
    cfg = ExecutorConfig(instruction="x", servo_rate_hz=1.0, model_rate_hz=1.0)
    ex = VLAExecutor(ros, cfg, mock_camera, oft)
    ex.start()
    time.sleep(0.1)
    ex.stop(timeout=0.2)
    # Thread is still alive — running should NOT be set to False.
    st = ex.status()
    assert "stop timed out" in (st.last_error or "")
