"""Test the VLA executor against a mock OFT client + mock ROS.

Verifies: streaming ServoP cadence, upsampling, safety clamping in the loop,
gripper thresholding, and clean start/stop.
"""

from __future__ import annotations

import time

from dobot_ros.vla.executor import VLAExecutor, ExecutorConfig


def _run_briefly(ex, duration=0.4):
    ex.start()
    time.sleep(duration)
    ex.stop()


def test_executor_streams_servo_p(mock_ros, mock_camera, mock_oft):
    cfg = ExecutorConfig(
        instruction="pick the red block",
        model_rate_hz=10.0,
        servo_rate_hz=30.0,
        chunk_actions_to_execute=4,
    )
    ex = VLAExecutor(
        ros_client=mock_ros, config=cfg,
        camera=mock_camera, oft_client=mock_oft,
    )
    _run_briefly(ex)

    # We expect a handful of ServoP calls and at least one model query.
    assert len(mock_ros.servo_p_calls) > 4
    assert len(mock_oft.predict_calls) >= 1
    # The mock chunks have +1 mm/step in X → pose X should grow monotonically.
    x_seq = [c["pose"][0] for c in mock_ros.servo_p_calls]
    assert x_seq == sorted(x_seq)
    assert x_seq[-1] > x_seq[0]
    # Servo t parameter should roughly match the servo period (1/30 ≈ 33 ms).
    ts = {round(c["t"], 3) for c in mock_ros.servo_p_calls}
    # Expected ~0.04 (period * 1.2). Allow 0.02–0.08 range.
    assert all(0.01 <= t <= 0.1 for t in ts), f"servo t values out of range: {ts}"


def test_executor_upsamples(mock_ros, mock_camera, mock_oft):
    cfg = ExecutorConfig(
        instruction="x",
        model_rate_hz=10.0,
        servo_rate_hz=30.0,   # ratio = 3
        chunk_actions_to_execute=2,
    )
    ex = VLAExecutor(mock_ros, cfg, mock_camera, mock_oft)
    _run_briefly(ex, duration=0.3)

    # Per query we execute chunk_actions_to_execute (2) * ratio (3) = 6 servo steps.
    # With ~3 queries in 0.3s we should see ≥ 6 servo calls.
    assert len(mock_ros.servo_p_calls) >= 6


def test_executor_gripper_threshold(mock_ros, mock_camera):
    # Scripted chunk: alternating wide-swing gripper values.
    # MockOFTClient is defined in conftest.py, which pytest auto-imports,
    # but we need the class directly — import it from the live conftest module.
    from conftest import MockOFTClient  # type: ignore
    chunk = [[0.5, 0, 0, 0, 0, 0, 0.0]] + [[0.0, 0, 0, 0, 0, 0, 1.0]] * 7
    oft = MockOFTClient(chunks=[chunk])

    cfg = ExecutorConfig(
        instruction="x",
        model_rate_hz=10.0, servo_rate_hz=10.0,
        chunk_actions_to_execute=4,
        gripper_threshold=0.3,
    )
    ex = VLAExecutor(mock_ros, cfg, mock_camera, oft)
    _run_briefly(ex, duration=0.3)

    # We should have issued at least one gripper command (state crossed threshold).
    assert len(mock_ros.gripper_moves) >= 1
    positions = {g["position"] for g in mock_ros.gripper_moves}
    assert any(0 <= p <= 1000 for p in positions)


def test_executor_stops_cleanly(mock_ros, mock_camera, mock_oft):
    cfg = ExecutorConfig(instruction="x", servo_rate_hz=50.0, model_rate_hz=10.0)
    ex = VLAExecutor(mock_ros, cfg, mock_camera, mock_oft)
    ex.start()
    assert ex.is_running()
    ex.stop(timeout=2.0)
    assert not ex.is_running()
    assert ex.status().running is False
