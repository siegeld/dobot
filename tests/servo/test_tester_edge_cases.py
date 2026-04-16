"""Edge-case tests for servo tester — circuit breaker, crash recovery, config validation."""

import time

import pytest

from dobot_ros.servo.tester import ServoTester, ServoConfig
from dobot_ros.servo.patterns import build_pattern
from dobot_ros.vla.safety import SafetyLimits


# Reuse MockRosClient from VLA conftest.
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "vla"))
from conftest import MockRosClient


@pytest.fixture
def mock_ros():
    return MockRosClient()


def test_circuit_breaker_stops_after_consecutive_failures(mock_ros):
    """Tester should abort after MAX_CONSECUTIVE_FAILURES servo_p failures."""
    fail_count = [0]
    orig_servo_p = mock_ros.servo_p

    def failing_servo_p(*a, **kw):
        fail_count[0] += 1
        raise RuntimeError("simulated servo_p failure")

    mock_ros.servo_p = failing_servo_p
    tester = ServoTester(mock_ros, ServoConfig(servo_rate_hz=100.0))
    tester.start()
    time.sleep(0.5)
    # Should have stopped on its own due to circuit breaker.
    assert not tester.is_running()
    assert fail_count[0] >= 10
    st = tester.status()
    assert st.last_error and "servo_p" in st.last_error


def test_ros_stop_called_on_tester_crash(mock_ros):
    """Tester must call ros.stop() in finally when it crashes."""
    stop_calls = []
    mock_ros.stop = lambda: stop_calls.append(1) or 0

    class CrashingRos(MockRosClient):
        _n = 0
        def get_cartesian_pose(self):
            self._n += 1
            if self._n > 3:
                raise RuntimeError("boom")
            return super().get_cartesian_pose()

    ros = CrashingRos()
    ros.stop = lambda: stop_calls.append(1) or 0
    tester = ServoTester(ros, ServoConfig(servo_rate_hz=50.0))
    tester.start()
    time.sleep(0.3)
    tester.stop()
    assert len(stop_calls) >= 1


def test_update_config_rejects_bad_types(mock_ros):
    """String values for numeric configs should not crash the tester."""
    tester = ServoTester(mock_ros, ServoConfig(servo_rate_hz=30.0))
    tester.update_config(servo_rate_hz="not_a_number")
    # Should still be the old value.
    assert tester.config.servo_rate_hz == 30.0


def test_update_config_rejects_unknown_fields(mock_ros):
    """Unknown field names should be silently ignored."""
    tester = ServoTester(mock_ros, ServoConfig(servo_rate_hz=30.0))
    tester.update_config(bogus_field=999)
    assert not hasattr(tester.config, "bogus_field")


def test_csv_failure_sets_error_in_status(mock_ros):
    """If CSV log fails to open, status.last_error should be set."""
    tester = ServoTester(
        mock_ros, ServoConfig(servo_rate_hz=30.0),
        log_csv_path="/nonexistent/path/log.csv",
    )
    tester.start()
    time.sleep(0.15)
    tester.stop()
    st = tester.status()
    assert st.last_error and "CSV" in st.last_error


def test_pattern_amplitude_clamped():
    """build_pattern should clamp amplitudes to 200mm max."""
    pat = build_pattern("circle", {"radius_mm": 9999})
    assert pat.radius_mm <= 200.0


def test_pattern_negative_amplitude_abs():
    """Negative amplitudes should be abs()'d."""
    pat = build_pattern("sine", {"amplitude": -150, "axis": "x", "freq_hz": 1.0})
    assert pat.amplitude == 150.0


def test_pattern_period_zero_clamped():
    """period_s=0 should be clamped to prevent division by zero."""
    pat = build_pattern("circle", {"radius_mm": 10, "period_s": 0})
    assert pat.period_s >= 0.1
    # Should not raise on target_at.
    offset = pat.target_at(1.0)
    assert len(offset) == 6


def test_pattern_invalid_axis_rejected():
    """Invalid axis should raise ValueError."""
    with pytest.raises(ValueError, match="invalid axis"):
        build_pattern("square", {"axis": "bogus", "amplitude": 10, "period_s": 2})
