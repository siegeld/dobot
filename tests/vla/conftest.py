"""Test fixtures for the VLA pipeline.

These tests never touch the real robot, real GPU, or real network. They use
mock ROS clients, mock cameras, and an in-process mock OFT server.
"""

from __future__ import annotations

# Make the mock classes importable as `from conftest import MockOFTClient`
# from within test files in this directory.
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))

import io
import json
import threading
import time
from pathlib import Path
from typing import List, Optional

import pytest
from PIL import Image


# ── Mock camera ─────────────────────────────────────────────────────────
class MockCamera:
    """Returns JPEG bytes of a generated gradient. Counts calls."""

    def __init__(self, width: int = 64, height: int = 48):
        self.width = width
        self.height = height
        self.calls = 0
        self.base_url = "mock://camera"

    def get_jpeg(self) -> bytes:
        self.calls += 1
        img = Image.new("RGB", (self.width, self.height))
        # Minimal gradient — just enough variety that JPEGs are distinct.
        px = img.load()
        shift = self.calls & 0xFF
        for y in range(self.height):
            for x in range(self.width):
                px[x, y] = ((x + shift) & 0xFF, (y + shift) & 0xFF, 128)
        buf = io.BytesIO()
        img.save(buf, format="JPEG", quality=75)
        return buf.getvalue()


# ── Mock ROS client ─────────────────────────────────────────────────────
class MockRosClient:
    """Stands in for DobotRosClient. Records every ServoP call for assertions."""

    def __init__(self, pose: Optional[List[float]] = None):
        self._pose = list(pose or [400.0, 0.0, 300.0, 180.0, 0.0, 0.0])
        self._joints = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
        self._gripper_pos = 500
        self._gripper_state = 1
        self.servo_p_calls: List[dict] = []
        self.servo_j_calls: List[dict] = []
        self.gripper_moves: List[dict] = []

    # State accessors used by recorder + executor.
    def get_cartesian_pose(self): return list(self._pose)
    def get_joint_angles(self): return list(self._joints)
    def gripper_get_position(self): return self._gripper_pos
    def gripper_get_state(self): return self._gripper_state

    # Commands.
    def start_drag(self): return 0
    def stop_drag(self): return 0

    def servo_p(self, pose, t=0.1, aheadtime=50.0, gain=500.0):
        self.servo_p_calls.append({
            "pose": list(pose), "t": t,
            "aheadtime": aheadtime, "gain": gain,
        })
        self._pose = list(pose)
        return 0

    def servo_j(self, joints, t=0.1, aheadtime=50.0, gain=500.0):
        self.servo_j_calls.append({
            "joints": list(joints), "t": t,
            "aheadtime": aheadtime, "gain": gain,
        })
        return 0

    def gripper_move(self, position, force=50, speed=50, wait=True, timeout=10.0):
        self.gripper_moves.append({"position": position, "wait": wait})
        self._gripper_pos = int(position)
        return 1


# ── Mock OFT client ─────────────────────────────────────────────────────
class MockOFTClient:
    """Returns scripted action chunks. Optionally records every predict() call."""

    def __init__(self, chunks: Optional[List[List[List[float]]]] = None):
        # Default script: small constant +X motion with gripper open.
        default_chunk = [[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]] * 8
        self._script = chunks or [default_chunk]
        self._idx = 0
        self.predict_calls: List[dict] = []

    def health(self):
        return {"status": "ok", "mode": "mock-test", "model": "scripted"}

    def predict(self, image_b64, instruction, state=None, wrist_image_b64=None, unnorm_key=None):
        self.predict_calls.append({
            "instruction": instruction, "state": state, "unnorm_key": unnorm_key,
        })
        chunk = self._script[self._idx % len(self._script)]
        self._idx += 1
        from dobot_ros.vla.client import VLAPrediction
        return VLAPrediction(actions=chunk, chunk_size=len(chunk),
                             latency_ms=1.0, model="scripted")


# ── Fixtures ────────────────────────────────────────────────────────────
@pytest.fixture
def mock_camera():
    return MockCamera()


@pytest.fixture
def mock_ros():
    return MockRosClient()


@pytest.fixture
def mock_oft():
    return MockOFTClient()


@pytest.fixture
def tmp_episodes(tmp_path):
    d = tmp_path / "episodes"
    d.mkdir()
    return d
