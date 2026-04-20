"""Fixtures for SpaceMouse tests.

Provides a FakeEvdev module that can be monkeypatched in place of the real
`evdev` import. Also re-exposes MockRosClient from tests/vla/conftest.py so
integration tests can reuse it.
"""
from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Optional

import pytest

# Expose MockRosClient from VLA conftest without requiring package markers.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "vla"))
from conftest import MockRosClient  # noqa: E402  (re-used MockRosClient)


# ── Fake evdev ─────────────────────────────────────────────────────────
# Minimal stand-in for the python-evdev interface we use: list_devices(),
# InputDevice with .info, .absinfo(), .path, .fd, .read_loop(), .close(),
# plus an ecodes namespace.

@dataclass
class FakeInfo:
    vendor: int
    product: int


@dataclass
class FakeAbsInfo:
    min: int
    max: int


@dataclass
class FakeEvent:
    """Mimics evdev.InputEvent (type, code, value, timestamp)."""
    type: int
    code: int
    value: int
    sec: int = 0
    usec: int = 0


class FakeInputDevice:
    def __init__(
        self,
        path: str,
        vendor: int,
        product: int,
        abs_max: int = 350,
        scripted_events: Optional[List[FakeEvent]] = None,
    ):
        self.path = path
        self.info = FakeInfo(vendor=vendor, product=product)
        self._abs_max = abs_max
        self._events: List[FakeEvent] = list(scripted_events or [])
        self._closed = False
        self.fd = 100
        self._raise_after_events = False

    def absinfo(self, code: int) -> FakeAbsInfo:
        return FakeAbsInfo(min=-self._abs_max, max=self._abs_max)

    def capabilities(self, verbose: bool = False, absinfo: bool = True):
        return {}

    def read_loop(self) -> Iterable[FakeEvent]:
        for ev in self._events:
            if self._closed:
                return
            yield ev
        if self._raise_after_events:
            raise OSError("fake device disconnected")

    def close(self):
        self._closed = True


class _FakeEcodes:
    ABS_X = 0
    ABS_Y = 1
    ABS_Z = 2
    ABS_RX = 3
    ABS_RY = 4
    ABS_RZ = 5
    BTN_0 = 100
    BTN_1 = 101
    EV_ABS = 3
    EV_KEY = 1


class FakeEvdev:
    ecodes = _FakeEcodes()

    def __init__(self, devices: Optional[List[FakeInputDevice]] = None):
        self._devices = {d.path: d for d in (devices or [])}

    def list_devices(self) -> List[str]:
        return list(self._devices.keys())

    def InputDevice(self, path: str) -> FakeInputDevice:
        if path not in self._devices:
            raise FileNotFoundError(path)
        return self._devices[path]

    def add(self, device: FakeInputDevice):
        self._devices[device.path] = device


@pytest.fixture
def fake_evdev_empty():
    return FakeEvdev(devices=[])


@pytest.fixture
def fake_spacemouse_device():
    from dobot_ros.spacemouse.hid_state import VENDOR_ID, PRODUCT_ID
    return FakeInputDevice(
        path="/dev/input/event21",
        vendor=VENDOR_ID,
        product=PRODUCT_ID,
    )


@pytest.fixture
def fake_evdev_with_spacemouse(fake_spacemouse_device):
    return FakeEvdev(devices=[fake_spacemouse_device])


@pytest.fixture
def fake_evdev_with_other_device():
    return FakeEvdev(devices=[
        FakeInputDevice(path="/dev/input/event8", vendor=0x05AC, product=0x024F),
    ])


@pytest.fixture
def mock_ros():
    return MockRosClient()


# ── Mock ServoTester ──────────────────────────────────────────────────
class MockServoTester:
    """Records arm/stop/target calls; mimics just the interface SpaceMouseReader uses."""

    def __init__(self, anchor=None):
        self._running = False
        self._anchor = list(anchor or [400.0, 0.0, 300.0, 180.0, 0.0, 0.0])
        self.start_calls = 0
        self.stop_calls = 0
        self.emergency_stop_calls = 0
        self.target_offsets: list = []
        # Rate-control API (set_target_velocity). Tests inspect these
        # lists to verify the reader emits velocity rather than position.
        self.target_velocities: list = []
        # Optional commanded-offset override; reader reads this back via
        # status().last_target_offset. Tests can set it to simulate the
        # tester lagging behind the commanded velocity.
        self._commanded_offset = [0.0] * 6

    def is_running(self) -> bool:
        return self._running

    def start(self):
        self._running = True
        self.start_calls += 1
        anchor_list = list(self._anchor)

        class _Status:
            anchor_pose = anchor_list
        return _Status()

    def stop(self, timeout=2.0, call_robot_stop=False):
        self._running = False
        self.stop_calls += 1

    def emergency_stop(self):
        self._running = False
        self.emergency_stop_calls += 1

    def set_target_offset(self, offset):
        self.target_offsets.append(list(offset))
        self._commanded_offset = list(offset)
        obj = type("S", (), {})()
        obj.last_target_offset = list(self._commanded_offset)
        obj.anchor_pose = list(self._anchor)
        return obj

    def set_target_velocity(self, velocity):
        """Rate-control API — mirrors the real tester. Integrates the
        velocity into ``_commanded_offset`` using a nominal 50 Hz dt so
        tests that want to inspect read-back behaviour get a sensible
        value. Tests that don't care can ignore ``_commanded_offset``."""
        self.target_velocities.append(list(velocity))
        dt = 1.0 / 50.0
        for i in range(6):
            self._commanded_offset[i] += velocity[i] * dt
        obj = type("S", (), {})()
        obj.last_target_offset = list(self._commanded_offset)
        obj.anchor_pose = list(self._anchor)
        return obj

    def status(self):
        obj = type("S", (), {})()
        obj.last_target_offset = list(self._commanded_offset)
        obj.anchor_pose = list(self._anchor)
        return obj

    @property
    def anchor(self):
        return list(self._anchor)


@pytest.fixture
def mock_servo():
    return MockServoTester()
