# SpaceMouse Pendant Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Ship a Bluetooth SpaceMouse Wireless BT pendant that jogs the Dobot CR5 in Cartesian rate-control mode and drives the gripper from the two puck buttons, with a web UI that doubles as a diagnostic HID echo. Fully additive — no existing motion path is modified.

**Architecture:** A `SpaceMouseReader` class (two daemon threads) lives inside the existing `dobot-ros` web-dashboard process (`dobot-web` container). One thread blocks on `evdev.read_loop()` to decode axes and buttons; the other runs a 50 Hz integration tick that applies deadband + velocity scale, integrates `target_offset += v*dt`, clamps, and — when armed — hands the offset to the existing `ServoTester.set_target_offset(...)`. Buttons debounce and call `ros_client.gripper_move(0|1000)`. HID state streams to the browser over a new `/ws/spacemouse` WebSocket. Device is discovered by vendor/product (`0x256F:0xC63A`) from a read-only bind-mount of `/dev/input/`. Safety: explicit arm, 30 s idle auto-disarm, excursion clamp, workspace clamp (via ServoTester), Esc/E-stop button calls `ros.stop()`.

**Tech Stack:** Python 3 (inside the existing ROS 2 Jazzy container), `python3-evdev` (Debian-packaged), FastAPI + WebSockets (already in `web/server.py`), vanilla HTML/JS/CSS (matching existing `web/static/` style), pytest with mocked evdev.

**Spec:** `docs/superpowers/specs/2026-04-17-spacemouse-pendant-design.md`

---

## File structure

**New module** (`dobot-ros/dobot_ros/spacemouse/`):

| File | Purpose |
|---|---|
| `__init__.py` | Re-exports `SpaceMouseReader`, `HidState`, `SpaceMouseSettings`. |
| `hid_state.py` | `HidState` dataclass (per-tick snapshot for WS), `SpaceMouseSettings` dataclass (persisted tuning), `DeviceStatus` enum, constants (VENDOR, PRODUCT, DEFAULTS). |
| `device.py` | `find_device() -> Optional[str]` (scan `evdev.list_devices()` by vendor/product), `open_device(path) -> evdev.InputDevice`, `read_battery_pct(path_or_device) -> Optional[int]`. Thin wrappers so the reader is easy to test with stubs. |
| `reader.py` | `SpaceMouseReader` — two-thread class that owns the device fd, maintains the latest axis/button state, runs a 50 Hz integration tick, calls into an injected `ServoTester`-like callback and gripper callback, and publishes `HidState` snapshots. Takes explicit `time_fn` and `sleep_fn` for testability. |

**New tests** (`tests/spacemouse/`):

| File | Purpose |
|---|---|
| `__init__.py` | Empty package marker. |
| `conftest.py` | `FakeEvdevDevice` (scripted event stream, fake `absinfo`), fake clock, `MockRosClient` re-used from `tests/vla/conftest.py`, a `MockServoTester` recording `set_target_offset` calls. |
| `test_hid_state.py` | `HidState` / `SpaceMouseSettings` round-trip; DEFAULTS sanity. |
| `test_device.py` | `find_device()` picks correct path; `read_battery_pct()` parses sysfs fixtures. |
| `test_reader_decode.py` | Axis normalization by `absinfo.max`; deadband cut; sign map; button edge/debounce. |
| `test_reader_integration.py` | Full tick loop with fake clock: integration, excursion clamp, armed vs disarmed gating, idle auto-disarm, disconnect path. |

**New frontend** (`dobot-ros/dobot_ros/web/static/`):

| File | Purpose |
|---|---|
| `spacemouse.html` | Full SpaceMouse page (status bar, big ARM + E-STOP, anchor/offset readout, live HID bars, buttons, collapsible settings). Uses existing bootstrap + dashboard.css for styling. |
| `js/spacemouse.js` | Page controller: opens `/ws/spacemouse`, renders bars + button glow, wires ARM/DISARM/E-STOP buttons, settings form POSTs to `/api/spacemouse/settings`, handles disconnect banner, binds `Esc` key. |
| `css/spacemouse.css` | Page-specific styles (axis bar component, button glow, banners). |

**New doc** (`docs/spacemouse-bringup.md`): operator manual smoke checklist.

**Modified:**

| File | Change |
|---|---|
| `dobot-ros/dobot_ros/web/server.py` | Add `spacemouse` settings group to `_SETTINGS_DEFAULTS`; add `_get_spacemouse_reader()` factory + subscribe to store; add `/api/spacemouse/{arm,disarm,estop,state,settings}` routes; add `@app.websocket("/ws/spacemouse")`; add `@app.get("/spacemouse")` FileResponse; extend `_acquire_motion` caller list to include `"spacemouse"`. |
| `dobot-ros/dobot_ros/web/static/index.html` | Add nav link to SpaceMouse page; embed small HID echo card on the servo tester section. |
| `docker-compose.yml` | Add `devices: - /dev/input:/dev/input:ro` and `group_add: - "${INPUT_GID:-104}"` to the base `dobot` service (all extends inherit). |
| `docker/Dockerfile` | Add `python3-evdev` to the `apt-get install` line. |
| `.env.example` | Add `INPUT_GID=104` with a discovery comment. (Create the file if it doesn't exist; `.env` is user-local.) |

**Untouched** (the additive guarantee):
`ros_client.py`, `servo/tester.py`, `servo/patterns.py`, `cli.py`, `shell.py`, `gripper_node.py`, `pick.py`, `strategies/*`, `vla/*`, `validation.py`, `driver.py`, `config.py`.

---

## Task 1: Scaffold the `spacemouse` package + constants/defaults

**Files:**
- Create: `dobot-ros/dobot_ros/spacemouse/__init__.py`
- Create: `dobot-ros/dobot_ros/spacemouse/hid_state.py`
- Create: `tests/spacemouse/__init__.py`
- Create: `tests/spacemouse/test_hid_state.py`

- [ ] **Step 1: Write the failing test**

File: `tests/spacemouse/test_hid_state.py`

```python
"""Unit tests for spacemouse dataclasses and constants."""
from __future__ import annotations

from dobot_ros.spacemouse.hid_state import (
    HidState, SpaceMouseSettings, DeviceStatus,
    DEFAULT_SETTINGS, VENDOR_ID, PRODUCT_ID,
)


def test_vendor_product_match_spacemouse_wireless_bt():
    assert VENDOR_ID == 0x256F
    assert PRODUCT_ID == 0xC63A


def test_default_settings_are_safe():
    s = SpaceMouseSettings(**DEFAULT_SETTINGS)
    assert s.max_velocity_xyz == 80.0
    assert s.max_velocity_rpy == 30.0
    assert s.deadband == 0.10
    assert s.sign_map == [1, 1, 1, 1, 1, 1]
    assert s.max_excursion_xyz == 300.0
    assert s.max_excursion_rpy == 90.0
    assert s.idle_auto_disarm_s == 30.0
    assert s.button_debounce_ms == 150
    assert s.gripper_force == 20


def test_hid_state_defaults_are_neutral():
    st = HidState()
    assert st.axes == [0.0] * 6
    assert st.buttons == [False, False]
    assert st.device_status == DeviceStatus.DISCONNECTED
    assert st.armed is False
    assert st.battery_pct is None


def test_hid_state_to_dict_roundtrips():
    st = HidState(
        axes=[0.1, -0.2, 0.3, 0.0, 0.0, 0.0],
        buttons=[True, False],
        device_status=DeviceStatus.CONNECTED,
        armed=True,
        battery_pct=87,
        offset=[5.0, -1.0, 0.0, 0.0, 0.0, 0.0],
        anchor=[400.0, 0.0, 300.0, 180.0, 0.0, 0.0],
        last_event_age_ms=12.5,
    )
    d = st.to_dict()
    assert d["axes"] == [0.1, -0.2, 0.3, 0.0, 0.0, 0.0]
    assert d["buttons"] == [True, False]
    assert d["device_status"] == "connected"
    assert d["armed"] is True
    assert d["battery_pct"] == 87


def test_signmap_validation_length():
    import pytest
    with pytest.raises(ValueError):
        SpaceMouseSettings(**{**DEFAULT_SETTINGS, "sign_map": [1, 1, 1]})


def test_signmap_validation_values():
    import pytest
    with pytest.raises(ValueError):
        SpaceMouseSettings(**{**DEFAULT_SETTINGS, "sign_map": [1, 1, 1, 1, 1, 2]})
```

- [ ] **Step 2: Run test to verify it fails**

```bash
cd /u/siegeld/dobot
PYTHONPATH=dobot-ros python -m pytest tests/spacemouse/test_hid_state.py -v
```

Expected: FAIL with `ModuleNotFoundError: No module named 'dobot_ros.spacemouse'`.

- [ ] **Step 3: Create package `__init__`**

File: `dobot-ros/dobot_ros/spacemouse/__init__.py`

```python
"""SpaceMouse pendant module.

Additive: this module provides a Bluetooth SpaceMouse Wireless BT jogging
pendant. It does not modify any existing motion path (MovJ/MovL/jog/pick
remain fully functional). See docs/superpowers/specs/2026-04-17-spacemouse-pendant-design.md.
"""

from dobot_ros.spacemouse.hid_state import (
    DEFAULT_SETTINGS,
    DeviceStatus,
    HidState,
    PRODUCT_ID,
    SpaceMouseSettings,
    VENDOR_ID,
)

__all__ = [
    "DEFAULT_SETTINGS",
    "DeviceStatus",
    "HidState",
    "PRODUCT_ID",
    "SpaceMouseSettings",
    "VENDOR_ID",
]
```

- [ ] **Step 4: Create `hid_state.py`**

File: `dobot-ros/dobot_ros/spacemouse/hid_state.py`

```python
"""Dataclasses and constants for the SpaceMouse pendant."""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional


# 3Dconnexion SpaceMouse Wireless BT (confirmed via `bluetoothctl info`).
VENDOR_ID = 0x256F
PRODUCT_ID = 0xC63A


class DeviceStatus(str, Enum):
    DISCONNECTED = "disconnected"   # not found on any /dev/input/event*
    CONNECTED = "connected"         # found and opened, no error
    LOST = "lost"                   # was connected, read failed (BT dropout)
    PERMISSION_DENIED = "permission_denied"


DEFAULT_SETTINGS = {
    "max_velocity_xyz": 80.0,
    "max_velocity_rpy": 30.0,
    "deadband": 0.10,
    "sign_map": [1, 1, 1, 1, 1, 1],
    "max_excursion_xyz": 300.0,
    "max_excursion_rpy": 90.0,
    "idle_auto_disarm_s": 30.0,
    "button_debounce_ms": 150,
    "gripper_force": 20,
}


@dataclass
class SpaceMouseSettings:
    """Persisted, live-editable tuning for the reader.

    Validated on construction; callers should catch ValueError when loading
    from untrusted input (e.g., a settings POST).
    """
    max_velocity_xyz: float = 80.0
    max_velocity_rpy: float = 30.0
    deadband: float = 0.10
    sign_map: List[int] = field(default_factory=lambda: [1, 1, 1, 1, 1, 1])
    max_excursion_xyz: float = 300.0
    max_excursion_rpy: float = 90.0
    idle_auto_disarm_s: float = 30.0
    button_debounce_ms: int = 150
    gripper_force: int = 20

    def __post_init__(self):
        if len(self.sign_map) != 6:
            raise ValueError(f"sign_map must have 6 elements, got {len(self.sign_map)}")
        for v in self.sign_map:
            if v not in (-1, 1):
                raise ValueError(f"sign_map values must be ±1, got {v}")
        if not 0.0 <= self.deadband < 1.0:
            raise ValueError("deadband must be in [0, 1)")
        if self.max_velocity_xyz <= 0 or self.max_velocity_rpy <= 0:
            raise ValueError("max_velocity_* must be > 0")
        if self.max_excursion_xyz <= 0 or self.max_excursion_rpy <= 0:
            raise ValueError("max_excursion_* must be > 0")
        if self.idle_auto_disarm_s <= 0:
            raise ValueError("idle_auto_disarm_s must be > 0")
        if not 0 <= self.gripper_force <= 100:
            raise ValueError("gripper_force must be in [0, 100]")


@dataclass
class HidState:
    """Snapshot of the pendant state pushed to the browser at ~10 Hz."""
    axes: List[float] = field(default_factory=lambda: [0.0] * 6)  # normalized -1..+1
    buttons: List[bool] = field(default_factory=lambda: [False, False])
    device_status: DeviceStatus = DeviceStatus.DISCONNECTED
    armed: bool = False
    battery_pct: Optional[int] = None
    offset: List[float] = field(default_factory=lambda: [0.0] * 6)        # mm/deg from anchor
    anchor: Optional[List[float]] = None                                  # pose at arm time, or None
    last_event_age_ms: float = 0.0                                        # since last evdev event
    idle_seconds: float = 0.0                                             # time in deadband
    error: Optional[str] = None                                           # human-readable reason

    def to_dict(self) -> dict:
        return {
            "axes": list(self.axes),
            "buttons": list(self.buttons),
            "device_status": self.device_status.value,
            "armed": self.armed,
            "battery_pct": self.battery_pct,
            "offset": list(self.offset),
            "anchor": list(self.anchor) if self.anchor is not None else None,
            "last_event_age_ms": self.last_event_age_ms,
            "idle_seconds": self.idle_seconds,
            "error": self.error,
        }
```

- [ ] **Step 5: Run tests to verify they pass**

```bash
cd /u/siegeld/dobot
PYTHONPATH=dobot-ros python -m pytest tests/spacemouse/test_hid_state.py -v
```

Expected: all tests PASS.

- [ ] **Step 6: Commit**

```bash
git add dobot-ros/dobot_ros/spacemouse/__init__.py \
        dobot-ros/dobot_ros/spacemouse/hid_state.py \
        tests/spacemouse/__init__.py \
        tests/spacemouse/test_hid_state.py
git commit -m "Add spacemouse package scaffolding with HidState and settings dataclasses"
```

---

## Task 2: Device discovery + battery helpers with a fake evdev

**Files:**
- Create: `dobot-ros/dobot_ros/spacemouse/device.py`
- Create: `tests/spacemouse/conftest.py`
- Create: `tests/spacemouse/test_device.py`

- [ ] **Step 1: Create the shared test fixtures (FakeEvdev)**

File: `tests/spacemouse/conftest.py`

```python
"""Fixtures for SpaceMouse tests.

Provides a FakeEvdev module that can be monkeypatched in place of the real
`evdev` import. Also re-exposes MockRosClient from tests/vla/conftest.py so
integration tests can reuse it.
"""
from __future__ import annotations

import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

import pytest

# Expose MockRosClient from VLA conftest without requiring package markers.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "vla"))
from conftest import MockRosClient  # noqa: E402  (re-used MockRosClient)


# ── Fake evdev ─────────────────────────────────────────────────────────
# Minimal stand-in for the python-evdev interface we use: list_devices(),
# InputDevice with .info, .absinfo(), .path, .fd, .read_loop(), .close().

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
    """Mimics evdev.InputEvent (code, type, value, timestamp)."""
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

    def absinfo(self, code: int) -> FakeAbsInfo:
        return FakeAbsInfo(min=-self._abs_max, max=self._abs_max)

    def capabilities(self, verbose: bool = False, absinfo: bool = True):
        # Not used in current tests; kept for API surface compatibility.
        return {}

    def read_loop(self) -> Iterable[FakeEvent]:
        for ev in self._events:
            if self._closed:
                return
            yield ev
        # After scripted events are drained, raise OSError to simulate disconnect
        # ONLY if the test asked for it (via setting ._raise_after_events).
        if getattr(self, "_raise_after_events", False):
            raise OSError("fake device disconnected")

    def close(self):
        self._closed = True


class FakeEvdev:
    """Holds a list of FakeInputDevice instances to return from list_devices()."""

    # evdev ecodes we rely on. The real values don't matter for tests as long
    # as they're stable within a test run.
    ecodes_ABS_X = 0
    ecodes_ABS_Y = 1
    ecodes_ABS_Z = 2
    ecodes_ABS_RX = 3
    ecodes_ABS_RY = 4
    ecodes_ABS_RZ = 5
    ecodes_BTN_0 = 100
    ecodes_BTN_1 = 101
    ecodes_EV_ABS = 3
    ecodes_EV_KEY = 1

    def __init__(self, devices: Optional[List[FakeInputDevice]] = None):
        self._devices: Dict[str, FakeInputDevice] = {
            d.path: d for d in (devices or [])
        }

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
```

- [ ] **Step 2: Write failing tests for `device.py`**

File: `tests/spacemouse/test_device.py`

```python
"""Tests for spacemouse/device.py."""
from __future__ import annotations

from pathlib import Path

import pytest

from dobot_ros.spacemouse import device as dev_mod


def test_find_device_returns_none_when_no_devices(fake_evdev_empty, monkeypatch):
    monkeypatch.setattr(dev_mod, "evdev", fake_evdev_empty)
    assert dev_mod.find_device() is None


def test_find_device_returns_none_for_unrelated_devices(fake_evdev_with_other_device, monkeypatch):
    monkeypatch.setattr(dev_mod, "evdev", fake_evdev_with_other_device)
    assert dev_mod.find_device() is None


def test_find_device_returns_path_for_spacemouse(fake_evdev_with_spacemouse, monkeypatch):
    monkeypatch.setattr(dev_mod, "evdev", fake_evdev_with_spacemouse)
    assert dev_mod.find_device() == "/dev/input/event21"


def test_open_device_returns_input_device(fake_evdev_with_spacemouse, monkeypatch):
    monkeypatch.setattr(dev_mod, "evdev", fake_evdev_with_spacemouse)
    d = dev_mod.open_device("/dev/input/event21")
    assert d.path == "/dev/input/event21"


def test_open_device_raises_permission_error_is_distinguished(monkeypatch):
    class DenyEvdev:
        def InputDevice(self, path):
            raise PermissionError(path)
    monkeypatch.setattr(dev_mod, "evdev", DenyEvdev())
    with pytest.raises(PermissionError):
        dev_mod.open_device("/dev/input/event21")


def test_read_battery_pct_parses_sysfs(tmp_path: Path, monkeypatch):
    # Simulate /sys/class/power_supply/hid-CD:D7:40:ED:B2:76-battery/capacity
    power = tmp_path / "power_supply"
    batt = power / "hid-CD:D7:40:ED:B2:76-battery"
    batt.mkdir(parents=True)
    (batt / "capacity").write_text("97\n")
    # Something else without capacity
    (power / "AC0").mkdir()

    result = dev_mod.read_battery_pct_in(power, match_substring="SpaceMouse")
    # Without name match, nothing returned
    assert result is None

    # With a name file, match
    (batt / "model_name").write_text("SpaceMouse Wireless BT\n")
    assert dev_mod.read_battery_pct_in(power, match_substring="SpaceMouse") == 97


def test_read_battery_pct_returns_none_when_missing(tmp_path: Path):
    assert dev_mod.read_battery_pct_in(tmp_path, match_substring="SpaceMouse") is None
```

- [ ] **Step 3: Run tests to verify they fail**

```bash
cd /u/siegeld/dobot
PYTHONPATH=dobot-ros python -m pytest tests/spacemouse/test_device.py -v
```

Expected: FAIL with `ModuleNotFoundError: No module named 'dobot_ros.spacemouse.device'`.

- [ ] **Step 4: Implement `device.py`**

File: `dobot-ros/dobot_ros/spacemouse/device.py`

```python
"""Device discovery and battery readout for the SpaceMouse pendant.

The real evdev module is imported lazily so this file can be imported in
test environments that don't have python3-evdev installed — tests replace
`device.evdev` with a fake before exercising the functions.
"""
from __future__ import annotations

import logging
from pathlib import Path
from typing import Optional

from dobot_ros.spacemouse.hid_state import VENDOR_ID, PRODUCT_ID

log = logging.getLogger(__name__)

try:  # pragma: no cover - environment dependent
    import evdev
except ImportError:  # pragma: no cover
    evdev = None  # tests monkeypatch this


_DEFAULT_POWER_SUPPLY = Path("/sys/class/power_supply")


def find_device() -> Optional[str]:
    """Return the /dev/input/event* path for the SpaceMouse, or None."""
    if evdev is None:
        log.warning("evdev not available; cannot scan for SpaceMouse")
        return None
    try:
        paths = evdev.list_devices()
    except Exception as e:
        log.warning("evdev.list_devices() failed: %s", e)
        return None
    for path in paths:
        try:
            d = evdev.InputDevice(path)
        except PermissionError:
            log.error("permission denied reading %s (container user not in 'input' group?)", path)
            raise
        except Exception as e:
            log.debug("skipping %s: %s", path, e)
            continue
        try:
            if d.info.vendor == VENDOR_ID and d.info.product == PRODUCT_ID:
                return path
        finally:
            try:
                d.close()
            except Exception:
                pass
    return None


def open_device(path: str):
    """Open an evdev.InputDevice; propagates PermissionError / FileNotFoundError."""
    if evdev is None:
        raise RuntimeError("evdev not available in this environment")
    return evdev.InputDevice(path)


def read_battery_pct(match_substring: str = "SpaceMouse") -> Optional[int]:
    """Scan /sys/class/power_supply/* for a battery whose model_name matches."""
    return read_battery_pct_in(_DEFAULT_POWER_SUPPLY, match_substring=match_substring)


def read_battery_pct_in(base: Path, match_substring: str = "SpaceMouse") -> Optional[int]:
    """Parameterized variant for testing."""
    try:
        if not base.exists():
            return None
        for entry in base.iterdir():
            model = entry / "model_name"
            cap = entry / "capacity"
            if model.exists() and cap.exists():
                try:
                    name = model.read_text().strip()
                except OSError:
                    continue
                if match_substring.lower() in name.lower():
                    try:
                        return int(cap.read_text().strip())
                    except (OSError, ValueError):
                        return None
    except Exception:  # pragma: no cover - defensive
        return None
    return None
```

- [ ] **Step 5: Run tests to verify they pass**

```bash
cd /u/siegeld/dobot
PYTHONPATH=dobot-ros python -m pytest tests/spacemouse/test_device.py -v
```

Expected: all PASS.

- [ ] **Step 6: Commit**

```bash
git add dobot-ros/dobot_ros/spacemouse/device.py \
        tests/spacemouse/conftest.py \
        tests/spacemouse/test_device.py
git commit -m "Add SpaceMouse device discovery and battery helpers"
```

---

## Task 3: `SpaceMouseReader` — skeleton with state + arm/disarm

**Files:**
- Create: `dobot-ros/dobot_ros/spacemouse/reader.py`
- Create: `tests/spacemouse/test_reader_decode.py`

This task gets the reader class structured with arm/disarm lifecycle, a `MockServoTester`-driven contract, and no threads yet (the tick is exposed as a public `_tick()` method so it's driven explicitly by tests). Task 4 adds the decoder; Task 5 wires the threads.

- [ ] **Step 1: Extend conftest with MockServoTester**

Edit: `tests/spacemouse/conftest.py` — append to the end (after `mock_ros` fixture):

```python
# ── Mock ServoTester ──────────────────────────────────────────────────
class MockServoTester:
    """Records arm/stop/target calls; mimics just the interface SpaceMouseReader uses."""

    def __init__(self, anchor=None):
        self._running = False
        self._anchor = list(anchor or [400.0, 0.0, 300.0, 180.0, 0.0, 0.0])
        self.start_calls = 0
        self.stop_calls = 0
        self.emergency_stop_calls = 0
        self.target_offsets: list[list[float]] = []

    def is_running(self) -> bool:
        return self._running

    def start(self):
        self._running = True
        self.start_calls += 1

        class _Status:
            anchor_pose = list(self._anchor)
        return _Status()

    def stop(self, timeout=2.0, call_robot_stop=False):
        self._running = False
        self.stop_calls += 1

    def emergency_stop(self):
        self._running = False
        self.emergency_stop_calls += 1

    def set_target_offset(self, offset):
        self.target_offsets.append(list(offset))

        class _Status:
            pass
        return _Status()

    @property
    def anchor(self):
        return list(self._anchor)


@pytest.fixture
def mock_servo():
    return MockServoTester()
```

- [ ] **Step 2: Write failing lifecycle tests**

File: `tests/spacemouse/test_reader_decode.py`

```python
"""Unit tests for SpaceMouseReader lifecycle and axis decode."""
from __future__ import annotations

from dobot_ros.spacemouse import device as dev_mod
from dobot_ros.spacemouse.hid_state import (
    DEFAULT_SETTINGS, DeviceStatus, SpaceMouseSettings,
)
from dobot_ros.spacemouse.reader import SpaceMouseReader


def _make_reader(mock_servo, mock_ros, **overrides):
    """Shared constructor used across reader tests."""
    settings = SpaceMouseSettings(**{**DEFAULT_SETTINGS, **overrides})
    clock = {"t": 0.0}

    def time_fn():
        return clock["t"]

    def advance(dt):
        clock["t"] += dt

    reader = SpaceMouseReader(
        servo_tester=mock_servo,
        ros_client=mock_ros,
        settings=settings,
        time_fn=time_fn,
        # Explicit device finder + opener so tests don't depend on evdev state.
        find_device_fn=lambda: "/dev/input/event21",
        open_device_fn=lambda path: None,   # reader doesn't auto-open in tests
        battery_fn=lambda: None,
    )
    return reader, advance


def test_reader_starts_disarmed_and_disconnected(mock_servo, mock_ros):
    reader, _ = _make_reader(mock_servo, mock_ros)
    state = reader.snapshot()
    assert state.armed is False
    assert state.device_status == DeviceStatus.DISCONNECTED
    assert state.offset == [0.0] * 6


def test_arm_requires_device(mock_servo, mock_ros):
    # Reader built with a finder that returns None.
    settings = SpaceMouseSettings(**DEFAULT_SETTINGS)
    reader = SpaceMouseReader(
        servo_tester=mock_servo, ros_client=mock_ros, settings=settings,
        time_fn=lambda: 0.0,
        find_device_fn=lambda: None,
        open_device_fn=lambda p: None,
        battery_fn=lambda: None,
    )
    ok, reason = reader.arm()
    assert ok is False
    assert "not found" in reason.lower() or "not connected" in reason.lower()
    assert mock_servo.start_calls == 0


def test_arm_starts_servo_and_records_anchor(mock_servo, mock_ros):
    reader, _ = _make_reader(mock_servo, mock_ros)
    ok, reason = reader.arm()
    assert ok is True, reason
    assert mock_servo.start_calls == 1
    state = reader.snapshot()
    assert state.armed is True
    assert state.anchor is not None


def test_disarm_stops_servo_and_resets_offset(mock_servo, mock_ros):
    reader, _ = _make_reader(mock_servo, mock_ros)
    reader.arm()
    reader.disarm()
    assert mock_servo.stop_calls == 1
    state = reader.snapshot()
    assert state.armed is False
    # Offset is zeroed on disarm (anchor will be recaptured on re-arm).
    assert state.offset == [0.0] * 6


def test_double_arm_is_noop(mock_servo, mock_ros):
    reader, _ = _make_reader(mock_servo, mock_ros)
    reader.arm()
    reader.arm()
    assert mock_servo.start_calls == 1


def test_emergency_stop_uses_tester_estop(mock_servo, mock_ros):
    reader, _ = _make_reader(mock_servo, mock_ros)
    reader.arm()
    reader.emergency_stop()
    assert mock_servo.emergency_stop_calls == 1
    state = reader.snapshot()
    assert state.armed is False
```

- [ ] **Step 3: Run tests to verify they fail**

```bash
cd /u/siegeld/dobot
PYTHONPATH=dobot-ros python -m pytest tests/spacemouse/test_reader_decode.py -v
```

Expected: FAIL with `ModuleNotFoundError` for `reader`.

- [ ] **Step 4: Create `reader.py` skeleton**

File: `dobot-ros/dobot_ros/spacemouse/reader.py`

```python
"""SpaceMouseReader — bridges a 3Dconnexion SpaceMouse Wireless BT to ServoTester.

Additive: calls ServoTester.set_target_offset() (already existing) to stream
live jog offsets. Two daemon threads:
  1. evdev read loop (blocks on device.read_loop()), decodes axis/button events.
  2. 50 Hz integration tick: deadband + scale + integrate + clamp, forwards
     to ServoTester; debounces buttons → gripper.

Both threads are added in a later task; this file starts with the lifecycle
API and a public `_tick()` that tests drive explicitly.

See docs/superpowers/specs/2026-04-17-spacemouse-pendant-design.md.
"""
from __future__ import annotations

import copy
import logging
import threading
from typing import Callable, List, Optional, Tuple

from dobot_ros.spacemouse.hid_state import (
    DeviceStatus, HidState, SpaceMouseSettings,
)

log = logging.getLogger(__name__)


class SpaceMouseReader:
    """Owner of the SpaceMouse device + integration loop.

    Parameters
    ----------
    servo_tester :
        Object exposing start(), stop(), emergency_stop(), is_running(),
        set_target_offset(list[float]). In production this is a ServoTester;
        in tests it's a MockServoTester.
    ros_client :
        DobotRosClient (or mock) — used only for gripper_move() on button edges.
    settings :
        Live-editable SpaceMouseSettings. Updated via update_settings().
    time_fn :
        Injected clock (default: time.monotonic). Tests pass a stub.
    find_device_fn, open_device_fn, battery_fn :
        Dependency injection for device I/O so tests don't need real evdev.
    """

    def __init__(
        self,
        servo_tester,
        ros_client,
        settings: SpaceMouseSettings,
        time_fn: Callable[[], float],
        find_device_fn: Callable[[], Optional[str]],
        open_device_fn: Callable[[str], object],
        battery_fn: Callable[[], Optional[int]],
    ):
        self._servo = servo_tester
        self._ros = ros_client
        self._settings = settings
        self._time = time_fn
        self._find_device = find_device_fn
        self._open_device = open_device_fn
        self._read_battery = battery_fn

        self._lock = threading.RLock()
        self._state = HidState()
        self._offset: List[float] = [0.0] * 6
        self._last_raw: List[float] = [0.0] * 6  # normalized axis values
        self._buttons: List[bool] = [False, False]
        self._last_button_edge: List[float] = [0.0, 0.0]  # time of last accepted press edge
        self._last_nonidle_t: float = 0.0
        self._device_path: Optional[str] = None
        self._armed: bool = False

    # ── Public API ──────────────────────────────────────────────────
    def snapshot(self) -> HidState:
        with self._lock:
            return copy.deepcopy(self._state)

    def update_settings(self, settings: SpaceMouseSettings):
        with self._lock:
            self._settings = settings

    def arm(self) -> Tuple[bool, Optional[str]]:
        """Check gates and arm the reader. Returns (ok, reason)."""
        with self._lock:
            if self._armed:
                return True, None
            path = self._find_device()
            if path is None:
                self._state.device_status = DeviceStatus.DISCONNECTED
                self._state.error = "SpaceMouse not connected"
                return False, "SpaceMouse not connected — check Bluetooth"
            self._device_path = path
            try:
                status = self._servo.start()
                anchor = list(getattr(status, "anchor_pose", []) or [])
            except Exception as e:
                return False, f"failed to start servo stream: {e}"
            self._offset = [0.0] * 6
            self._last_nonidle_t = self._time()
            self._state = HidState(
                armed=True,
                device_status=DeviceStatus.CONNECTED,
                anchor=anchor,
                battery_pct=self._safe_battery(),
            )
            self._armed = True
            log.info("SpaceMouse armed; anchor=%s device=%s", anchor, path)
        return True, None

    def disarm(self):
        """Graceful disarm: stop the servo stream, hold pose."""
        with self._lock:
            if not self._armed:
                return
            self._armed = False
            self._offset = [0.0] * 6
            try:
                self._servo.stop()
            except Exception as e:
                log.warning("servo.stop() during disarm failed: %s", e)
            self._state.armed = False
            self._state.offset = [0.0] * 6
            log.info("SpaceMouse disarmed")

    def emergency_stop(self):
        """E-stop: halt the servo stream AND call ros.stop()."""
        with self._lock:
            self._armed = False
            self._offset = [0.0] * 6
            try:
                self._servo.emergency_stop()
            except Exception as e:
                log.warning("servo.emergency_stop() failed: %s", e)
            self._state.armed = False
            self._state.offset = [0.0] * 6
            self._state.error = "emergency stop"
            log.warning("SpaceMouse E-stop")

    # ── Internals ───────────────────────────────────────────────────
    def _safe_battery(self) -> Optional[int]:
        try:
            return self._read_battery()
        except Exception:
            return None
```

- [ ] **Step 5: Run tests to verify they pass**

```bash
cd /u/siegeld/dobot
PYTHONPATH=dobot-ros python -m pytest tests/spacemouse/test_reader_decode.py -v
```

Expected: all PASS.

- [ ] **Step 6: Commit**

```bash
git add dobot-ros/dobot_ros/spacemouse/reader.py \
        tests/spacemouse/conftest.py \
        tests/spacemouse/test_reader_decode.py
git commit -m "Add SpaceMouseReader with arm/disarm lifecycle and tester injection"
```

---

## Task 4: Axis normalization, deadband, sign map, button debounce (pure functions + decode API)

**Files:**
- Modify: `dobot-ros/dobot_ros/spacemouse/reader.py`
- Modify: `tests/spacemouse/test_reader_decode.py`

- [ ] **Step 1: Append failing tests for decode helpers**

Append to `tests/spacemouse/test_reader_decode.py`:

```python
# ── Decode / deadband / sign-map tests ──────────────────────────────


def test_normalize_by_absinfo_max(mock_servo, mock_ros):
    reader, _ = _make_reader(mock_servo, mock_ros)
    # Simulate absinfo.max = 350 per axis.
    reader.set_axis_absmax([350] * 6)
    reader.on_raw_axis(axis_index=0, raw_value=175)   # half-deflection +X
    s = reader.snapshot()
    assert abs(s.axes[0] - 0.5) < 1e-6


def test_normalize_clips_above_one(mock_servo, mock_ros):
    reader, _ = _make_reader(mock_servo, mock_ros)
    reader.set_axis_absmax([350] * 6)
    reader.on_raw_axis(axis_index=2, raw_value=10_000)
    s = reader.snapshot()
    assert s.axes[2] == 1.0


def test_normalize_clips_below_minus_one(mock_servo, mock_ros):
    reader, _ = _make_reader(mock_servo, mock_ros)
    reader.set_axis_absmax([350] * 6)
    reader.on_raw_axis(axis_index=2, raw_value=-10_000)
    s = reader.snapshot()
    assert s.axes[2] == -1.0


def test_deadband_zeroes_small_deflections(mock_servo, mock_ros):
    reader, _ = _make_reader(mock_servo, mock_ros, deadband=0.10)
    reader.set_axis_absmax([350] * 6)
    # 5% deflection — below deadband.
    reader.on_raw_axis(axis_index=0, raw_value=int(0.05 * 350))
    assert reader.effective_velocity()[0] == 0.0
    # 20% deflection — above deadband, positive velocity.
    reader.on_raw_axis(axis_index=0, raw_value=int(0.20 * 350))
    assert reader.effective_velocity()[0] > 0.0


def test_sign_map_flips_velocity(mock_servo, mock_ros):
    reader, _ = _make_reader(mock_servo, mock_ros, sign_map=[-1, 1, 1, 1, 1, 1])
    reader.set_axis_absmax([350] * 6)
    reader.on_raw_axis(axis_index=0, raw_value=350)  # full +X
    v = reader.effective_velocity()
    assert v[0] < 0.0   # flipped


def test_velocity_uses_max_velocity_from_settings(mock_servo, mock_ros):
    reader, _ = _make_reader(mock_servo, mock_ros, max_velocity_xyz=80.0, max_velocity_rpy=30.0)
    reader.set_axis_absmax([350] * 6)
    reader.on_raw_axis(axis_index=0, raw_value=350)  # full +X → 80 mm/s
    reader.on_raw_axis(axis_index=5, raw_value=350)  # full +RZ → 30 deg/s
    v = reader.effective_velocity()
    assert abs(v[0] - 80.0) < 1e-6
    assert abs(v[5] - 30.0) < 1e-6


def test_button_press_edge_armed_triggers_gripper(mock_servo, mock_ros):
    reader, advance = _make_reader(mock_servo, mock_ros)
    reader.arm()
    reader.on_button(0, pressed=True)          # press CLOSE
    assert len(mock_ros.gripper_moves) == 1
    assert mock_ros.gripper_moves[0]["position"] == 0


def test_button_press_edge_disarmed_is_echoed_but_no_action(mock_servo, mock_ros):
    reader, _ = _make_reader(mock_servo, mock_ros)
    reader.on_button(0, pressed=True)
    assert mock_ros.gripper_moves == []
    s = reader.snapshot()
    assert s.buttons[0] is True


def test_button_debounce_drops_rapid_repeats(mock_servo, mock_ros):
    reader, advance = _make_reader(mock_servo, mock_ros, button_debounce_ms=150)
    reader.arm()
    reader.on_button(0, pressed=True)
    advance(0.050)       # 50ms later — within debounce
    reader.on_button(0, pressed=False)
    reader.on_button(0, pressed=True)
    assert len(mock_ros.gripper_moves) == 1
    advance(0.200)       # well past debounce
    reader.on_button(0, pressed=False)
    reader.on_button(0, pressed=True)
    assert len(mock_ros.gripper_moves) == 2


def test_button_open_maps_to_1000(mock_servo, mock_ros):
    reader, _ = _make_reader(mock_servo, mock_ros)
    reader.arm()
    reader.on_button(1, pressed=True)
    assert mock_ros.gripper_moves[-1]["position"] == 1000


def test_button_close_maps_to_0(mock_servo, mock_ros):
    reader, _ = _make_reader(mock_servo, mock_ros)
    reader.arm()
    reader.on_button(0, pressed=True)
    assert mock_ros.gripper_moves[-1]["position"] == 0
```

- [ ] **Step 2: Run tests — they should fail**

```bash
cd /u/siegeld/dobot
PYTHONPATH=dobot-ros python -m pytest tests/spacemouse/test_reader_decode.py -v
```

Expected: many new failures on `AttributeError: SpaceMouseReader has no attribute 'set_axis_absmax'` etc.

- [ ] **Step 3: Add decode methods to `reader.py`**

Append inside `class SpaceMouseReader` (before `# ── Internals`):

```python
    # ── Event ingestion (driven by evdev thread OR by tests) ────────
    def set_axis_absmax(self, absmax_per_axis: List[int]):
        """Record the raw range for normalization. Called once after device open."""
        if len(absmax_per_axis) != 6:
            raise ValueError("absmax_per_axis must have 6 entries")
        with self._lock:
            self._abs_max = list(absmax_per_axis)

    def on_raw_axis(self, axis_index: int, raw_value: int):
        """Ingest one axis event. axis_index in [0..5]: 0..2 = TX/TY/TZ, 3..5 = RX/RY/RZ."""
        if not 0 <= axis_index <= 5:
            return
        with self._lock:
            amax = self._abs_max[axis_index] if getattr(self, "_abs_max", None) else 350
            norm = raw_value / float(amax) if amax else 0.0
            if norm > 1.0:
                norm = 1.0
            elif norm < -1.0:
                norm = -1.0
            self._last_raw[axis_index] = norm
            self._state.axes = list(self._last_raw)

    def on_button(self, button_index: int, pressed: bool):
        """Ingest one button event. button_index in {0, 1}."""
        if button_index not in (0, 1):
            return
        with self._lock:
            was = self._buttons[button_index]
            self._buttons[button_index] = bool(pressed)
            self._state.buttons = list(self._buttons)
            # Edge detection: only the rising edge triggers actions.
            if pressed and not was:
                now = self._time()
                debounce_s = self._settings.button_debounce_ms / 1000.0
                if now - self._last_button_edge[button_index] < debounce_s:
                    return
                self._last_button_edge[button_index] = now
                self._last_nonidle_t = now  # count as activity for idle-timer
                if self._armed:
                    self._fire_gripper(button_index)

    def effective_velocity(self) -> List[float]:
        """Compute the current 6-D velocity command (with deadband + sign map).

        Units: mm/s for indices 0..2, deg/s for 3..5. Exposed publicly so
        tests don't have to drive the tick loop.
        """
        with self._lock:
            s = self._settings
            v = [0.0] * 6
            for i in range(6):
                a = self._last_raw[i]
                if abs(a) < s.deadband:
                    continue
                scale = s.max_velocity_xyz if i < 3 else s.max_velocity_rpy
                v[i] = a * scale * s.sign_map[i]
            return v

    def _fire_gripper(self, button_index: int):
        position = 0 if button_index == 0 else 1000
        force = self._settings.gripper_force
        try:
            # Fire-and-forget to stay consistent with the rest of the web API:
            # we don't block the read thread on the robot's ack. `wait=False`
            # matches the existing /api/gripper/* endpoints.
            self._ros.gripper_move(position, force=force, speed=50, wait=False)
        except Exception as e:
            log.warning("gripper_move(%d) failed: %s", position, e)
```

- [ ] **Step 4: Run tests to confirm all pass**

```bash
cd /u/siegeld/dobot
PYTHONPATH=dobot-ros python -m pytest tests/spacemouse/test_reader_decode.py -v
```

Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add dobot-ros/dobot_ros/spacemouse/reader.py \
        tests/spacemouse/test_reader_decode.py
git commit -m "Add axis decode, deadband, sign-map, and button debounce to SpaceMouseReader"
```

---

## Task 5: Integration tick — deflection → offset → ServoTester, with clamp + idle disarm + disconnect handling

**Files:**
- Modify: `dobot-ros/dobot_ros/spacemouse/reader.py`
- Create: `tests/spacemouse/test_reader_integration.py`

- [ ] **Step 1: Write failing integration tests**

File: `tests/spacemouse/test_reader_integration.py`

```python
"""Integration tests for SpaceMouseReader tick → ServoTester + idle disarm."""
from __future__ import annotations

import pytest

from dobot_ros.spacemouse.hid_state import DEFAULT_SETTINGS, SpaceMouseSettings
from dobot_ros.spacemouse.reader import SpaceMouseReader


def _reader_with_clock(mock_servo, mock_ros, **overrides):
    settings = SpaceMouseSettings(**{**DEFAULT_SETTINGS, **overrides})
    clock = {"t": 0.0}

    def time_fn():
        return clock["t"]

    def advance(dt):
        clock["t"] += dt

    r = SpaceMouseReader(
        servo_tester=mock_servo, ros_client=mock_ros, settings=settings,
        time_fn=time_fn,
        find_device_fn=lambda: "/dev/input/event21",
        open_device_fn=lambda p: None,
        battery_fn=lambda: 87,
    )
    r.set_axis_absmax([350] * 6)
    return r, advance


def test_tick_does_nothing_when_disarmed(mock_servo, mock_ros):
    r, advance = _reader_with_clock(mock_servo, mock_ros)
    r.on_raw_axis(0, 350)   # full +X
    advance(0.02)
    r.tick_once(dt=0.02)
    assert mock_servo.target_offsets == []


def test_tick_integrates_offset_when_armed(mock_servo, mock_ros):
    r, advance = _reader_with_clock(mock_servo, mock_ros, max_velocity_xyz=80.0)
    r.arm()
    r.on_raw_axis(0, 350)   # full +X → +80 mm/s
    advance(0.02)
    r.tick_once(dt=0.02)
    assert len(mock_servo.target_offsets) == 1
    off = mock_servo.target_offsets[-1]
    assert abs(off[0] - 80.0 * 0.02) < 1e-6   # 1.6 mm
    assert off[1:] == [0.0, 0.0, 0.0, 0.0, 0.0]


def test_tick_accumulates_offset_over_multiple_ticks(mock_servo, mock_ros):
    r, advance = _reader_with_clock(mock_servo, mock_ros, max_velocity_xyz=80.0)
    r.arm()
    r.on_raw_axis(0, 350)   # full +X held
    for _ in range(10):
        advance(0.02)
        r.tick_once(dt=0.02)
    off = mock_servo.target_offsets[-1]
    # 80 mm/s * 0.02 s * 10 ticks = 16 mm
    assert abs(off[0] - 16.0) < 1e-3


def test_tick_respects_deadband(mock_servo, mock_ros):
    r, advance = _reader_with_clock(mock_servo, mock_ros, deadband=0.1)
    r.arm()
    r.on_raw_axis(0, int(0.05 * 350))   # below deadband
    for _ in range(10):
        advance(0.02)
        r.tick_once(dt=0.02)
    off = mock_servo.target_offsets[-1]
    # Deadband → no advance. ServoTester was still called each tick with offset=0.
    assert off == [0.0] * 6


def test_tick_clamps_offset_to_max_excursion(mock_servo, mock_ros):
    r, advance = _reader_with_clock(
        mock_servo, mock_ros, max_velocity_xyz=1000.0, max_excursion_xyz=50.0)
    r.arm()
    r.on_raw_axis(0, 350)   # full +X at 1000 mm/s
    for _ in range(20):
        advance(0.02)
        r.tick_once(dt=0.02)
    off = mock_servo.target_offsets[-1]
    # Hit the 50 mm excursion cap and stayed there.
    assert off[0] == pytest.approx(50.0)


def test_tick_clamps_negative_excursion(mock_servo, mock_ros):
    r, advance = _reader_with_clock(
        mock_servo, mock_ros, max_velocity_xyz=1000.0, max_excursion_xyz=50.0)
    r.arm()
    r.on_raw_axis(0, -350)
    for _ in range(20):
        advance(0.02)
        r.tick_once(dt=0.02)
    off = mock_servo.target_offsets[-1]
    assert off[0] == pytest.approx(-50.0)


def test_idle_auto_disarm_fires_after_timeout(mock_servo, mock_ros):
    r, advance = _reader_with_clock(mock_servo, mock_ros, idle_auto_disarm_s=1.0)
    r.arm()
    # All axes inside deadband — idle.
    for _ in range(60):   # 1.2 s of ticks at 50 Hz
        advance(0.02)
        r.tick_once(dt=0.02)
    assert r.snapshot().armed is False
    assert mock_servo.stop_calls == 1


def test_button_press_resets_idle_timer(mock_servo, mock_ros):
    r, advance = _reader_with_clock(mock_servo, mock_ros, idle_auto_disarm_s=1.0)
    r.arm()
    for _ in range(30):   # 0.6 s idle
        advance(0.02)
        r.tick_once(dt=0.02)
    r.on_button(1, pressed=True)
    r.on_button(1, pressed=False)
    for _ in range(30):   # another 0.6 s — cumulatively past 1.0 s from arm,
                          # but button at 0.6 s reset the timer.
        advance(0.02)
        r.tick_once(dt=0.02)
    assert r.snapshot().armed is True


def test_axis_motion_resets_idle_timer(mock_servo, mock_ros):
    r, advance = _reader_with_clock(mock_servo, mock_ros, idle_auto_disarm_s=1.0)
    r.arm()
    for _ in range(30):
        advance(0.02)
        r.tick_once(dt=0.02)
    r.on_raw_axis(0, 350)   # full +X
    for _ in range(5):
        advance(0.02)
        r.tick_once(dt=0.02)
    r.on_raw_axis(0, 0)
    for _ in range(30):
        advance(0.02)
        r.tick_once(dt=0.02)
    assert r.snapshot().armed is True


def test_disconnect_disarms_and_calls_servo_estop(mock_servo, mock_ros):
    r, _ = _reader_with_clock(mock_servo, mock_ros)
    r.arm()
    r.notify_device_lost("read failed")
    assert r.snapshot().armed is False
    assert r.snapshot().device_status.value == "lost"
    assert mock_servo.emergency_stop_calls == 1


def test_snapshot_offset_matches_last_commanded(mock_servo, mock_ros):
    r, advance = _reader_with_clock(mock_servo, mock_ros, max_velocity_xyz=80.0)
    r.arm()
    r.on_raw_axis(0, 350)
    for _ in range(5):
        advance(0.02)
        r.tick_once(dt=0.02)
    s = r.snapshot()
    assert abs(s.offset[0] - 80.0 * 0.02 * 5) < 1e-3
```

- [ ] **Step 2: Run tests — should fail on missing `tick_once` / `notify_device_lost`**

```bash
cd /u/siegeld/dobot
PYTHONPATH=dobot-ros python -m pytest tests/spacemouse/test_reader_integration.py -v
```

Expected: FAIL.

- [ ] **Step 3: Add `tick_once` + `notify_device_lost` to `reader.py`**

Append inside `class SpaceMouseReader` (before `# ── Internals`):

```python
    # ── Tick loop ───────────────────────────────────────────────────
    def tick_once(self, dt: float):
        """One iteration of the 50 Hz integration loop. Tests drive this directly;
        in production the Task-6 thread calls it every `dt` seconds."""
        with self._lock:
            s = self._settings
            v = [0.0] * 6
            any_active = False
            for i in range(6):
                a = self._last_raw[i]
                if abs(a) >= s.deadband:
                    scale = s.max_velocity_xyz if i < 3 else s.max_velocity_rpy
                    v[i] = a * scale * s.sign_map[i]
                    any_active = True

            # Integrate.
            new_offset = list(self._offset)
            for i in range(6):
                new_offset[i] += v[i] * dt

            # Excursion clamp.
            mx = s.max_excursion_xyz
            mr = s.max_excursion_rpy
            for i in range(3):
                if new_offset[i] > mx: new_offset[i] = mx
                elif new_offset[i] < -mx: new_offset[i] = -mx
            for i in range(3, 6):
                if new_offset[i] > mr: new_offset[i] = mr
                elif new_offset[i] < -mr: new_offset[i] = -mr

            self._offset = new_offset
            self._state.offset = list(new_offset)

            # Idle tracking: timestamp the last tick where something was active.
            now = self._time()
            if any_active:
                self._last_nonidle_t = now
            idle_s = now - self._last_nonidle_t
            self._state.idle_seconds = idle_s

            armed = self._armed

        # Side effects (servo + idle disarm) outside the lock where possible,
        # but we re-acquire for armed state transitions.
        if armed:
            try:
                self._servo.set_target_offset(self._offset)
            except Exception as e:
                log.warning("servo.set_target_offset failed: %s", e)

            # Idle auto-disarm.
            if idle_s >= self._settings.idle_auto_disarm_s:
                log.info("SpaceMouse idle %.1fs → auto-disarm", idle_s)
                self.disarm()

    def notify_device_lost(self, reason: str):
        """Called by the evdev thread (or tests) when the device goes away."""
        with self._lock:
            was_armed = self._armed
            self._armed = False
            self._state.armed = False
            self._state.device_status = DeviceStatus.LOST
            self._state.error = f"device lost: {reason}"
            self._device_path = None
        if was_armed:
            try:
                self._servo.emergency_stop()
            except Exception as e:
                log.warning("emergency_stop on device loss failed: %s", e)
        log.warning("SpaceMouse device lost: %s", reason)
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
cd /u/siegeld/dobot
PYTHONPATH=dobot-ros python -m pytest tests/spacemouse/test_reader_integration.py -v
```

Expected: all PASS.

- [ ] **Step 5: Run the full spacemouse suite**

```bash
cd /u/siegeld/dobot
PYTHONPATH=dobot-ros python -m pytest tests/spacemouse/ -v
```

Expected: all green.

- [ ] **Step 6: Commit**

```bash
git add dobot-ros/dobot_ros/spacemouse/reader.py \
        tests/spacemouse/test_reader_integration.py
git commit -m "Implement SpaceMouseReader tick loop, excursion clamp, idle disarm, disconnect"
```

---

## Task 6: Threaded run/stop wrapper — real evdev read loop + 50 Hz tick thread

**Files:**
- Modify: `dobot-ros/dobot_ros/spacemouse/reader.py`
- Modify: `tests/spacemouse/test_reader_integration.py`

This wires the decode + tick to two real daemon threads. Tests script the fake evdev and verify the threads drive the same call sequence.

- [ ] **Step 1: Append failing test using threads + FakeEvdev**

Append to `tests/spacemouse/test_reader_integration.py`:

```python
import time as real_time


def test_run_spins_both_threads_and_stops_cleanly(mock_servo, mock_ros, fake_evdev_with_spacemouse, monkeypatch):
    from dobot_ros.spacemouse import reader as reader_mod
    # Route the reader's evdev imports through our fake.
    monkeypatch.setattr(reader_mod, "_evdev", fake_evdev_with_spacemouse)
    from dobot_ros.spacemouse.hid_state import DEFAULT_SETTINGS, SpaceMouseSettings

    r = reader_mod.SpaceMouseReader(
        servo_tester=mock_servo, ros_client=mock_ros,
        settings=SpaceMouseSettings(**DEFAULT_SETTINGS),
        time_fn=real_time.monotonic,
        find_device_fn=lambda: "/dev/input/event21",
        open_device_fn=lambda path: fake_evdev_with_spacemouse.InputDevice(path),
        battery_fn=lambda: None,
    )
    r.start_threads(tick_hz=100.0)          # high rate for a short test
    try:
        r.arm()
        real_time.sleep(0.15)
        r.disarm()
    finally:
        r.stop_threads(timeout=1.0)
    # At least a few ticks should have happened.
    assert mock_servo.start_calls == 1
    assert mock_servo.stop_calls == 1
```

- [ ] **Step 2: Run test — expected to fail on `start_threads` missing**

```bash
cd /u/siegeld/dobot
PYTHONPATH=dobot-ros python -m pytest tests/spacemouse/test_reader_integration.py::test_run_spins_both_threads_and_stops_cleanly -v
```

Expected: FAIL.

- [ ] **Step 3: Add threads to `reader.py`**

Add at the top of `reader.py`, near imports:

```python
import time

try:
    import evdev as _evdev  # re-bound so tests can monkeypatch reader._evdev
    from evdev import ecodes as _ecodes
except ImportError:  # pragma: no cover
    _evdev = None
    _ecodes = None
```

Append to `class SpaceMouseReader` (before `# ── Internals`):

```python
    # ── Threads ─────────────────────────────────────────────────────
    def start_threads(self, tick_hz: float = 50.0):
        """Start the evdev read thread and the 50 Hz integration tick."""
        if getattr(self, "_stop_event", None) is not None:
            return
        self._stop_event = threading.Event()
        self._tick_hz = tick_hz
        self._tick_thread = threading.Thread(
            target=self._tick_loop, name="spacemouse-tick", daemon=True)
        self._read_thread = threading.Thread(
            target=self._read_loop, name="spacemouse-read", daemon=True)
        self._tick_thread.start()
        self._read_thread.start()

    def stop_threads(self, timeout: float = 2.0):
        """Stop both threads. Idempotent."""
        ev = getattr(self, "_stop_event", None)
        if ev is None:
            return
        ev.set()
        # Disarm first so nothing calls set_target_offset while we shut down.
        try:
            self.disarm()
        except Exception:
            pass
        # The read thread is blocked on read_loop(); closing the device
        # releases it. device.close() is idempotent in our fake.
        try:
            dev = getattr(self, "_open_device_obj", None)
            if dev is not None:
                try:
                    dev.close()
                except Exception:
                    pass
        finally:
            self._tick_thread.join(timeout=timeout)
            self._read_thread.join(timeout=timeout)
            self._stop_event = None
            self._tick_thread = None
            self._read_thread = None
            self._open_device_obj = None

    # Background loops ------------------------------------------------

    def _tick_loop(self):
        period = 1.0 / max(1.0, self._tick_hz)
        last = self._time()
        while not self._stop_event.is_set():
            now = self._time()
            dt = max(1e-4, now - last)
            last = now
            try:
                self.tick_once(dt=dt)
            except Exception:
                log.exception("spacemouse tick failed")
            self._stop_event.wait(timeout=period)

    def _read_loop(self):
        # Auto-(re)connect: loop until stop_event is set.
        while not self._stop_event.is_set():
            path = self._find_device()
            if path is None:
                self._stop_event.wait(timeout=1.0)
                continue
            try:
                dev = self._open_device(path)
            except PermissionError:
                with self._lock:
                    self._state.device_status = DeviceStatus.PERMISSION_DENIED
                    self._state.error = "permission denied on /dev/input (check input group)"
                return  # fatal — don't spin forever
            except Exception as e:
                log.warning("open_device(%s) failed: %s", path, e)
                self._stop_event.wait(timeout=1.0)
                continue
            self._open_device_obj = dev
            self._discover_absinfo(dev)
            with self._lock:
                self._state.device_status = DeviceStatus.CONNECTED
                self._state.error = None
                self._device_path = path
            try:
                self._pump_events(dev)
            except OSError as e:
                self.notify_device_lost(str(e))
            except Exception as e:
                log.exception("read loop crashed: %s", e)
                self.notify_device_lost(str(e))
            finally:
                try:
                    dev.close()
                except Exception:
                    pass
            # Loop and try to rediscover.

    def _pump_events(self, dev):
        """Translate evdev events into on_raw_axis / on_button calls.

        The evdev axis code → axis_index mapping uses the ecodes module so the
        code values aren't hard-coded.
        """
        if _ecodes is None and _evdev is None:
            return
        axis_code_map = {
            _ecodes.ABS_X: 0, _ecodes.ABS_Y: 1, _ecodes.ABS_Z: 2,
            _ecodes.ABS_RX: 3, _ecodes.ABS_RY: 4, _ecodes.ABS_RZ: 5,
        } if _ecodes is not None else {}
        # Button codes: evdev varies by firmware; BTN_0 / BTN_1 are the common
        # mapping for 3Dconnexion pucks. If the firmware reports different
        # codes, settings can be extended to remap; v1 only supports the defaults.
        button_code_map = {
            _ecodes.BTN_0: 0, _ecodes.BTN_1: 1,
        } if _ecodes is not None else {}
        for ev in dev.read_loop():
            if self._stop_event.is_set():
                return
            etype = ev.type
            if etype == (_ecodes.EV_ABS if _ecodes else 3):
                idx = axis_code_map.get(ev.code)
                if idx is not None:
                    self.on_raw_axis(idx, ev.value)
            elif etype == (_ecodes.EV_KEY if _ecodes else 1):
                idx = button_code_map.get(ev.code)
                if idx is not None:
                    self.on_button(idx, pressed=bool(ev.value))

    def _discover_absinfo(self, dev):
        """Populate per-axis absmax from device capabilities."""
        if _ecodes is None:
            # Testing path without real evdev: the test has already called
            # set_axis_absmax() explicitly, or we fall back to 350.
            if not getattr(self, "_abs_max", None):
                self.set_axis_absmax([350] * 6)
            return
        axes = [_ecodes.ABS_X, _ecodes.ABS_Y, _ecodes.ABS_Z,
                _ecodes.ABS_RX, _ecodes.ABS_RY, _ecodes.ABS_RZ]
        maxes: List[int] = []
        for code in axes:
            try:
                info = dev.absinfo(code)
                maxes.append(int(info.max) if info and info.max else 350)
            except Exception:
                maxes.append(350)
        self.set_axis_absmax(maxes)
```

Also update the conftest `FakeEvdev` so it exposes the `ecodes` namespace the reader expects. Edit: `tests/spacemouse/conftest.py` — replace the `class FakeEvdev:` section's ecodes constants with a proper ecodes holder:

```python
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

    def __init__(self, devices=None):
        self._devices = {d.path: d for d in (devices or [])}

    def list_devices(self):
        return list(self._devices.keys())

    def InputDevice(self, path):
        if path not in self._devices:
            raise FileNotFoundError(path)
        return self._devices[path]

    def add(self, device):
        self._devices[device.path] = device
```

And update the `test_run_spins_both_threads_and_stops_cleanly` monkeypatch to also bind `_ecodes`:

```python
    monkeypatch.setattr(reader_mod, "_evdev", fake_evdev_with_spacemouse)
    monkeypatch.setattr(reader_mod, "_ecodes", fake_evdev_with_spacemouse.ecodes)
```

- [ ] **Step 4: Run the failing test to confirm it now passes**

```bash
cd /u/siegeld/dobot
PYTHONPATH=dobot-ros python -m pytest tests/spacemouse/test_reader_integration.py -v
```

Expected: all PASS (including the new `test_run_spins_both_threads_and_stops_cleanly`).

- [ ] **Step 5: Run the full spacemouse suite**

```bash
cd /u/siegeld/dobot
PYTHONPATH=dobot-ros python -m pytest tests/spacemouse/ -v
```

Expected: all green.

- [ ] **Step 6: Commit**

```bash
git add dobot-ros/dobot_ros/spacemouse/reader.py \
        tests/spacemouse/conftest.py \
        tests/spacemouse/test_reader_integration.py
git commit -m "Add threaded run/stop to SpaceMouseReader with evdev read loop"
```

---

## Task 7: Wire reader into the web server — settings group, factory, motion lock

**Files:**
- Modify: `dobot-ros/dobot_ros/web/server.py`

- [ ] **Step 1: Read the relevant sections of `server.py`**

```bash
cd /u/siegeld/dobot
PYTHONPATH=dobot-ros python -m pytest tests/spacemouse/ -v --co -q
```

(just to confirm the tree; not required, but useful)

- [ ] **Step 2: Add `spacemouse` defaults to the settings store**

Edit `dobot-ros/dobot_ros/web/server.py` at `_SETTINGS_DEFAULTS` (currently around line 1625). Add a new group entry inside the dict:

Replace:
```python
    "jog": {"linear_step": 5, "angular_step": 5, "speed": 5, "mode": "user"},
    "pick_strategy": {"active": "simple_top_down"},
}
```

with:
```python
    "jog": {"linear_step": 5, "angular_step": 5, "speed": 5, "mode": "user"},
    "pick_strategy": {"active": "simple_top_down"},
    "spacemouse": {
        "max_velocity_xyz": 80.0,
        "max_velocity_rpy": 30.0,
        "deadband": 0.10,
        "sign_map": [1, 1, 1, 1, 1, 1],
        "max_excursion_xyz": 300.0,
        "max_excursion_rpy": 90.0,
        "idle_auto_disarm_s": 30.0,
        "button_debounce_ms": 150,
        "gripper_force": 20,
    },
}
```

- [ ] **Step 3: Add the reader factory at the bottom of the servo block**

Find the Servo section that ends near line 1960 (`class ServoPatternRequest(BaseModel): ...`). Scroll past the servo endpoints — find a natural insertion point after the servo block and BEFORE any existing VLA or WebSocket sections. Add a new section block.

Append this block *immediately before* `# ── WebSocket ───────────────────────────────────────────────────` (the pre-existing section around line 2135):

```python
# ── SpaceMouse pendant (additive) ─────────────────────────────────
#
# Drives the same ServoTester the /api/servo/* endpoints use, so the robot
# motion path is unchanged. The reader thread converts 3Dconnexion SpaceMouse
# Wireless BT deflections into rate-controlled target offsets and feeds them
# into ServoTester.set_target_offset. Buttons on the puck drive the gripper.
#
# See docs/superpowers/specs/2026-04-17-spacemouse-pendant-design.md.

_spacemouse_reader = None
_spacemouse_lock = threading.Lock()


def _get_spacemouse_reader():
    global _spacemouse_reader
    with _spacemouse_lock:
        if _spacemouse_reader is None:
            from dobot_ros.spacemouse.reader import SpaceMouseReader
            from dobot_ros.spacemouse.hid_state import (
                DEFAULT_SETTINGS, SpaceMouseSettings,
            )
            from dobot_ros.spacemouse.device import (
                find_device, open_device, read_battery_pct,
            )
            import time as _time

            cfg_dict = {**DEFAULT_SETTINGS, **_settings_store.get("spacemouse")}
            try:
                settings = SpaceMouseSettings(**{
                    k: v for k, v in cfg_dict.items()
                    if k in DEFAULT_SETTINGS
                })
            except ValueError as e:
                logging.warning(
                    "spacemouse settings invalid, falling back to defaults: %s", e)
                settings = SpaceMouseSettings(**DEFAULT_SETTINGS)

            _spacemouse_reader = SpaceMouseReader(
                servo_tester=_get_servo_tester(),
                ros_client=_get_client(),
                settings=settings,
                time_fn=_time.monotonic,
                find_device_fn=find_device,
                open_device_fn=open_device,
                battery_fn=read_battery_pct,
            )
            _spacemouse_reader.start_threads(tick_hz=50.0)

            def _on_spacemouse_change(new_cfg: dict):
                try:
                    merged = {**DEFAULT_SETTINGS, **new_cfg}
                    _spacemouse_reader.update_settings(SpaceMouseSettings(**{
                        k: v for k, v in merged.items() if k in DEFAULT_SETTINGS
                    }))
                except Exception:
                    logging.exception("failed to apply spacemouse settings update")

            _settings_store.subscribe("spacemouse", _on_spacemouse_change)
        return _spacemouse_reader


@app.post("/api/spacemouse/arm")
async def spacemouse_arm():
    """Arm the SpaceMouse. Refuses if motion is already active."""
    reader = _get_spacemouse_reader()
    blocked = _acquire_motion("spacemouse")
    if blocked is not None:
        return {"success": False, "error": blocked}
    ok, reason = reader.arm()
    if not ok:
        _release_motion()
        return {"success": False, "error": reason}
    return {"success": True, "state": reader.snapshot().to_dict()}


@app.post("/api/spacemouse/disarm")
async def spacemouse_disarm():
    reader = _get_spacemouse_reader()
    try:
        reader.disarm()
    finally:
        # Only release the lock if we actually own it.
        with _motion_lock:
            global _motion_active
            if _motion_active == "spacemouse":
                _motion_active = None
    return {"success": True, "state": reader.snapshot().to_dict()}


@app.post("/api/spacemouse/estop")
async def spacemouse_estop():
    reader = _get_spacemouse_reader()
    try:
        reader.emergency_stop()
    finally:
        with _motion_lock:
            global _motion_active
            if _motion_active == "spacemouse":
                _motion_active = None
    return {"success": True, "state": reader.snapshot().to_dict()}


@app.get("/api/spacemouse/state")
async def spacemouse_state():
    reader = _get_spacemouse_reader()
    return {"success": True, "state": reader.snapshot().to_dict()}


@app.get("/api/spacemouse/settings")
async def spacemouse_settings_get():
    return {"success": True, "settings": _settings_store.get("spacemouse")}


class SpaceMouseSettingsRequest(BaseModel):
    max_velocity_xyz: Optional[float] = None
    max_velocity_rpy: Optional[float] = None
    deadband: Optional[float] = None
    sign_map: Optional[List[int]] = None
    max_excursion_xyz: Optional[float] = None
    max_excursion_rpy: Optional[float] = None
    idle_auto_disarm_s: Optional[float] = None
    button_debounce_ms: Optional[int] = None
    gripper_force: Optional[int] = None


@app.post("/api/spacemouse/settings")
async def spacemouse_settings_post(req: SpaceMouseSettingsRequest):
    # Validate the merged settings before persisting.
    current = _settings_store.get("spacemouse")
    patch = {k: v for k, v in req.dict().items() if v is not None}
    merged = {**current, **patch}
    try:
        from dobot_ros.spacemouse.hid_state import SpaceMouseSettings as _SS
        _SS(**merged)   # raises ValueError on bad input
    except ValueError as e:
        return {"success": False, "error": str(e)}
    _settings_store.patch("spacemouse", patch)
    return {"success": True, "settings": _settings_store.get("spacemouse")}
```

- [ ] **Step 4: Add the WebSocket endpoint and static-file route**

Inside the existing `# ── WebSocket ───` block (after `ws_state`), add:

```python
@app.websocket("/ws/spacemouse")
async def ws_spacemouse(websocket: WebSocket):
    """10 Hz SpaceMouse state stream (HID echo + armed/offset/anchor)."""
    await websocket.accept()
    try:
        reader = _get_spacemouse_reader()
        while True:
            snap = reader.snapshot().to_dict()
            await websocket.send_json(snap)
            await asyncio.sleep(0.1)   # 10 Hz
    except WebSocketDisconnect:
        pass
    except Exception:
        pass
```

And in the static-file routes section (near the existing `@app.get("/pendant")`), add:

```python
@app.get("/spacemouse")
async def spacemouse_page():
    return FileResponse(str(STATIC_DIR / "spacemouse.html"), headers=_NO_CACHE_HEADERS)
```

- [ ] **Step 5: Smoke-test imports**

```bash
cd /u/siegeld/dobot
PYTHONPATH=dobot-ros python -c "from dobot_ros.web import server; print('ok')"
```

Expected: `ok`. (Module imports without errors; the reader is not instantiated yet.)

- [ ] **Step 6: Run the full spacemouse suite + any unrelated server tests we have**

```bash
cd /u/siegeld/dobot
PYTHONPATH=dobot-ros python -m pytest tests/spacemouse/ -v
```

Expected: all PASS.

- [ ] **Step 7: Commit**

```bash
git add dobot-ros/dobot_ros/web/server.py
git commit -m "Wire SpaceMouse reader into web server with /api/spacemouse/* routes"
```

---

## Task 8: Web UI — SpaceMouse page (HTML + JS + CSS)

**Files:**
- Create: `dobot-ros/dobot_ros/web/static/spacemouse.html`
- Create: `dobot-ros/dobot_ros/web/static/js/spacemouse.js`
- Create: `dobot-ros/dobot_ros/web/static/css/spacemouse.css`
- Modify: `dobot-ros/dobot_ros/web/static/index.html`

- [ ] **Step 1: Create `spacemouse.html`**

File: `dobot-ros/dobot_ros/web/static/spacemouse.html`

```html
<!DOCTYPE html>
<html lang="en" data-bs-theme="dark">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Dobot CR5 — SpaceMouse</title>
  <link href="https://cdn.jsdelivr.net/npm/bootstrap@5.3.3/dist/css/bootstrap.min.css" rel="stylesheet">
  <link href="https://cdn.jsdelivr.net/npm/bootstrap-icons@1.11.3/font/bootstrap-icons.min.css" rel="stylesheet">
  <link href="/static/css/dashboard.css" rel="stylesheet">
  <link href="/static/css/spacemouse.css" rel="stylesheet">
</head>
<body class="bg-body text-body">

<nav class="navbar navbar-expand navbar-dark bg-dark">
  <div class="container-fluid">
    <a class="navbar-brand" href="/"><i class="bi bi-robot"></i> Dobot CR5</a>
    <a class="nav-link" href="/">Dashboard</a>
    <a class="nav-link active" href="/spacemouse">SpaceMouse</a>
  </div>
</nav>

<div class="container py-3" id="sm-root">

  <!-- Status bar -->
  <div class="d-flex align-items-center mb-3">
    <h3 class="mb-0 me-3">SpaceMouse Wireless BT</h3>
    <span id="sm-device-pill" class="badge rounded-pill bg-secondary">
      <i class="bi bi-circle"></i> <span id="sm-device-status">Disconnected</span>
    </span>
    <span id="sm-battery" class="ms-2 text-muted small" style="display:none">
      <i class="bi bi-battery-half"></i> <span id="sm-battery-pct">—</span>%
    </span>
    <span id="sm-armed-pill" class="ms-2 badge rounded-pill bg-secondary" style="display:none">
      <i class="bi bi-record-fill"></i> Armed
    </span>
  </div>

  <!-- Banner area -->
  <div id="sm-banner" class="alert" style="display:none" role="alert"></div>

  <!-- Primary controls -->
  <div class="d-flex gap-3 mb-3">
    <button id="sm-arm-btn" class="btn btn-success btn-lg flex-fill" disabled>
      <i class="bi bi-power"></i> Arm
    </button>
    <button id="sm-disarm-btn" class="btn btn-secondary btn-lg flex-fill" style="display:none">
      <i class="bi bi-power"></i> Disarm
    </button>
    <button id="sm-estop-btn" class="btn btn-danger btn-lg flex-fill">
      <i class="bi bi-octagon-fill"></i> E-STOP (Esc)
    </button>
  </div>

  <!-- Anchor / Offset readout -->
  <div class="row g-3 mb-3">
    <div class="col-md-6">
      <div class="card">
        <div class="card-header">Anchor pose</div>
        <div class="card-body font-monospace small" id="sm-anchor">
          <em class="text-muted">not armed</em>
        </div>
      </div>
    </div>
    <div class="col-md-6">
      <div class="card">
        <div class="card-header">Offset from anchor</div>
        <div class="card-body font-monospace small" id="sm-offset">
          X <span data-offset="0">0.0</span> mm
          &nbsp;&nbsp; Y <span data-offset="1">0.0</span> mm
          &nbsp;&nbsp; Z <span data-offset="2">0.0</span> mm<br>
          RX <span data-offset="3">0.0</span>°
          &nbsp;&nbsp; RY <span data-offset="4">0.0</span>°
          &nbsp;&nbsp; RZ <span data-offset="5">0.0</span>°
        </div>
      </div>
    </div>
  </div>

  <!-- Live HID -->
  <div class="card mb-3">
    <div class="card-header d-flex justify-content-between">
      <span>Live HID (always visible — this is your test page)</span>
      <small class="text-muted" id="sm-event-age">— ms</small>
    </div>
    <div class="card-body">
      <div class="sm-axis-bars">
        <div class="sm-axis" data-axis="0"><span class="sm-label">TX</span><div class="sm-bar-track"><div class="sm-bar-fill"></div></div><span class="sm-val">0.00</span></div>
        <div class="sm-axis" data-axis="1"><span class="sm-label">TY</span><div class="sm-bar-track"><div class="sm-bar-fill"></div></div><span class="sm-val">0.00</span></div>
        <div class="sm-axis" data-axis="2"><span class="sm-label">TZ</span><div class="sm-bar-track"><div class="sm-bar-fill"></div></div><span class="sm-val">0.00</span></div>
        <div class="sm-axis" data-axis="3"><span class="sm-label">RX</span><div class="sm-bar-track"><div class="sm-bar-fill"></div></div><span class="sm-val">0.00</span></div>
        <div class="sm-axis" data-axis="4"><span class="sm-label">RY</span><div class="sm-bar-track"><div class="sm-bar-fill"></div></div><span class="sm-val">0.00</span></div>
        <div class="sm-axis" data-axis="5"><span class="sm-label">RZ</span><div class="sm-bar-track"><div class="sm-bar-fill"></div></div><span class="sm-val">0.00</span></div>
      </div>
      <div class="d-flex gap-4 mt-3">
        <div class="sm-btn-indicator" data-btn="0">
          <span class="sm-dot"></span> BTN 0 (Close)
        </div>
        <div class="sm-btn-indicator" data-btn="1">
          <span class="sm-dot"></span> BTN 1 (Open)
        </div>
        <small class="text-muted align-self-center" id="sm-button-note" style="display:none">
          no gripper action while disarmed
        </small>
      </div>
    </div>
  </div>

  <!-- Settings -->
  <div class="card">
    <div class="card-header">
      <button class="btn btn-link text-decoration-none" data-bs-toggle="collapse" data-bs-target="#sm-settings">
        <i class="bi bi-sliders"></i> Settings
      </button>
    </div>
    <div id="sm-settings" class="collapse">
      <div class="card-body">
        <form id="sm-settings-form" class="row g-2 small">
          <label class="col-md-4">Max vel XYZ (mm/s)<input name="max_velocity_xyz" type="number" step="1" class="form-control form-control-sm"></label>
          <label class="col-md-4">Max vel RPY (°/s)<input name="max_velocity_rpy" type="number" step="1" class="form-control form-control-sm"></label>
          <label class="col-md-4">Deadband<input name="deadband" type="number" step="0.01" class="form-control form-control-sm"></label>
          <label class="col-md-4">Max excursion XYZ (mm)<input name="max_excursion_xyz" type="number" step="1" class="form-control form-control-sm"></label>
          <label class="col-md-4">Max excursion RPY (°)<input name="max_excursion_rpy" type="number" step="1" class="form-control form-control-sm"></label>
          <label class="col-md-4">Idle auto-disarm (s)<input name="idle_auto_disarm_s" type="number" step="1" class="form-control form-control-sm"></label>
          <label class="col-md-6">Button debounce (ms)<input name="button_debounce_ms" type="number" step="10" class="form-control form-control-sm"></label>
          <label class="col-md-6">Gripper force (%)<input name="gripper_force" type="number" step="5" min="0" max="100" class="form-control form-control-sm"></label>
          <fieldset class="col-12">
            <legend class="small text-muted">Sign map (per-axis ±1)</legend>
            <div id="sm-sign-map" class="d-flex gap-2">
              <!-- JS fills in 6 toggles -->
            </div>
          </fieldset>
          <div class="col-12"><button type="submit" class="btn btn-primary btn-sm">Save</button></div>
        </form>
      </div>
    </div>
  </div>

</div>

<script src="https://cdn.jsdelivr.net/npm/bootstrap@5.3.3/dist/js/bootstrap.bundle.min.js"></script>
<script src="/static/js/spacemouse.js"></script>
</body>
</html>
```

- [ ] **Step 2: Create `spacemouse.css`**

File: `dobot-ros/dobot_ros/web/static/css/spacemouse.css`

```css
.sm-axis { display: grid; grid-template-columns: 40px 1fr 60px; align-items: center; gap: 10px; margin-bottom: 8px; }
.sm-label { font-weight: 600; font-family: monospace; }
.sm-bar-track { position: relative; height: 16px; background: #333; border-radius: 4px; overflow: hidden; }
.sm-bar-fill { position: absolute; top: 0; bottom: 0; left: 50%; background: #0d6efd; transition: width 0.05s linear, left 0.05s linear; }
.sm-val { text-align: right; font-family: monospace; font-size: 0.9em; }
.sm-axis[data-deadband="true"] .sm-bar-fill { background: #6c757d; }

.sm-btn-indicator { display: flex; align-items: center; gap: 6px; font-family: monospace; }
.sm-dot { width: 14px; height: 14px; border-radius: 50%; background: #444; display: inline-block; box-shadow: inset 0 0 3px rgba(0,0,0,0.6); }
.sm-btn-indicator[data-active="true"] .sm-dot { background: #dc3545; box-shadow: 0 0 8px #dc3545; }

#sm-arm-btn:disabled { opacity: 0.6; }

.sm-sign-toggle { display: inline-flex; align-items: center; gap: 2px; }
.sm-sign-toggle button { width: 38px; font-family: monospace; }
```

- [ ] **Step 3: Create `spacemouse.js`**

File: `dobot-ros/dobot_ros/web/static/js/spacemouse.js`

```javascript
(function () {
  'use strict';

  const SETTING_KEYS = [
    'max_velocity_xyz', 'max_velocity_rpy', 'deadband',
    'max_excursion_xyz', 'max_excursion_rpy', 'idle_auto_disarm_s',
    'button_debounce_ms', 'gripper_force',
  ];

  const root = document.getElementById('sm-root');
  const devicePill = document.getElementById('sm-device-pill');
  const deviceStatus = document.getElementById('sm-device-status');
  const batteryWrap = document.getElementById('sm-battery');
  const batteryPct = document.getElementById('sm-battery-pct');
  const armedPill = document.getElementById('sm-armed-pill');
  const banner = document.getElementById('sm-banner');
  const armBtn = document.getElementById('sm-arm-btn');
  const disarmBtn = document.getElementById('sm-disarm-btn');
  const estopBtn = document.getElementById('sm-estop-btn');
  const anchorEl = document.getElementById('sm-anchor');
  const eventAgeEl = document.getElementById('sm-event-age');
  const buttonNote = document.getElementById('sm-button-note');
  const signMapEl = document.getElementById('sm-sign-map');
  const settingsForm = document.getElementById('sm-settings-form');

  // ── WebSocket ─────────────────────────────────────────────────
  let currentSignMap = [1, 1, 1, 1, 1, 1];

  function connectWs() {
    const proto = location.protocol === 'https:' ? 'wss' : 'ws';
    const ws = new WebSocket(`${proto}://${location.host}/ws/spacemouse`);
    ws.onmessage = (ev) => {
      try { render(JSON.parse(ev.data)); } catch (e) { console.error(e); }
    };
    ws.onclose = () => setTimeout(connectWs, 1000);
    ws.onerror = () => {};
  }

  function render(s) {
    // Device pill
    const status = s.device_status || 'disconnected';
    deviceStatus.textContent = status.replace('_', ' ');
    devicePill.classList.remove('bg-secondary', 'bg-success', 'bg-warning', 'bg-danger');
    if (status === 'connected') {
      devicePill.classList.add('bg-success');
    } else if (status === 'lost') {
      devicePill.classList.add('bg-warning');
    } else if (status === 'permission_denied') {
      devicePill.classList.add('bg-danger');
    } else {
      devicePill.classList.add('bg-secondary');
    }

    // Battery
    if (typeof s.battery_pct === 'number') {
      batteryWrap.style.display = '';
      batteryPct.textContent = s.battery_pct;
    } else {
      batteryWrap.style.display = 'none';
    }

    // Armed indicator + button swap
    if (s.armed) {
      armedPill.style.display = '';
      armBtn.style.display = 'none';
      disarmBtn.style.display = '';
      buttonNote.style.display = 'none';
    } else {
      armedPill.style.display = 'none';
      armBtn.style.display = '';
      disarmBtn.style.display = 'none';
      buttonNote.style.display = '';
    }

    // Arm button gating (client-side — server re-validates on POST).
    armBtn.disabled = (status !== 'connected');

    // Axis bars
    for (let i = 0; i < 6; i++) {
      const row = root.querySelector(`.sm-axis[data-axis="${i}"]`);
      if (!row) continue;
      const v = (s.axes && s.axes[i]) || 0;
      const fill = row.querySelector('.sm-bar-fill');
      const val = row.querySelector('.sm-val');
      val.textContent = v.toFixed(2);
      // Bars grow from the center: positive → right, negative → left.
      const pct = Math.min(100, Math.abs(v) * 100);
      if (v >= 0) {
        fill.style.left = '50%';
        fill.style.width = (pct / 2) + '%';
      } else {
        fill.style.left = (50 - pct / 2) + '%';
        fill.style.width = (pct / 2) + '%';
      }
      row.setAttribute('data-deadband', Math.abs(v) < 0.01 ? 'true' : 'false');
    }

    // Buttons
    for (let b = 0; b < 2; b++) {
      const el = root.querySelector(`.sm-btn-indicator[data-btn="${b}"]`);
      if (!el) continue;
      el.setAttribute('data-active', (s.buttons && s.buttons[b]) ? 'true' : 'false');
    }

    // Anchor + offset
    if (s.anchor && s.anchor.length === 6) {
      const [x, y, z, rx, ry, rz] = s.anchor;
      anchorEl.innerHTML =
        `X ${x.toFixed(1)} &nbsp; Y ${y.toFixed(1)} &nbsp; Z ${z.toFixed(1)}<br>` +
        `RX ${rx.toFixed(1)} &nbsp; RY ${ry.toFixed(1)} &nbsp; RZ ${rz.toFixed(1)}`;
    } else {
      anchorEl.innerHTML = '<em class="text-muted">not armed</em>';
    }
    for (let i = 0; i < 6; i++) {
      const el = root.querySelector(`[data-offset="${i}"]`);
      if (el) el.textContent = ((s.offset && s.offset[i]) || 0).toFixed(1);
    }

    eventAgeEl.textContent = ((s.last_event_age_ms || 0).toFixed(0)) + ' ms';

    // Banners
    if (s.error && status !== 'connected') {
      banner.style.display = '';
      banner.className = 'alert alert-warning';
      banner.textContent = s.error;
    } else {
      banner.style.display = 'none';
    }
  }

  // ── Buttons ───────────────────────────────────────────────────
  async function post(path, body) {
    const res = await fetch(path, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: body ? JSON.stringify(body) : '{}',
    });
    return res.json();
  }

  armBtn.addEventListener('click', async () => {
    const r = await post('/api/spacemouse/arm');
    if (!r.success) showToast(r.error || 'arm failed');
  });
  disarmBtn.addEventListener('click', () => post('/api/spacemouse/disarm'));
  estopBtn.addEventListener('click', () => post('/api/spacemouse/estop'));
  document.addEventListener('keydown', (e) => {
    if (e.key === 'Escape') post('/api/spacemouse/estop');
  });

  function showToast(msg) {
    banner.style.display = '';
    banner.className = 'alert alert-danger';
    banner.textContent = msg;
    setTimeout(() => { if (banner.textContent === msg) banner.style.display = 'none'; }, 4000);
  }

  // ── Settings ──────────────────────────────────────────────────
  async function loadSettings() {
    const r = await fetch('/api/spacemouse/settings').then(r => r.json());
    if (!r.success) return;
    for (const k of SETTING_KEYS) {
      const el = settingsForm.querySelector(`[name="${k}"]`);
      if (el && r.settings[k] !== undefined) el.value = r.settings[k];
    }
    currentSignMap = (r.settings.sign_map && r.settings.sign_map.slice()) || [1,1,1,1,1,1];
    renderSignMap();
  }

  function renderSignMap() {
    const labels = ['X', 'Y', 'Z', 'RX', 'RY', 'RZ'];
    signMapEl.innerHTML = '';
    for (let i = 0; i < 6; i++) {
      const wrap = document.createElement('span');
      wrap.className = 'sm-sign-toggle';
      const btn = document.createElement('button');
      btn.type = 'button';
      btn.className = 'btn btn-sm ' + (currentSignMap[i] > 0 ? 'btn-outline-success' : 'btn-outline-danger');
      btn.textContent = (currentSignMap[i] > 0 ? '+' : '−') + labels[i];
      btn.addEventListener('click', () => {
        currentSignMap[i] *= -1;
        renderSignMap();
      });
      wrap.appendChild(btn);
      signMapEl.appendChild(wrap);
    }
  }

  settingsForm.addEventListener('submit', async (ev) => {
    ev.preventDefault();
    const body = {};
    for (const k of SETTING_KEYS) {
      const v = settingsForm.querySelector(`[name="${k}"]`).value;
      if (v !== '') body[k] = Number(v);
    }
    body.sign_map = currentSignMap.slice();
    const r = await post('/api/spacemouse/settings', body);
    if (!r.success) showToast(r.error || 'save failed');
  });

  // ── Boot ──────────────────────────────────────────────────────
  loadSettings();
  connectWs();
})();
```

- [ ] **Step 4: Add nav link from `index.html`**

Find the existing nav section in `dobot-ros/dobot_ros/web/static/index.html`. Add a link to `/spacemouse` next to any existing nav links. Open the file, locate `<nav` or the existing `Pendant` link, and add an adjacent `<a href="/spacemouse" class="nav-link">SpaceMouse</a>` anchor. (Exact insertion depends on the file's current structure; inspect and match existing style.)

Example: read existing nav first, then add the link matching the existing pattern.

```bash
cd /u/siegeld/dobot
grep -n 'href="/pendant"' dobot-ros/dobot_ros/web/static/index.html
```

Use the existing pendant link as a template — copy its enclosing element and change `/pendant` → `/spacemouse` and "Pendant" → "SpaceMouse".

- [ ] **Step 5: Manual sanity check — open in browser locally**

```bash
cd /u/siegeld/dobot
PYTHONPATH=dobot-ros python -c "import dobot_ros.web.server; print('ok')"
```

If you have the container running, rebuild is not required — the `dobot-ros/` volume is live-mounted. Reload `http://localhost:7070/spacemouse` in the browser. You should see the layout even without a device (device pill says "disconnected", bars idle).

- [ ] **Step 6: Commit**

```bash
git add dobot-ros/dobot_ros/web/static/spacemouse.html \
        dobot-ros/dobot_ros/web/static/js/spacemouse.js \
        dobot-ros/dobot_ros/web/static/css/spacemouse.css \
        dobot-ros/dobot_ros/web/static/index.html
git commit -m "Add SpaceMouse web UI page with live HID echo and settings"
```

---

## Task 9: Docker + env wiring (device passthrough, evdev dep)

**Files:**
- Modify: `docker-compose.yml`
- Modify: `docker/Dockerfile`
- Create or modify: `.env.example`
- Modify: `docker/entrypoint.sh` *(only if required — see step)*

- [ ] **Step 1: Add `python3-evdev` to Dockerfile**

Open `docker/Dockerfile` and find the `apt-get install` line (there will be one or more). Append `python3-evdev` to the package list. Example diff concept:

```diff
  apt-get install -y \
      ros-jazzy-ros-base \
-     python3-modbus-tk
+     python3-modbus-tk \
+     python3-evdev
```

Exact layout depends on current file — match the existing style.

- [ ] **Step 2: Add device passthrough to `docker-compose.yml`**

Open `docker-compose.yml`. In the base `dobot` service block, add two keys just below `tty: true`:

```yaml
    devices:
      - /dev/input:/dev/input:ro
    group_add:
      - "${INPUT_GID:-104}"
```

Final base block looks like:

```yaml
  dobot:
    build:
      context: .
      dockerfile: docker/Dockerfile
    network_mode: host
    environment:
      - ROS_DOMAIN_ID=0
      - RMW_IMPLEMENTATION=rmw_fastrtps_cpp
      - FASTRTPS_DEFAULT_PROFILES_FILE=/fastrtps_profile.xml
    volumes:
      - ./dobot-ros:/dobot-ros
      - ./docker/fastrtps_profile.xml:/fastrtps_profile.xml:ro
      - dobot-shared:/tmp/dobot-shared
    stdin_open: true
    tty: true
    devices:
      - /dev/input:/dev/input:ro
    group_add:
      - "${INPUT_GID:-104}"
```

All `extends: dobot` services (`dobot-driver`, `dobot-cli`, `dobot-gripper`, `dobot-web`) inherit these.

- [ ] **Step 3: Add `INPUT_GID` to `.env.example`**

Check if `.env.example` exists:

```bash
ls -la /u/siegeld/dobot/.env.example 2>/dev/null
```

If it does not exist, create it:

File: `.env.example`

```env
# Robot
ROBOT_IP=192.168.5.1
ROBOT_TYPE=cr5

# SpaceMouse device passthrough:
# The 'input' group GID on the host (required so the container can read
# /dev/input/event* for the SpaceMouse). Discover with:
#   getent group input | cut -d: -f3
# Fedora default: 104. Ubuntu default: 104. Set here if yours differs.
INPUT_GID=104
```

If `.env.example` already exists, append (or merge) just the SpaceMouse block — don't duplicate `ROBOT_IP`/`ROBOT_TYPE`.

- [ ] **Step 4: Rebuild the container image**

```bash
cd /u/siegeld/dobot
docker compose build dobot
```

Expected: successful build; no errors installing `python3-evdev`.

- [ ] **Step 5: Verify evdev import works inside the container**

```bash
cd /u/siegeld/dobot
docker compose run --rm dobot python3 -c "import evdev; print(evdev.list_devices()[:5])"
```

Expected: prints a list of `/dev/input/event*` paths. If it prints an empty list, check `INPUT_GID` against `getent group input | cut -d: -f3`.

- [ ] **Step 6: Verify the SpaceMouse is discoverable from the container**

With the SpaceMouse paired and connected on the host:

```bash
cd /u/siegeld/dobot
docker compose run --rm dobot python3 -c "
from dobot_ros.spacemouse.device import find_device
print(find_device())
"
```

Expected: prints `/dev/input/event21` (or whichever event node the BT uhid layer assigns). If it prints `None`, re-check pairing with `bluetoothctl info CD:D7:40:ED:B2:76` on the host and confirm `Connected: yes`.

- [ ] **Step 7: Commit**

```bash
git add docker-compose.yml docker/Dockerfile .env.example
git commit -m "Add SpaceMouse device passthrough and python3-evdev to container"
```

---

## Task 10: Manual bring-up checklist + diagnostic card on servo page

**Files:**
- Create: `docs/spacemouse-bringup.md`
- Modify: `dobot-ros/dobot_ros/web/static/index.html` (small diagnostic card)

- [ ] **Step 1: Create bring-up doc**

File: `docs/spacemouse-bringup.md`

```markdown
# SpaceMouse Pendant — Bring-up Checklist

Run this the first time after install, after any BT pairing change, and
any time the puck feels "wrong."

## Pre-flight (without robot motion)

1. **Pair on host** (once): `bluetoothctl` → `scan on` → `pair CD:D7:40:ED:B2:76` → `trust` → `connect`.
2. Confirm connection: `bluetoothctl info CD:D7:40:ED:B2:76` shows `Connected: yes`.
3. Rebuild container if `python3-evdev` wasn't installed yet: `docker compose build dobot`.
4. Verify `INPUT_GID` in `.env` matches the host's `input` group:
   `getent group input | cut -d: -f3` — should match. Fedora/Ubuntu default: 104.
5. Start the system: `./startup.sh`.
6. Open `http://localhost:7070/spacemouse`. The device pill should be green
   ("connected") and a battery percentage visible.
7. With the robot **disabled**: move each axis on the puck; confirm all six
   bars respond in the expected direction. Press each button; confirm both
   dots light up red. No robot motion expected.

## In-flight (robot enabled, armed)

8. Enable the robot from the main dashboard.
9. Click **Arm**. The page should flip to show an Armed pill, a Disarm
   button, and an Anchor pose. No motion yet.
10. Gently push +X (right). Robot should move +X in the base frame. Release
    — motion stops instantly.
11. Walk each axis: +X/-X, +Y/-Y, +Z/-Z, +RX/-RX, +RY/-RY, +RZ/-RZ.
    Any axis that moves the wrong way → open Settings → flip its sign toggle → Save.
12. Push full-deflection +X and hold. Watch the "Offset from anchor" — it
    climbs to 300 mm and stops. Release. Good.
13. Walk away for 30 seconds with the puck at rest. The page auto-disarms.
14. Re-arm. Press the left puck button (BTN 0). Gripper closes. Press the
    right puck button (BTN 1). Gripper opens.
15. Disarm, press puck buttons — confirm gripper does nothing (safety).
16. Re-arm. Hit `Esc`. E-stop fires, robot halts, page shows "emergency stop".

## Failure drills

- Turn the puck off mid-jog → page should immediately disarm + show "lost".
  Turn puck back on → pill goes green; re-arm is manual.
- Restart the `dobot-web` container; verify the page reconnects.

## Known limits

- World-frame only (no tool-frame jog in v1).
- No CLI; use the web page.
- One puck at a time.
```

- [ ] **Step 2: Add a diagnostic echo card to the servo tester section of `index.html`**

Find the servo tester UI block in `dobot-ros/dobot_ros/web/static/index.html` (search for `servo` / `servo-tester` / `ServoTester`). Insert a small read-only card showing the SpaceMouse state using the same `/ws/spacemouse` stream. Minimum content:

```html
<!-- SpaceMouse diagnostic echo (read-only) -->
<div class="card mt-2 sm-diag" id="sm-diag-card" style="display:none">
  <div class="card-body p-2">
    <div class="d-flex justify-content-between align-items-center">
      <small class="text-muted">SpaceMouse: <span id="sm-diag-status">—</span></small>
      <small class="text-muted" id="sm-diag-btns">● ●</small>
    </div>
    <div class="sm-axis-bars sm-diag-bars">
      <!-- 6 thinner bars -->
      <div class="sm-axis sm-diag-axis" data-axis="0"><span class="sm-label">TX</span><div class="sm-bar-track"><div class="sm-bar-fill"></div></div></div>
      <div class="sm-axis sm-diag-axis" data-axis="1"><span class="sm-label">TY</span><div class="sm-bar-track"><div class="sm-bar-fill"></div></div></div>
      <div class="sm-axis sm-diag-axis" data-axis="2"><span class="sm-label">TZ</span><div class="sm-bar-track"><div class="sm-bar-fill"></div></div></div>
      <div class="sm-axis sm-diag-axis" data-axis="3"><span class="sm-label">RX</span><div class="sm-bar-track"><div class="sm-bar-fill"></div></div></div>
      <div class="sm-axis sm-diag-axis" data-axis="4"><span class="sm-label">RY</span><div class="sm-bar-track"><div class="sm-bar-fill"></div></div></div>
      <div class="sm-axis sm-diag-axis" data-axis="5"><span class="sm-label">RZ</span><div class="sm-bar-track"><div class="sm-bar-fill"></div></div></div>
    </div>
  </div>
</div>
<link href="/static/css/spacemouse.css" rel="stylesheet">
<script>
(function () {
  const card = document.getElementById('sm-diag-card');
  if (!card) return;
  function apply(s) {
    card.style.display = '';
    document.getElementById('sm-diag-status').textContent = s.device_status + (s.armed ? ' · armed' : '');
    document.getElementById('sm-diag-btns').textContent =
      (s.buttons && s.buttons[0] ? '●' : '○') + ' ' + (s.buttons && s.buttons[1] ? '●' : '○');
    for (let i = 0; i < 6; i++) {
      const row = card.querySelector(`.sm-diag-axis[data-axis="${i}"]`);
      if (!row) continue;
      const v = (s.axes && s.axes[i]) || 0;
      const fill = row.querySelector('.sm-bar-fill');
      const pct = Math.min(100, Math.abs(v) * 100);
      if (v >= 0) { fill.style.left = '50%'; fill.style.width = (pct/2) + '%'; }
      else { fill.style.left = (50 - pct/2) + '%'; fill.style.width = (pct/2) + '%'; }
    }
  }
  function connect() {
    const proto = location.protocol === 'https:' ? 'wss' : 'ws';
    const ws = new WebSocket(`${proto}://${location.host}/ws/spacemouse`);
    ws.onmessage = (ev) => { try { apply(JSON.parse(ev.data)); } catch (e) {} };
    ws.onclose = () => setTimeout(connect, 1000);
  }
  connect();
})();
</script>
```

Insertion point: inside the servo-tester panel section of `index.html`. If the exact markup is different, adapt to fit — the key is that the card sits near the servo tester controls and uses the same WebSocket stream.

- [ ] **Step 3: Smoke test**

Reload `http://localhost:7070/` — if a SpaceMouse is connected, the diagnostic card in the servo-tester section should show "connected" and the axis bars should respond when you push the puck.

- [ ] **Step 4: Commit**

```bash
git add docs/spacemouse-bringup.md \
        dobot-ros/dobot_ros/web/static/index.html
git commit -m "Add SpaceMouse bring-up doc and diagnostic echo on servo tester page"
```

---

## Task 11: Final full-stack verification

**Files:** none modified (verification only).

- [ ] **Step 1: Run the entire test suite**

```bash
cd /u/siegeld/dobot
PYTHONPATH=dobot-ros python -m pytest tests/ -v
```

Expected: everything green. The new `tests/spacemouse/` suite adds ~25 tests; no other suite's results change.

- [ ] **Step 2: Verify no imports were broken in `server.py`**

```bash
cd /u/siegeld/dobot
PYTHONPATH=dobot-ros python -c "from dobot_ros.web import server; print(list(server.app.router.routes)[-5:])"
```

Expected: prints at least one `/api/spacemouse/*` route in the tail.

- [ ] **Step 3: Hardware smoke — walk the bring-up checklist**

Follow `docs/spacemouse-bringup.md` end-to-end. Report any deviation.

- [ ] **Step 4: Confirm all the "untouched" files are untouched**

```bash
cd /u/siegeld/dobot
git log --name-only --since="7 days" -- \
  dobot-ros/dobot_ros/ros_client.py \
  dobot-ros/dobot_ros/servo/tester.py \
  dobot-ros/dobot_ros/servo/patterns.py \
  dobot-ros/dobot_ros/cli.py \
  dobot-ros/dobot_ros/shell.py \
  dobot-ros/dobot_ros/gripper_node.py \
  dobot-ros/dobot_ros/pick.py \
  dobot-ros/dobot_ros/strategies \
  dobot-ros/dobot_ros/vla \
  dobot-ros/dobot_ros/validation.py \
  dobot-ros/dobot_ros/driver.py \
  dobot-ros/dobot_ros/config.py \
  2>&1 | head -30
```

Expected: no commits touching those files from this feature's branch.

- [ ] **Step 5: Tag and write a release note (optional)**

If the user uses branch-per-feature: merge to master with a short message describing the new `/spacemouse` page, puck-button gripper control, and additive status.

---

## Self-review notes

**Spec coverage**: arm/disarm gates (Task 7), integration tick math (Task 5), excursion clamp (Task 5), workspace clamp via ServoTester (inherited — no task needed, no change), idle auto-disarm (Task 5), device disconnect path (Tasks 5 & 6), E-stop (Tasks 3 & 7), battery (Tasks 1 & 2), device discovery by vendor/product (Task 2), axis normalization via `absinfo.max` (Task 6), default axis mapping (Tasks 1 & 4), sign map (Tasks 1 & 4), button mapping (Task 4), debounce (Task 4), UI Pendant page (Task 8), settings persistence + live subscription (Tasks 1 & 7), Docker passthrough (Task 9), Dockerfile dep (Task 9), bring-up checklist (Task 10), diagnostic echo on servo page (Task 10), no changes to the "untouched" list (Task 11 verifies).

**Placeholder scan**: every step has concrete code, concrete commands, and an expected outcome. The only "adapt to existing markup" notes are in Task 8 step 4 and Task 10 step 2 (adding a nav link and embedding a card in `index.html`) — these are genuinely dependent on the current file structure, which the engineer will read before editing, and include exact grep one-liners to locate the insertion point. No "TBD", "TODO", or "similar to".

**Type consistency**: `SpaceMouseReader`, `HidState`, `SpaceMouseSettings`, `DeviceStatus` names are consistent across all tasks. `set_target_offset(offset)` matches the existing `ServoTester` method signature. `gripper_move(position, force, speed, wait)` matches the existing `DobotRosClient` signature (see `ros_client.py:654`).
