"""Device discovery and battery readout for the SpaceMouse pendant.

The real evdev module is imported lazily so this file can be imported in
test environments that don't have python3-evdev installed — tests replace
`device.evdev` with a fake before exercising the functions.
"""
from __future__ import annotations

import glob
import logging
from pathlib import Path
from typing import List, Optional

from dobot_ros.spacemouse.hid_state import VENDOR_ID, PRODUCT_ID

log = logging.getLogger(__name__)

try:  # pragma: no cover - environment dependent
    import evdev
except ImportError:  # pragma: no cover
    evdev = None  # tests monkeypatch this


_DEFAULT_POWER_SUPPLY = Path("/sys/class/power_supply")


def _candidate_event_paths() -> List[str]:
    """Glob /dev/input/event* directly — bypasses evdev.list_devices() which
    requires write access (python-evdev's is_device() checks R_OK | W_OK, and
    we mount /dev/input read-only in Docker)."""
    if evdev is not None and hasattr(evdev, "list_devices"):
        # In tests, our FakeEvdev.list_devices() returns the scripted set —
        # prefer it so test fixtures keep working.
        try:
            paths = list(evdev.list_devices())
            if paths:
                return paths
        except Exception:
            pass
    return sorted(glob.glob("/dev/input/event*"))


def find_device() -> Optional[str]:
    """Return the /dev/input/event* path for the SpaceMouse, or None."""
    if evdev is None:
        log.warning("evdev not available; cannot scan for SpaceMouse")
        return None
    try:
        paths = _candidate_event_paths()
    except Exception as e:
        log.warning("scanning /dev/input/event* failed: %s", e)
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
