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


def test_read_battery_pct_parses_sysfs(tmp_path: Path):
    power = tmp_path / "power_supply"
    batt = power / "hid-CD:D7:40:ED:B2:76-battery"
    batt.mkdir(parents=True)
    (batt / "capacity").write_text("97\n")
    (power / "AC0").mkdir()

    assert dev_mod.read_battery_pct_in(power, match_substring="SpaceMouse") is None

    (batt / "model_name").write_text("SpaceMouse Wireless BT\n")
    assert dev_mod.read_battery_pct_in(power, match_substring="SpaceMouse") == 97


def test_read_battery_pct_returns_none_when_missing(tmp_path: Path):
    assert dev_mod.read_battery_pct_in(tmp_path, match_substring="SpaceMouse") is None
