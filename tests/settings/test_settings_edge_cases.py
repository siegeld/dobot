"""Edge-case tests for settings store — import validation, temp cleanup, corruption."""

import json
from pathlib import Path

import pytest

from dobot_ros.web.settings_store import SettingsStore, slugify


DEFAULTS = {"servo": {"gain": 500}, "vla": {}, "gripper": {"force": 20}}


def test_import_rejects_non_dict_settings(tmp_path):
    store = SettingsStore(tmp_path, defaults=DEFAULTS)
    with pytest.raises(ValueError, match="must be a JSON object"):
        store.import_bundle({"name": "bad", "settings": "not a dict"})


def test_import_rejects_non_dict_group(tmp_path):
    store = SettingsStore(tmp_path, defaults=DEFAULTS)
    with pytest.raises(ValueError, match="must be a JSON object"):
        store.import_bundle({"name": "bad", "settings": {"servo": [1, 2, 3]}})


def test_import_rejects_oversized_settings(tmp_path):
    store = SettingsStore(tmp_path, defaults=DEFAULTS)
    # Build a settings block > 500KB.
    big = {"big_group": {"key": "x" * 600_000}}
    with pytest.raises(ValueError, match="too large"):
        store.import_bundle({"name": "huge", "settings": big})


def test_temp_file_cleanup_on_startup(tmp_path):
    # Create some orphaned .tmp files.
    (tmp_path / "settings.json.abc123.tmp").write_text("{}")
    (tmp_path / "other.tmp").write_text("x")
    store = SettingsStore(tmp_path, defaults=DEFAULTS)
    # Temp files should be cleaned up.
    assert not list(tmp_path.glob("*.tmp"))


def test_corrupted_last_settings_falls_back_to_defaults(tmp_path):
    (tmp_path / "last_settings.json").write_text("NOT VALID JSON {{{")
    store = SettingsStore(tmp_path, defaults=DEFAULTS)
    # Should fall back to defaults instead of crashing.
    assert store.get("servo") == DEFAULTS["servo"]


def test_empty_last_settings_uses_defaults(tmp_path):
    (tmp_path / "last_settings.json").write_text("{}")
    store = SettingsStore(tmp_path, defaults=DEFAULTS)
    assert store.get("servo") == DEFAULTS["servo"]


def test_slugify_strips_dangerous_chars():
    assert slugify("../../etc/passwd") == "etcpasswd"
    assert slugify("<script>alert(1)</script>") == "scriptalert1script"
    assert slugify("   ") == "unnamed"
    assert slugify("") == "unnamed"


def test_concurrent_patch_and_flush(tmp_path):
    """Rapid patches + flush should not lose data."""
    store = SettingsStore(tmp_path, defaults=DEFAULTS)
    for i in range(50):
        store.patch("servo", {"gain": i})
    store.flush()
    store.shutdown()
    # Re-read.
    store2 = SettingsStore(tmp_path, defaults=DEFAULTS)
    assert store2.get("servo")["gain"] == 49
