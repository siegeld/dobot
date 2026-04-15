"""Tests for the SettingsStore: CRUD, subscribers, migration, safety-gated load."""

from __future__ import annotations

import json
import time
from pathlib import Path

import pytest

from dobot_ros.web.settings_store import (
    SettingsStore, SettingsBlocked, SettingsConflict, SettingsNotFound,
    slugify,
)


DEFAULTS = {
    "servo": {"servo_rate_hz": 30.0, "t": 0.05, "aheadtime": 50.0, "gain": 500.0},
    "vla": {"server_url": "http://gpurbr2:7071", "model_rate_hz": 10.0},
    "table_plane": {},
    "camera_view": {},
    "gripper": {"force": 20},
    "jog": {"linear_step": 5, "angular_step": 5, "speed": 5, "mode": "user"},
}


# ── slugify ─────────────────────────────────────────────────────────
def test_slugify_basics():
    assert slugify("My Setting!") == "my_setting"
    assert slugify("good tuning AG-105") == "good_tuning_ag-105"
    assert slugify("   ") == "unnamed"
    assert slugify("") == "unnamed"
    assert slugify("a" * 200, maxlen=20).__len__() == 20


# ── Core lifecycle ─────────────────────────────────────────────────
def test_fresh_store_uses_defaults(tmp_path):
    store = SettingsStore(tmp_path, defaults=DEFAULTS)
    assert store.get("servo") == DEFAULTS["servo"]
    assert store.get("gripper") == DEFAULTS["gripper"]


def test_patch_persists_and_reloads(tmp_path):
    store = SettingsStore(tmp_path, defaults=DEFAULTS)
    store.patch("servo", {"gain": 800})
    store.flush()
    assert store.get("servo")["gain"] == 800
    store.shutdown()

    # Re-open — should see the write-through.
    store2 = SettingsStore(tmp_path, defaults=DEFAULTS)
    assert store2.get("servo")["gain"] == 800
    # Other defaults preserved.
    assert store2.get("servo")["t"] == DEFAULTS["servo"]["t"]


def test_subscribers_fired_immediately_and_on_change(tmp_path):
    store = SettingsStore(tmp_path, defaults=DEFAULTS)
    seen = []

    def cb(cfg):
        seen.append(dict(cfg))

    store.subscribe("servo", cb)
    # Immediate fire.
    assert len(seen) == 1
    assert seen[0]["gain"] == 500.0

    store.patch("servo", {"gain": 700})
    assert len(seen) == 2
    assert seen[1]["gain"] == 700

    # Unrelated group does not notify servo subscriber.
    store.patch("gripper", {"force": 40})
    assert len(seen) == 2


# ── Saved bundles ──────────────────────────────────────────────────
def test_save_current_and_list(tmp_path):
    store = SettingsStore(tmp_path, defaults=DEFAULTS)
    store.patch("servo", {"gain": 700})
    store.flush()

    meta = store.save_current_as("good-tuning", description="for AG-105")
    assert meta["name"] == "good-tuning"
    assert meta["description"] == "for AG-105"

    lst = store.list_saved()
    assert len(lst) == 1
    assert lst[0]["name"] == "good-tuning"

    # Reading it back gives the settings block.
    bundle = store.get_saved("good-tuning")
    assert bundle["settings"]["servo"]["gain"] == 700


def test_save_name_conflict(tmp_path):
    store = SettingsStore(tmp_path, defaults=DEFAULTS)
    store.save_current_as("a")
    with pytest.raises(SettingsConflict):
        store.save_current_as("a")


def test_rename_saved(tmp_path):
    store = SettingsStore(tmp_path, defaults=DEFAULTS)
    store.save_current_as("original", description="hi")
    store.rename_saved("original", "renamed")

    names = [s["name"] for s in store.list_saved()]
    assert names == ["renamed"]
    bundle = store.get_saved("renamed")
    assert bundle["name"] == "renamed"
    assert bundle["description"] == "hi"


def test_rename_collision(tmp_path):
    store = SettingsStore(tmp_path, defaults=DEFAULTS)
    store.save_current_as("a")
    store.save_current_as("b")
    with pytest.raises(SettingsConflict):
        store.rename_saved("a", "b")


def test_update_description(tmp_path):
    store = SettingsStore(tmp_path, defaults=DEFAULTS)
    store.save_current_as("a", description="old")
    store.update_description("a", "new")
    assert store.get_saved("a")["description"] == "new"


def test_delete_saved(tmp_path):
    store = SettingsStore(tmp_path, defaults=DEFAULTS)
    store.save_current_as("a")
    store.delete_saved("a")
    assert store.list_saved() == []


def test_delete_missing_raises(tmp_path):
    store = SettingsStore(tmp_path, defaults=DEFAULTS)
    with pytest.raises(SettingsNotFound):
        store.delete_saved("nope")


# ── Load saved (with safety gate) ──────────────────────────────────
def test_load_saved_rehydrates_subscribers(tmp_path):
    store = SettingsStore(tmp_path, defaults=DEFAULTS)
    store.patch("servo", {"gain": 700})
    store.save_current_as("good")

    # Now change servo and load back.
    store.patch("servo", {"gain": 300})
    seen = []
    store.subscribe("servo", lambda cfg: seen.append(cfg.get("gain")))
    seen.clear()

    store.load_saved("good")
    # The subscriber should fire with the restored value.
    assert 700 in seen


def test_load_saved_blocked_by_safety_check(tmp_path):
    store = SettingsStore(tmp_path, defaults=DEFAULTS)
    store.patch("servo", {"gain": 700})
    store.save_current_as("good")
    store.patch("servo", {"gain": 300})

    def check():
        return (False, ["robot is moving", "servo tester is running"])

    with pytest.raises(SettingsBlocked) as exc:
        store.load_saved("good", safety_check=check)

    assert len(exc.value.reasons) == 2
    # Servo should NOT have been rewritten.
    assert store.get("servo")["gain"] == 300


def test_load_saved_passes_safety_check(tmp_path):
    store = SettingsStore(tmp_path, defaults=DEFAULTS)
    store.patch("servo", {"gain": 700})
    store.save_current_as("good")
    store.patch("servo", {"gain": 300})

    store.load_saved("good", safety_check=lambda: (True, []))
    assert store.get("servo")["gain"] == 700


# ── Migration ──────────────────────────────────────────────────────
def test_migrate_legacy_files_on_first_startup(tmp_path):
    # Simulate existing table_plane.json + camera_state.json.
    legacy_dir = tmp_path / "legacy"
    legacy_dir.mkdir()
    tp = legacy_dir / "table_plane.json"
    cv = legacy_dir / "camera_state.json"
    tp.write_text(json.dumps({"corners": [{"x": 1, "y": 2, "z": 3}]}))
    cv.write_text(json.dumps({"position": {"x": 0, "y": 0, "z": 1}}))

    store_dir = tmp_path / "settings"
    store = SettingsStore(
        store_dir, defaults=DEFAULTS,
        legacy_files={"table_plane": tp, "camera_view": cv},
    )
    assert store.get("table_plane") == {"corners": [{"x": 1, "y": 2, "z": 3}]}
    assert store.get("camera_view") == {"position": {"x": 0, "y": 0, "z": 1}}


def test_migration_only_on_first_startup(tmp_path):
    # Second instance does not re-migrate — user edits to last_settings.json stick.
    legacy_dir = tmp_path / "legacy"
    legacy_dir.mkdir()
    tp = legacy_dir / "table_plane.json"
    tp.write_text(json.dumps({"corners": [{"x": 1, "y": 2, "z": 3}]}))

    store_dir = tmp_path / "settings"
    store = SettingsStore(store_dir, defaults=DEFAULTS, legacy_files={"table_plane": tp})
    store.patch("table_plane", {"corners": [{"x": 9, "y": 9, "z": 9}]})
    store.flush()
    store.shutdown()

    # Legacy file still says original — migration happens only when last_settings.json absent.
    store2 = SettingsStore(store_dir, defaults=DEFAULTS, legacy_files={"table_plane": tp})
    assert store2.get("table_plane") == {"corners": [{"x": 9, "y": 9, "z": 9}]}


# ── Import ─────────────────────────────────────────────────────────
def test_import_bundle(tmp_path):
    store = SettingsStore(tmp_path, defaults=DEFAULTS)
    bundle = {
        "name": "imported",
        "description": "from another machine",
        "settings": {"servo": {"gain": 999}, "gripper": {"force": 50}},
    }
    meta = store.import_bundle(bundle)
    assert meta["name"] == "imported"
    assert store.get_saved("imported")["settings"]["servo"]["gain"] == 999


def test_import_missing_required(tmp_path):
    store = SettingsStore(tmp_path, defaults=DEFAULTS)
    with pytest.raises(ValueError):
        store.import_bundle({"description": "no name or settings"})


def test_import_name_conflict_auto_renames(tmp_path):
    store = SettingsStore(tmp_path, defaults=DEFAULTS)
    bundle = {"name": "dup", "settings": {}}
    m1 = store.import_bundle(bundle)
    m2 = store.import_bundle(bundle)
    assert m1["slug"] != m2["slug"]
    assert m2["name"] != "dup"  # renamed with "(2)" suffix
