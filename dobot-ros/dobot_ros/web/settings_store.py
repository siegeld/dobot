"""Centralized settings store.

Design (2026-04-15):
- All persistent settings live under `dobot-ros/dobot_ros/web/settings/`.
- `last_settings.json` — live state. Auto-persisted on every change. Loaded on
  startup. Crash-recovery, not history.
- `saved/*.json` — named user saves. The *only* "history" — user-managed.
  Each file carries name, description, created_at, updated_at, schema_version,
  and the settings block.

Live components (ServoTester, VLAExecutor, web UI defaults) subscribe to their
setting group. The store calls subscribers on `patch` and `replace_all` so they
can hot-swap config without restart.

Safety:
- Loading a saved setting is gated by a callable `safety_check` provided by the
  caller (the web layer). The store refuses to `replace_all` if the check returns
  a blocked status. See `feedback_always_safe` memory.
"""

from __future__ import annotations

import copy
import json
import logging
import os
import re
import shutil
import tempfile
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple


log = logging.getLogger(__name__)


SCHEMA_VERSION = 1
WRITE_DEBOUNCE_S = 0.5  # coalesce rapid writes (e.g., slider drags)


# ── Slugify ─────────────────────────────────────────────────────────────
_SLUG_RE = re.compile(r"[^a-z0-9_-]+")


def slugify(name: str, maxlen: int = 80) -> str:
    """Lowercase, alphanumeric + - _. Empty → 'unnamed'."""
    s = name.strip().lower().replace(" ", "_")
    s = _SLUG_RE.sub("", s)
    s = re.sub(r"_+", "_", s).strip("_-")
    return (s or "unnamed")[:maxlen]


# ── Exceptions ──────────────────────────────────────────────────────────
class SettingsError(Exception):
    """Base class for settings errors."""


class SettingsBlocked(SettingsError):
    """Raised when a safety check refuses an operation."""
    def __init__(self, reasons: List[str]):
        self.reasons = reasons
        super().__init__("; ".join(reasons))


class SettingsNotFound(SettingsError):
    pass


class SettingsConflict(SettingsError):
    pass


# ── Store ───────────────────────────────────────────────────────────────
def _utc_iso() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def _atomic_write(path: Path, content: str):
    """Write content to path atomically (tmp file + rename)."""
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = tempfile.NamedTemporaryFile(
        mode="w", delete=False, dir=str(path.parent),
        prefix=path.name + ".", suffix=".tmp",
    )
    try:
        tmp.write(content)
        tmp.flush()
        os.fsync(tmp.fileno())
        tmp.close()
        os.replace(tmp.name, str(path))
    except Exception:
        try:
            os.unlink(tmp.name)
        except Exception:
            pass
        raise


@dataclass
class SavedMeta:
    """Metadata about a named save (no settings block)."""
    name: str
    description: str
    created_at: str
    updated_at: str
    schema_version: int = SCHEMA_VERSION

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "description": self.description,
            "created_at": self.created_at,
            "updated_at": self.updated_at,
            "schema_version": self.schema_version,
        }


class SettingsStore:
    """Thread-safe persistent settings store."""

    def __init__(
        self,
        base_dir: Path,
        defaults: Optional[Dict[str, dict]] = None,
        legacy_files: Optional[Dict[str, Path]] = None,
    ):
        """base_dir: directory containing last_settings.json and saved/.
        defaults: dict of {group_name: default_settings_dict}.
        legacy_files: migrate these files into setting groups on first startup.
            e.g. {"table_plane": path_to_table_plane_json, "camera_view": ...}
        """
        self.base_dir = Path(base_dir)
        self.last_path = self.base_dir / "last_settings.json"
        self.saved_dir = self.base_dir / "saved"
        self.defaults: Dict[str, dict] = copy.deepcopy(defaults or {})
        self._legacy_files = legacy_files or {}

        self._lock = threading.RLock()
        self._data: Dict[str, Any] = {}
        self._subscribers: Dict[str, List[Callable[[dict], None]]] = {}

        # Throttled writer.
        self._dirty = False
        self._last_write = 0.0
        self._writer_thread: Optional[threading.Thread] = None
        self._writer_stop = threading.Event()

        # Ensure dirs exist.
        self.base_dir.mkdir(parents=True, exist_ok=True)
        self.saved_dir.mkdir(parents=True, exist_ok=True)

        self._load()
        self._start_writer()

    # ── Initial load / migration ──────────────────────────────────
    def _load(self):
        if self.last_path.exists():
            try:
                raw = json.loads(self.last_path.read_text())
                if isinstance(raw, dict) and "settings" in raw:
                    self._data = raw["settings"]
                elif isinstance(raw, dict):
                    self._data = raw
                else:
                    raise ValueError("last_settings.json has unexpected shape")
                log.info("Loaded last_settings.json (%d groups)", len(self._data))
            except Exception as e:
                log.warning("Failed to load %s: %s — using defaults", self.last_path, e)
                self._data = copy.deepcopy(self.defaults)
        else:
            log.info("No last_settings.json — initializing from defaults and legacy files")
            self._data = copy.deepcopy(self.defaults)
            self._migrate_legacy()
            self._mark_dirty()

        # Ensure every registered default group exists even if the on-disk
        # file predates it.
        for group, default in self.defaults.items():
            self._data.setdefault(group, copy.deepcopy(default))

    def _migrate_legacy(self):
        """Read pre-settings-store JSON files and fold them into the current data."""
        for group, path in self._legacy_files.items():
            try:
                path = Path(path)
                if path.exists():
                    data = json.loads(path.read_text())
                    self._data[group] = data
                    log.info("Migrated legacy %s from %s", group, path)
            except Exception as e:
                log.warning("Failed to migrate %s from %s: %s", group, path, e)

    # ── Writer thread ─────────────────────────────────────────────
    def _start_writer(self):
        if self._writer_thread and self._writer_thread.is_alive():
            return
        self._writer_thread = threading.Thread(
            target=self._writer_loop, daemon=True, name="SettingsWriter",
        )
        self._writer_thread.start()

    def _writer_loop(self):
        while not self._writer_stop.is_set():
            if self._dirty and (time.time() - self._last_write) >= WRITE_DEBOUNCE_S:
                self._flush()
            self._writer_stop.wait(timeout=0.25)

    def _flush(self):
        with self._lock:
            if not self._dirty:
                return
            payload = {
                "schema_version": SCHEMA_VERSION,
                "updated_at": _utc_iso(),
                "settings": self._data,
            }
            try:
                _atomic_write(self.last_path, json.dumps(payload, indent=2))
                self._dirty = False
                self._last_write = time.time()
            except Exception as e:
                log.exception("Failed to write %s: %s", self.last_path, e)

    def _mark_dirty(self):
        with self._lock:
            self._dirty = True

    def flush(self):
        """Force an immediate write. Used on shutdown / before loads."""
        self._flush()

    def shutdown(self):
        """Flush pending writes and stop the writer thread."""
        self._writer_stop.set()
        self._flush()

    # ── Live access ───────────────────────────────────────────────
    def get(self, group: str) -> dict:
        with self._lock:
            return copy.deepcopy(self._data.get(group, self.defaults.get(group, {})))

    def get_all(self) -> dict:
        with self._lock:
            return copy.deepcopy(self._data)

    def patch(self, group: str, partial: dict) -> dict:
        """Shallow-merge `partial` into group. Notify subscribers. Returns new group dict."""
        if not isinstance(partial, dict):
            raise ValueError("patch expects a dict")
        with self._lock:
            cur = dict(self._data.get(group, {}))
            cur.update(partial)
            self._data[group] = cur
            self._mark_dirty()
            new = copy.deepcopy(cur)
        self._notify(group, new)
        return new

    def set_group(self, group: str, value: dict) -> dict:
        """Replace a whole group (not merge). Notifies subscribers."""
        if not isinstance(value, dict):
            raise ValueError("set_group expects a dict")
        with self._lock:
            self._data[group] = copy.deepcopy(value)
            self._mark_dirty()
            new = copy.deepcopy(value)
        self._notify(group, new)
        return new

    def replace_all(self, settings: dict, safety_check: Optional[Callable[[], Tuple[bool, List[str]]]] = None):
        """Overwrite all settings and re-notify every subscriber.

        If `safety_check` is provided, it must return (ok, reasons). If not ok,
        SettingsBlocked is raised with the reasons. Used by the load-saved flow.
        """
        if safety_check is not None:
            ok, reasons = safety_check()
            if not ok:
                raise SettingsBlocked(reasons)
        if not isinstance(settings, dict):
            raise ValueError("replace_all expects a dict")
        with self._lock:
            self._data = copy.deepcopy(settings)
            # Ensure every default group is present.
            for group, default in self.defaults.items():
                self._data.setdefault(group, copy.deepcopy(default))
            self._mark_dirty()
            self._flush()  # write immediately on loads
            groups_snapshot = {g: copy.deepcopy(v) for g, v in self._data.items()}
        for group, value in groups_snapshot.items():
            self._notify(group, value)

    # ── Subscribers ──────────────────────────────────────────────
    def subscribe(self, group: str, callback: Callable[[dict], None]):
        """Register a callback invoked with the latest group settings whenever
        the group is patched, replaced, or a saved bundle is loaded.
        """
        with self._lock:
            self._subscribers.setdefault(group, []).append(callback)
        # Fire once now so the subscriber gets the current state.
        try:
            callback(self.get(group))
        except Exception:
            log.exception("initial subscriber call failed for group %s", group)

    def _notify(self, group: str, value: dict):
        with self._lock:
            subs = list(self._subscribers.get(group, []))
        for cb in subs:
            try:
                cb(copy.deepcopy(value))
            except Exception:
                log.exception("subscriber for group %s failed", group)

    # ── Saved bundles ────────────────────────────────────────────
    def _saved_path(self, name: str) -> Path:
        return self.saved_dir / f"{slugify(name)}.json"

    def list_saved(self) -> List[dict]:
        out = []
        for p in sorted(self.saved_dir.glob("*.json")):
            try:
                d = json.loads(p.read_text())
                meta = {k: d.get(k) for k in ("name", "description", "created_at", "updated_at", "schema_version")}
                meta["slug"] = p.stem
                out.append(meta)
            except Exception as e:
                log.warning("skipping malformed saved file %s: %s", p, e)
        # Most recently updated first.
        out.sort(key=lambda m: m.get("updated_at") or "", reverse=True)
        return out

    def get_saved(self, name: str) -> dict:
        p = self._saved_path(name)
        if not p.exists():
            raise SettingsNotFound(f"saved settings '{name}' not found")
        return json.loads(p.read_text())

    def save_current_as(self, name: str, description: str = "") -> dict:
        """Snapshot current `last_settings` into a named save."""
        slug = slugify(name)
        if not slug:
            raise ValueError("name cannot be empty")
        p = self._saved_path(name)
        if p.exists():
            raise SettingsConflict(f"a saved setting named '{slug}' already exists")
        now = _utc_iso()
        with self._lock:
            bundle = {
                "name": name,
                "description": description,
                "created_at": now,
                "updated_at": now,
                "schema_version": SCHEMA_VERSION,
                "settings": copy.deepcopy(self._data),
            }
        _atomic_write(p, json.dumps(bundle, indent=2))
        log.info("Saved settings as '%s' → %s", slug, p)
        bundle_meta = {k: bundle[k] for k in ("name", "description", "created_at", "updated_at", "schema_version")}
        bundle_meta["slug"] = slug
        return bundle_meta

    def load_saved(self, name: str, safety_check: Optional[Callable[[], Tuple[bool, List[str]]]] = None) -> dict:
        """Load a named save into current settings. Honors safety_check."""
        bundle = self.get_saved(name)
        if "settings" not in bundle:
            raise SettingsError(f"saved file '{name}' has no 'settings' block")
        self.replace_all(bundle["settings"], safety_check=safety_check)
        return self.get_all()

    def rename_saved(self, old: str, new_name: str) -> dict:
        new_slug = slugify(new_name)
        if not new_slug:
            raise ValueError("new name cannot be empty")
        old_p = self._saved_path(old)
        new_p = self.saved_dir / f"{new_slug}.json"
        if not old_p.exists():
            raise SettingsNotFound(f"saved settings '{old}' not found")
        if new_p.exists() and new_p.resolve() != old_p.resolve():
            raise SettingsConflict(f"a saved setting named '{new_slug}' already exists")
        bundle = json.loads(old_p.read_text())
        bundle["name"] = new_name
        bundle["updated_at"] = _utc_iso()
        _atomic_write(new_p, json.dumps(bundle, indent=2))
        if new_p.resolve() != old_p.resolve():
            old_p.unlink()
        return {k: bundle[k] for k in ("name", "description", "created_at", "updated_at", "schema_version")} | {"slug": new_slug}

    def update_description(self, name: str, description: str) -> dict:
        p = self._saved_path(name)
        if not p.exists():
            raise SettingsNotFound(f"saved settings '{name}' not found")
        bundle = json.loads(p.read_text())
        bundle["description"] = description
        bundle["updated_at"] = _utc_iso()
        _atomic_write(p, json.dumps(bundle, indent=2))
        return {k: bundle[k] for k in ("name", "description", "created_at", "updated_at", "schema_version")} | {"slug": slugify(name)}

    def delete_saved(self, name: str):
        p = self._saved_path(name)
        if not p.exists():
            raise SettingsNotFound(f"saved settings '{name}' not found")
        p.unlink()
        log.info("Deleted saved settings '%s'", slugify(name))

    def import_bundle(self, data: dict) -> dict:
        """Import a JSON bundle produced by save_current_as. Returns the saved meta."""
        required = {"name", "settings"}
        missing = required - set(data.keys())
        if missing:
            raise ValueError(f"import missing required keys: {sorted(missing)}")
        name = data["name"]
        slug = slugify(name)
        p = self._saved_path(name)
        bundle = {
            "name": name,
            "description": data.get("description", ""),
            "created_at": data.get("created_at") or _utc_iso(),
            "updated_at": _utc_iso(),
            "schema_version": SCHEMA_VERSION,
            "settings": data["settings"],
        }
        if p.exists():
            # Disambiguate by appending a suffix.
            i = 2
            while (self.saved_dir / f"{slug}_{i}.json").exists():
                i += 1
            slug = f"{slug}_{i}"
            p = self.saved_dir / f"{slug}.json"
            bundle["name"] = f"{name} ({i})"
        _atomic_write(p, json.dumps(bundle, indent=2))
        meta = {k: bundle[k] for k in ("name", "description", "created_at", "updated_at", "schema_version")}
        meta["slug"] = slug
        return meta
