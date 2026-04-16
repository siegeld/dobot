"""Strategy registry — discovers, loads, and manages pick strategies.

Scans this package for PickStrategy subclasses at import time. Each strategy
has a companion JSON params file in the params/ subdirectory.
"""

from __future__ import annotations

import importlib
import json
import logging
import pkgutil
from pathlib import Path
from typing import Dict, List, Optional, Type

from dobot_ros.strategies.base import (
    ParameterDef, PickContext, PickPlan, PickStrategy, PickWaypoint,
)

log = logging.getLogger(__name__)

__all__ = [
    "ParameterDef", "PickContext", "PickPlan", "PickStrategy", "PickWaypoint",
    "StrategyRegistry",
]


class StrategyRegistry:
    """Discovers and manages pick strategies."""

    def __init__(self, params_dir: Optional[Path] = None):
        self._classes: Dict[str, Type[PickStrategy]] = {}
        self._params_dir = Path(params_dir or Path(__file__).parent / "params")
        self._params_dir.mkdir(parents=True, exist_ok=True)
        self._discover()

    def _discover(self):
        """Scan this package for PickStrategy subclasses."""
        package = importlib.import_module("dobot_ros.strategies")
        for importer, modname, ispkg in pkgutil.iter_modules(package.__path__):
            if modname.startswith("_") or modname == "base":
                continue
            try:
                mod = importlib.import_module(f"dobot_ros.strategies.{modname}")
                for attr_name in dir(mod):
                    attr = getattr(mod, attr_name)
                    if (isinstance(attr, type) and issubclass(attr, PickStrategy)
                            and attr is not PickStrategy and hasattr(attr, "slug")):
                        self._classes[attr.slug] = attr
                        log.info("Registered strategy: %s (%s)", attr.slug, attr.name)
            except Exception as e:
                log.warning("Failed to load strategy module %s: %s", modname, e)

    def list_strategies(self) -> List[dict]:
        """Return metadata for all discovered strategies."""
        out = []
        for slug, cls in sorted(self._classes.items()):
            instance = cls()
            meta = instance.metadata()
            meta["current_params"] = self.get_params(slug)
            out.append(meta)
        return out

    def get_strategy(self, slug: str) -> PickStrategy:
        """Instantiate a strategy by slug. Raises KeyError if not found."""
        cls = self._classes.get(slug)
        if cls is None:
            raise KeyError(f"unknown strategy: {slug}")
        return cls()

    def get_params(self, slug: str) -> dict:
        """Load current params from JSON, merged over defaults."""
        cls = self._classes.get(slug)
        if cls is None:
            return {}
        defaults = {p.name: p.default for p in cls.parameter_defs()}
        path = self._params_path(slug)
        if path.exists():
            try:
                saved = json.loads(path.read_text())
                defaults.update(saved)
            except Exception as e:
                log.warning("Failed to read params for %s: %s", slug, e)
        return defaults

    def set_params(self, slug: str, params: dict) -> dict:
        """Validate and save params. Returns the merged result."""
        cls = self._classes.get(slug)
        if cls is None:
            raise KeyError(f"unknown strategy: {slug}")
        defs = {p.name: p for p in cls.parameter_defs()}
        validated = {}
        for name, pdef in defs.items():
            if name in params:
                validated[name] = pdef.validate(params[name])
            else:
                validated[name] = pdef.default
        path = self._params_path(slug)
        path.write_text(json.dumps(validated, indent=2))
        log.info("Saved params for strategy %s → %s", slug, path)
        return validated

    def _params_path(self, slug: str) -> Path:
        return self._params_dir / f"{slug}.json"
