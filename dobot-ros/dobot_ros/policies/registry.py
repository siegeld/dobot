"""Policy registry — discovers Policy subclasses in this package and keeps
their params JSON in params/. Same shape as strategies.StrategyRegistry."""

from __future__ import annotations

import importlib
import json
import logging
import pkgutil
from pathlib import Path
from typing import Dict, List, Optional, Type

from dobot_ros.policies.base import Policy

log = logging.getLogger(__name__)

_SKIP = {"base", "types", "primitives", "registry", "guards", "runner", "scene"}


class PolicyRegistry:
    def __init__(self, params_dir: Optional[Path] = None):
        self._classes: Dict[str, Type[Policy]] = {}
        self._params_dir = Path(params_dir or Path(__file__).parent / "params")
        self._params_dir.mkdir(parents=True, exist_ok=True)
        self._discover()

    def _discover(self):
        package = importlib.import_module("dobot_ros.policies")
        for _, modname, _ in pkgutil.iter_modules(package.__path__):
            if modname.startswith("_") or modname in _SKIP:
                continue
            try:
                mod = importlib.import_module(f"dobot_ros.policies.{modname}")
                for attr_name in dir(mod):
                    attr = getattr(mod, attr_name)
                    if (isinstance(attr, type) and issubclass(attr, Policy)
                            and attr is not Policy and getattr(attr, "slug", None)):
                        self._classes[attr.slug] = attr
                        log.info("Registered policy: %s (%s)", attr.slug, attr.name)
            except Exception as e:
                log.warning("Failed to load policy module %s: %s", modname, e)

    def reload(self) -> None:
        """Re-import the primitive library and the policy modules so an
        edited file is picked up without a server restart (the code tree is
        volume-mounted). Engine modules (types, guards, runner, sim/, real/)
        are deliberately NOT reloaded — isinstance checks and live objects
        depend on their identity — so an engine change needs a container
        restart."""
        import sys
        self._classes.clear()
        if "dobot_ros.policies.primitives" in sys.modules:
            importlib.reload(sys.modules["dobot_ros.policies.primitives"])
        package = importlib.import_module("dobot_ros.policies")
        for _, modname, _ in pkgutil.iter_modules(package.__path__):
            if modname.startswith("_") or modname in _SKIP:
                continue
            name = f"dobot_ros.policies.{modname}"
            try:
                import sys
                if name in sys.modules:
                    importlib.reload(sys.modules[name])
            except Exception as e:
                log.warning("reload %s failed: %s", name, e)
        self._discover()

    def slugs(self) -> List[str]:
        return sorted(self._classes)

    def list_policies(self) -> List[dict]:
        out = []
        for slug, cls in sorted(self._classes.items()):
            meta = cls().metadata()
            meta["current_params"] = self.get_params(slug)
            meta["source_file"] = self.source_path(slug).name
            out.append(meta)
        return out

    def get_policy(self, slug: str) -> Policy:
        cls = self._classes.get(slug)
        if cls is None:
            raise KeyError(f"unknown policy: {slug}")
        return cls()

    def source_path(self, slug: str) -> Path:
        import inspect
        return Path(inspect.getsourcefile(self._classes[slug]))

    def source(self, slug: str) -> str:
        return self.source_path(slug).read_text()

    def get_params(self, slug: str) -> dict:
        cls = self._classes.get(slug)
        if cls is None:
            return {}
        defaults = {p.name: p.default for p in cls.parameter_defs()}
        path = self._params_path(slug)
        if path.exists():
            try:
                defaults.update(json.loads(path.read_text()))
            except Exception as e:
                log.warning("Failed to read params for %s: %s", slug, e)
        return defaults

    def set_params(self, slug: str, params: dict) -> dict:
        cls = self._classes.get(slug)
        if cls is None:
            raise KeyError(f"unknown policy: {slug}")
        validated = {}
        for pdef in cls.parameter_defs():
            validated[pdef.name] = pdef.validate(params[pdef.name]) if pdef.name in params else pdef.default
        self._params_path(slug).write_text(json.dumps(validated, indent=2))
        return validated

    def validate_params(self, slug: str, params: dict) -> dict:
        """Merge an override dict over the saved params without persisting."""
        merged = self.get_params(slug)
        cls = self._classes[slug]
        defs = {p.name: p for p in cls.parameter_defs()}
        for k, v in (params or {}).items():
            if k in defs:
                merged[k] = defs[k].validate(v)
        return merged

    def _params_path(self, slug: str) -> Path:
        return self._params_dir / f"{slug}.json"
