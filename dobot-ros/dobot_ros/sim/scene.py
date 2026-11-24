"""Scene files: the objects on the simulated table.

A scene is JSON: ``{"name", "description", "objects": [WorldObject dicts]}``.
``position`` may be 2-D (XY, the object rests on the table) or 3-D.
Presets ship in ``sim/scenes/``; the same directory receives user saves.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Union

import numpy as np

from dobot_ros.policies.guards import WorkspaceBounds
from dobot_ros.policies.types import WorldObject

SCENES_DIR = Path(__file__).parent / "scenes"

COLORS = ["red", "green", "blue", "yellow", "orange", "purple"]


def list_scenes(scenes_dir: Path = SCENES_DIR) -> List[dict]:
    out = []
    for p in sorted(scenes_dir.glob("*.json")):
        try:
            d = json.loads(p.read_text())
            out.append({"name": p.stem, "description": d.get("description", ""),
                        "object_count": len(d.get("objects", []))})
        except Exception:
            continue
    return out


def load_scene(name_or_dict: Union[str, dict], scenes_dir: Path = SCENES_DIR) -> dict:
    if isinstance(name_or_dict, dict):
        return name_or_dict
    p = scenes_dir / f"{name_or_dict}.json"
    if not p.exists():
        raise FileNotFoundError(f"scene not found: {name_or_dict}")
    return json.loads(p.read_text())


def save_scene(name: str, scene: dict, scenes_dir: Path = SCENES_DIR) -> Path:
    safe = "".join(c if c.isalnum() or c in "-_" else "_" for c in name.strip())[:64]
    if not safe:
        raise ValueError("scene name required")
    p = scenes_dir / f"{safe}.json"
    scene = dict(scene)
    scene["name"] = safe
    p.write_text(json.dumps(scene, indent=2))
    return p


def objects_from_scene(scene: dict, surface_z: float) -> List[WorldObject]:
    objs = []
    for i, d in enumerate(scene.get("objects", [])):
        d = dict(d)
        pos = [float(v) for v in d.get("position", [0, 0])]
        size = [float(v) for v in d.get("size", [40, 40, 40])]
        if len(pos) == 2:
            pos = [pos[0], pos[1], surface_z + size[2] / 2.0]
        d["position"], d["size"] = pos, size
        d.setdefault("id", f"{d.get('color', 'obj')}_{d.get('label', 'object')}_{i}")
        objs.append(WorldObject.from_dict(d))
    return objs


def _footprints_overlap(a: WorldObject, b: WorldObject, gap_mm: float) -> bool:
    # Conservative: circumscribed circles.
    ra = math.hypot(a.size[0], a.size[1]) / 2.0
    rb = math.hypot(b.size[0], b.size[1]) / 2.0
    return math.hypot(a.position[0] - b.position[0], a.position[1] - b.position[1]) < ra + rb + gap_mm


def randomize(
    spec: Sequence[dict],
    bounds: WorkspaceBounds,
    surface_z: float,
    rng: np.random.Generator,
    reachable=None,
    margin_mm: float = 60.0,
    gap_mm: float = 30.0,
    max_tries: int = 200,
) -> List[WorldObject]:
    """Place each object in ``spec`` (color/label/size, position ignored) at
    a random non-overlapping XY inside the bounds, optionally filtered by a
    ``reachable(x, y)`` predicate. Yaw uniform in [-90, 90)."""
    placed: List[WorldObject] = []
    for i, d in enumerate(spec):
        d = dict(d)
        size = [float(v) for v in d.get("size", [40, 40, 40])]
        for _ in range(max_tries):
            x = float(rng.uniform(bounds.x_min + margin_mm, bounds.x_max - margin_mm))
            y = float(rng.uniform(bounds.y_min + margin_mm, bounds.y_max - margin_mm))
            if reachable is not None and not reachable(x, y):
                continue
            cand = WorldObject(
                id=d.get("id", f"{d.get('color', 'obj')}_{d.get('label', 'object')}_{i}"),
                label=d.get("label", "block"), color=d.get("color", COLORS[i % len(COLORS)]),
                position=[x, y, surface_z + size[2] / 2.0], size=size,
                yaw_deg=float(rng.uniform(-90.0, 90.0)),
            )
            if all(not _footprints_overlap(cand, p, gap_mm) for p in placed):
                placed.append(cand)
                break
        else:
            raise RuntimeError(f"could not place object {i} after {max_tries} tries")
    return placed
