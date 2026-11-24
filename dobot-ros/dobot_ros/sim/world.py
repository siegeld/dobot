"""SimWorld — the simulated table and the objects on it."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Union

import numpy as np

from dobot_ros.policies.guards import WorkspaceBounds
from dobot_ros.policies.types import WorldObject
from dobot_ros.sim import kinematics as K
from dobot_ros.sim.noise import NoiseModel
from dobot_ros.sim.scene import load_scene, objects_from_scene, randomize

# Fallbacks matching web/table_plane.json at the time of writing, used when
# the calibration file is unavailable (tests, laptops).
DEFAULT_TABLE_Z = 28.34
DEFAULT_TABLE_POINTS = [[-676.7, 80.0], [-281.9, 85.7], [-661.1, -676.3], [-271.4, -678.9]]
DEFAULT_MIN_CLEARANCE_MM = 25.0


def load_table_plane(path: Optional[Path] = None) -> Dict[str, Any]:
    """Read table_plane.json → {table_z, points}. Falls back to defaults."""
    p = path or Path(__file__).resolve().parent.parent / "web" / "table_plane.json"
    try:
        d = json.loads(Path(p).read_text())
        pts = [[float(q[0]), float(q[1])] for q in d.get("points", [])]
        z = float(d.get("plane", {}).get("mean_z", DEFAULT_TABLE_Z))
        if len(pts) >= 3:
            return {"table_z": z, "points": pts}
    except Exception:
        pass
    return {"table_z": DEFAULT_TABLE_Z, "points": DEFAULT_TABLE_POINTS}


class SimWorld:
    """Ground-truth objects plus a noisy ``objects()`` view.

    ``table_z`` keeps the project convention (wrist Z at fingertip touch);
    the physical surface in base frame is ``table_z - tool_length_mm``.
    """

    def __init__(
        self,
        table_z: float = DEFAULT_TABLE_Z,
        tool_length_mm: float = K.DEFAULT_TOOL_LENGTH_MM,
        min_clearance_mm: float = DEFAULT_MIN_CLEARANCE_MM,
        workspace: Optional[WorkspaceBounds] = None,
        noise: Optional[NoiseModel] = None,
        seed: Optional[int] = None,
    ):
        self.table_z = float(table_z)
        self.tool_length_mm = float(tool_length_mm)
        self.min_clearance_mm = float(min_clearance_mm)
        self.workspace = workspace or WorkspaceBounds.from_points(DEFAULT_TABLE_POINTS)
        self.noise = noise or NoiseModel()
        self.rng = np.random.default_rng(seed)
        self.seed = seed
        self._objects: Dict[str, WorldObject] = {}
        self.scene_name: str = ""

    # ── construction ────────────────────────────────────────────────
    @classmethod
    def from_scene(cls, scene: Union[str, dict], seed: Optional[int] = None,
                   noise: Optional[NoiseModel] = None, random_positions: bool = False,
                   table: Optional[dict] = None, **kw) -> "SimWorld":
        table = table or load_table_plane()
        ws = WorkspaceBounds.from_points(table["points"])
        w = cls(table_z=table["table_z"], workspace=ws, noise=noise, seed=seed, **kw)
        sd = load_scene(scene)
        w.scene_name = sd.get("name", "custom")
        if random_positions:
            w.randomize(sd.get("objects", []))
        else:
            for o in objects_from_scene(sd, w.surface_z):
                w.add(o)
        return w

    @property
    def surface_z(self) -> float:
        return self.table_z - self.tool_length_mm

    def add(self, obj: WorldObject) -> None:
        self._objects[obj.id] = obj

    def clear(self) -> None:
        self._objects.clear()

    def randomize(self, spec: Sequence[dict], seed: Optional[int] = None) -> List[WorldObject]:
        if seed is not None:
            self.rng = np.random.default_rng(seed)
            self.seed = seed
        self._objects.clear()
        hover_z = self.surface_z + 80.0
        grasp_z = self.surface_z + self.min_clearance_mm

        def reachable(x: float, y: float) -> bool:
            return (K.reachable([x, y, hover_z, 180, 0, 0], tool_length_mm=self.tool_length_mm)
                    and K.reachable([x, y, grasp_z, 180, 0, 0], tool_length_mm=self.tool_length_mm))

        for o in randomize(spec, self.workspace, self.surface_z, self.rng, reachable=reachable):
            self.add(o)
        return list(self._objects.values())

    # ── World protocol ──────────────────────────────────────────────
    def objects(self) -> List[WorldObject]:
        return self.noise.apply(list(self._objects.values()), self.rng)

    def find(self, **query: Any) -> Optional[WorldObject]:
        best = None
        for o in self.objects():
            if all(_match(getattr(o, k, o.attrs.get(k)), v) for k, v in query.items()):
                if best is None or o.confidence > best.confidence:
                    best = o
        return best

    # ── ground truth for the simulator / tests ──────────────────────
    def truth(self) -> List[WorldObject]:
        return list(self._objects.values())

    def get(self, obj_id: str) -> WorldObject:
        return self._objects[obj_id]

    def support_z_at(self, x: float, y: float, exclude: Optional[str] = None) -> float:
        """Top of the highest object under (x, y), else the table surface."""
        z = self.surface_z
        for o in self._objects.values():
            if o.id == exclude or o.support == "gripper":
                continue
            if _footprint_contains(o, x, y, 0.0) and o.top_z > z:
                z = o.top_z
        return z

    def snapshot(self) -> List[dict]:
        return [{"id": o.id, "color": o.color, "label": o.label,
                 "position": [round(v, 1) for v in o.position],
                 "size": o.size, "yaw_deg": round(o.yaw_deg, 1), "support": o.support}
                for o in self._objects.values()]

    def to_scene(self) -> dict:
        return {"name": self.scene_name or "custom", "objects": [o.to_dict() for o in self._objects.values()]}


def _match(a: Any, b: Any) -> bool:
    if isinstance(a, str) and isinstance(b, str):
        return a.lower() == b.lower()
    return a == b


def _footprint_contains(o: WorldObject, x: float, y: float, expand_mm: float) -> bool:
    """Is (x, y) inside the object's yawed rectangle (grown by expand_mm)?"""
    d = math.radians(-o.yaw_deg)
    dx, dy = x - o.position[0], y - o.position[1]
    lx = dx * math.cos(d) - dy * math.sin(d)
    ly = dx * math.sin(d) + dy * math.cos(d)
    return abs(lx) <= o.size[0] / 2.0 + expand_mm and abs(ly) <= o.size[1] / 2.0 + expand_mm
