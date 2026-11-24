"""Perception noise model for the simulator.

Defaults copy the measured behaviour of the static D555 at working distance:
XY from table-plane projection is good to a few mm, Z (object height) is
poor, yaw from a 640×360 mask is a few degrees off, and detections are
occasionally missed. A policy that only works with perfect perception fails
here the way it would fail on the arm.
"""

from __future__ import annotations

import copy
from dataclasses import dataclass, asdict
from typing import List

import numpy as np

from dobot_ros.policies.types import WorldObject


@dataclass
class NoiseModel:
    enabled: bool = True
    xy_sigma_mm: float = 4.0
    height_sigma_mm: float = 10.0
    yaw_sigma_deg: float = 4.0
    miss_prob: float = 0.03

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: dict) -> "NoiseModel":
        base = cls()
        for k, v in (d or {}).items():
            if hasattr(base, k):
                setattr(base, k, type(getattr(base, k))(v))
        return base

    def apply(self, objects: List[WorldObject], rng: np.random.Generator) -> List[WorldObject]:
        if not self.enabled:
            return [copy.deepcopy(o) for o in objects]
        out = []
        for o in objects:
            if o.support != "gripper" and rng.random() < self.miss_prob:
                continue
            n = copy.deepcopy(o)
            n.position[0] += float(rng.normal(0.0, self.xy_sigma_mm))
            n.position[1] += float(rng.normal(0.0, self.xy_sigma_mm))
            dh = float(rng.normal(0.0, self.height_sigma_mm))
            n.size[2] = max(5.0, n.size[2] + dh)
            # Height noise moves the perceived top, not the resting bottom.
            n.position[2] = o.bottom_z + n.size[2] / 2.0
            n.yaw_deg += float(rng.normal(0.0, self.yaw_sigma_deg))
            n.confidence = float(max(0.3, min(1.0, 0.9 - abs(dh) / 100.0)))
            out.append(n)
        return out
