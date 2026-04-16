"""Base class and data types for pick strategies.

A pick strategy is a pluggable Python module that decides HOW to pick an
object once the Vision system has resolved WHERE it is. Strategies are
pure planning code — they receive a PickContext (robot-frame coordinates,
table Z, object data) and return a PickPlan (sequence of waypoints +
gripper actions). No I/O, no ROS calls, no camera access.

Each strategy defines its own tunable parameters via parameter_defs().
The current values live in a JSON file alongside the strategy code.
The GUI renders a form for them dynamically.
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, List, Optional


@dataclass
class ParameterDef:
    """Describes one tunable parameter that the GUI renders as a form control."""
    name: str               # machine key, e.g. "approach_height_mm"
    type: str               # "float" | "int" | "bool" | "choice"
    default: Any            # default value
    description: str        # human label
    min: Optional[float] = None
    max: Optional[float] = None
    step: Optional[float] = None
    choices: Optional[List[str]] = None  # for type="choice"
    unit: Optional[str] = None           # "mm", "deg", "%"

    def to_dict(self) -> dict:
        d = {"name": self.name, "type": self.type, "default": self.default,
             "description": self.description}
        if self.min is not None: d["min"] = self.min
        if self.max is not None: d["max"] = self.max
        if self.step is not None: d["step"] = self.step
        if self.choices is not None: d["choices"] = self.choices
        if self.unit is not None: d["unit"] = self.unit
        return d

    def validate(self, value: Any) -> Any:
        """Coerce and validate a value against this parameter definition."""
        if self.type == "float":
            value = float(value)
            if self.min is not None: value = max(self.min, value)
            if self.max is not None: value = min(self.max, value)
        elif self.type == "int":
            value = int(value)
            if self.min is not None: value = max(int(self.min), value)
            if self.max is not None: value = min(int(self.max), value)
        elif self.type == "bool":
            value = bool(value)
        elif self.type == "choice":
            if self.choices and str(value) not in self.choices:
                value = self.default
            value = str(value)
        return value


@dataclass
class PickContext:
    """Everything a strategy needs to compute a pick plan.

    Pre-resolved by the server layer — strategies never touch VisionTransform,
    the camera, or the ROS client directly.
    """
    robot_x: float              # mm, table-plane projected
    robot_y: float              # mm
    table_z: float              # mm, wrist Z when gripper tip touches table
    min_clearance_mm: float     # safety floor above table_z
    rotation_deg: float         # perspective-corrected table-plane rotation

    current_pose: List[float]   # [X,Y,Z,RX,RY,RZ] from get_cartesian_pose()

    object_height_mm: float = 0.0
    object_present: bool = False
    object_data: Optional[dict] = None  # raw detection dict from camera

    params: dict = field(default_factory=dict)  # strategy params from JSON


@dataclass
class PickWaypoint:
    """One step in the pick sequence."""
    pose: List[float]                       # [X,Y,Z,RX,RY,RZ]
    label: str                              # "approach", "grasp", "retract", etc.
    pause_s: float = 0.0                    # dwell time after arriving
    gripper_action: Optional[str] = None    # "open" | "close" | None
    gripper_pos: Optional[int] = None       # 0-1000, used with gripper_action


@dataclass
class PickPlan:
    """Output of a strategy's plan() method."""
    waypoints: List[PickWaypoint]
    move_speed: Optional[int] = None  # override global speed if set


class PickStrategy(ABC):
    """Abstract base for pick strategies.

    Subclasses must set name, slug, description as class attributes and
    implement parameter_defs() and plan().
    """
    name: str = "Unnamed"
    slug: str = "unnamed"
    description: str = ""

    @classmethod
    @abstractmethod
    def parameter_defs(cls) -> List[ParameterDef]:
        """Declare the tunable parameters this strategy accepts."""
        ...

    @abstractmethod
    def plan(self, ctx: PickContext) -> PickPlan:
        """Given a pick context, compute the full waypoint sequence."""
        ...

    def metadata(self) -> dict:
        return {
            "name": self.name,
            "slug": self.slug,
            "description": self.description,
            "parameters": [p.to_dict() for p in self.parameter_defs()],
        }
