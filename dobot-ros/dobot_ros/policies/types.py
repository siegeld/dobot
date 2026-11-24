"""Shared types: the Robot / World protocols, WorldObject, Trace, results."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field, asdict
from typing import Any, Callable, Dict, List, Optional, Protocol, Sequence, runtime_checkable

Pose6 = List[float]  # [X, Y, Z, RX, RY, RZ] mm / deg, in the active tool frame


# ── Exceptions ──────────────────────────────────────────────────────

class PolicyError(Exception):
    """Base for everything a policy can raise or be interrupted by."""


class PolicyAbort(PolicyError):
    """Stop requested (UI stop button, confirm cancelled, server shutdown)."""


class GraspMissed(PolicyError):
    """Gripper closed fully — nothing between the fingers."""


class NotFound(PolicyError):
    """Perception query matched nothing. ``seen`` carries what was visible."""
    def __init__(self, msg: str, seen: Optional[List["WorldObject"]] = None):
        super().__init__(msg)
        self.seen = seen or []


class FloorViolation(PolicyError):
    """A commanded pose would put the fingertip below the table clearance."""


class WorkspaceViolation(PolicyError):
    """A commanded pose is outside the calibrated workspace XY bounds."""


class Unreachable(PolicyError):
    """IK found no joint solution for the commanded pose."""


class Collision(PolicyError):
    """(Sim only) the gripper ran into an object it was not grasping."""


# ── World objects ───────────────────────────────────────────────────

@dataclass
class WorldObject:
    """One perceived or simulated object, robot base frame, millimetres.

    ``position`` is the centre of the object. ``size`` is (lx, ly, lz) of its
    bounding box *before* yaw; ``yaw_deg`` rotates it about Z.
    """
    id: str
    label: str
    color: str
    position: List[float]
    size: List[float]
    yaw_deg: float = 0.0
    confidence: float = 1.0
    support: str = "table"          # "table" | "<object id>" | "gripper"
    attrs: Dict[str, Any] = field(default_factory=dict)

    @property
    def xy(self) -> List[float]:
        return [self.position[0], self.position[1]]

    @property
    def top_z(self) -> float:
        return self.position[2] + self.size[2] / 2.0

    @property
    def bottom_z(self) -> float:
        return self.position[2] - self.size[2] / 2.0

    def width_across(self, axis_yaw_deg: float) -> float:
        """Extent of the footprint measured along a horizontal axis."""
        d = math.radians(axis_yaw_deg - self.yaw_deg)
        return abs(self.size[0] * math.cos(d)) + abs(self.size[1] * math.sin(d))

    def short_axis_yaw(self) -> float:
        """Yaw of the footprint's shorter side (the axis to close jaws along)."""
        return self.yaw_deg + (90.0 if self.size[0] >= self.size[1] else 0.0)

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: dict) -> "WorldObject":
        return cls(
            id=str(d["id"]), label=d.get("label", "object"), color=d.get("color", "unknown"),
            position=[float(v) for v in d["position"]], size=[float(v) for v in d["size"]],
            yaw_deg=float(d.get("yaw_deg", 0.0)), confidence=float(d.get("confidence", 1.0)),
            support=d.get("support", "table"), attrs=dict(d.get("attrs", {})),
        )


# ── Protocols ───────────────────────────────────────────────────────

@runtime_checkable
class Robot(Protocol):
    """Motion + gripper. Poses are in the active tool frame, like the controller."""
    tool_length_mm: float

    def pose(self) -> Pose6: ...
    def joints(self) -> List[float]: ...
    def tool_index(self) -> int: ...
    def set_tool(self, index: int) -> None: ...
    def move_l(self, pose: Sequence[float], speed: Optional[int] = None) -> None: ...
    def move_j(self, joints: Sequence[float], speed: Optional[int] = None) -> None: ...
    def gripper(self, pos_0_1000: int, force: int = 20) -> int: ...
    def gripper_pos(self) -> int: ...
    def stop(self) -> None: ...


@runtime_checkable
class World(Protocol):
    """Perception + table geometry."""
    table_z: float            # wrist Z when the fingertip touches the table (project convention)
    min_clearance_mm: float   # fingertip may not go closer than this to the table

    def objects(self) -> List[WorldObject]: ...
    def find(self, **query: Any) -> Optional[WorldObject]: ...


# ── Trace ───────────────────────────────────────────────────────────

@dataclass
class TraceEvent:
    t: float                      # seconds since run start (sim: virtual clock)
    kind: str                     # "call" | "return" | "sample" | "log" | "error" | "result"
    name: str = ""
    args: Dict[str, Any] = field(default_factory=dict)
    result: Any = None
    joints: Optional[List[float]] = None
    pose: Optional[List[float]] = None
    gripper: Optional[int] = None
    tool_index: Optional[int] = None
    objects: Optional[List[dict]] = None   # [{id, position, yaw_deg, support}]
    message: str = ""
    line: Optional[int] = None             # policy source line that issued the call

    def to_dict(self) -> dict:
        d = {"t": round(self.t, 3), "kind": self.kind}
        if self.name: d["name"] = self.name
        if self.args: d["args"] = self.args
        if self.result is not None: d["result"] = self.result
        if self.joints is not None: d["joints"] = [round(v, 2) for v in self.joints]
        if self.pose is not None: d["pose"] = [round(v, 1) for v in self.pose]
        if self.gripper is not None: d["gripper"] = self.gripper
        if self.tool_index is not None: d["tool_index"] = self.tool_index
        if self.objects is not None: d["objects"] = self.objects
        if self.message: d["message"] = self.message
        if self.line is not None: d["line"] = self.line
        return d


class Trace:
    """Append-only record of a run. Sinks (websocket, stdout) get each event."""

    def __init__(self, clock: Optional[Callable[[], float]] = None):
        self.events: List[TraceEvent] = []
        self._sinks: List[Callable[[TraceEvent], None]] = []
        self._clock = clock or time.monotonic
        self._t0 = self._clock()
        self._depth = 0

    def now(self) -> float:
        return self._clock() - self._t0

    def add_sink(self, fn: Callable[[TraceEvent], None]) -> None:
        self._sinks.append(fn)

    def emit(self, ev: TraceEvent) -> TraceEvent:
        self.events.append(ev)
        for s in self._sinks:
            try:
                s(ev)
            except Exception:
                pass
        return ev

    def log(self, message: str) -> None:
        self.emit(TraceEvent(t=self.now(), kind="log", message=message))

    def call(self, name: str, **args: Any) -> None:
        self.emit(TraceEvent(t=self.now(), kind="call", name=name, args=_jsonable(args),
                             line=_policy_line()))

    def ret(self, name: str, result: Any = None, **state: Any) -> None:
        self.emit(TraceEvent(t=self.now(), kind="return", name=name, result=_jsonable(result), **state))

    def sample(self, **state: Any) -> None:
        self.emit(TraceEvent(t=self.now(), kind="sample", **state))

    def error(self, name: str, exc: BaseException, **state: Any) -> None:
        self.emit(TraceEvent(t=self.now(), kind="error", name=name,
                             message=f"{type(exc).__name__}: {exc}", **state))

    def to_list(self) -> List[dict]:
        return [e.to_dict() for e in self.events]


_INFRA = ("/policies/primitives.py", "/policies/types.py", "/policies/base.py",
          "/policies/runner.py", "/sim/", "/real/")


def _policy_line() -> Optional[int]:
    """Line number in the *policy* source file that led to this call —
    the first frame up the stack that lives in dobot_ros/policies/ but is
    not one of the infrastructure modules."""
    import sys
    f = sys._getframe(2)
    for _ in range(25):
        if f is None:
            break
        fn = f.f_code.co_filename.replace("\\", "/")
        if "/policies/" in fn and not any(x in fn for x in _INFRA):
            return f.f_lineno
        f = f.f_back
    return None


def _jsonable(v: Any) -> Any:
    if isinstance(v, dict):
        return {k: _jsonable(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [_jsonable(x) for x in v]
    if isinstance(v, WorldObject):
        return v.to_dict()
    if isinstance(v, float):
        return round(v, 3)
    if isinstance(v, (int, str, bool)) or v is None:
        return v
    return str(v)


# ── Result ──────────────────────────────────────────────────────────

@dataclass
class PolicyResult:
    ok: bool
    reason: str = ""
    attempts: int = 1
    observations: Dict[str, Any] = field(default_factory=dict)
    elapsed_s: float = 0.0

    def to_dict(self) -> dict:
        return {"ok": self.ok, "reason": self.reason, "attempts": self.attempts,
                "observations": _jsonable(self.observations),
                "elapsed_s": round(self.elapsed_s, 3)}
