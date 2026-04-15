"""Shared dataclasses for the VLA pipeline."""

from __future__ import annotations

from dataclasses import dataclass, field, asdict
from typing import List, Optional


@dataclass
class State:
    """Proprioceptive state at a single step."""
    ee_pose: List[float]      # [X, Y, Z, RX, RY, RZ] in mm, deg
    joints: List[float]       # [J1..J6] in deg
    gripper_pos: int          # 0..1000 (0 closed, 1000 open)
    gripper_state: int        # DH AG-105 state code (0 moving, 1 reached, 2 caught, 3 dropped)

    def to_vector(self) -> List[float]:
        """7-D feature vector for RLDS: [x, y, z, rx, ry, rz, gripper_norm]."""
        return [*self.ee_pose, self.gripper_pos / 1000.0]


@dataclass
class Action:
    """Action executed at a single step.

    ee_delta is computed as (state[i+1] - state[i]) during recording.
    gripper_target is the absolute target for the next step.
    """
    ee_delta: List[float]     # [dX, dY, dZ, dRX, dRY, dRZ]
    gripper_target: int       # 0..1000

    def to_vector(self) -> List[float]:
        """7-D action vector for RLDS: [dx, dy, dz, drx, dry, drz, gripper_norm]."""
        return [*self.ee_delta, self.gripper_target / 1000.0]

    @classmethod
    def zero(cls) -> "Action":
        return cls(ee_delta=[0.0] * 6, gripper_target=0)


@dataclass
class Step:
    """One recorded timestep."""
    step: int
    t: float                  # wall clock time (unix seconds)
    image_path: str           # relative to episode dir
    state: State
    action: Action
    is_first: bool = False
    is_last: bool = False
    is_terminal: bool = False

    def to_json(self) -> dict:
        d = asdict(self)
        return d


@dataclass
class EpisodeMeta:
    """Per-episode metadata, written as meta.json at the episode root."""
    instruction: str
    started_at: float
    ended_at: Optional[float] = None
    num_steps: int = 0
    rate_hz: float = 10.0
    image_shape: List[int] = field(default_factory=list)  # [H, W, C]
    camera_url: str = ""
    # Reference to calibration files present when this episode was recorded, for traceability.
    table_plane_ref: Optional[dict] = None
    robot_config: dict = field(default_factory=dict)
    schema_version: int = 1
