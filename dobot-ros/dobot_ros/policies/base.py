"""Policy ABC. Mirrors strategies/base.py: class attrs for metadata,
``parameter_defs()`` for the auto-rendered form, one ``run()`` method."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import List

from dobot_ros.strategies.base import ParameterDef
from dobot_ros.policies.types import PolicyResult, Robot, Trace, World


class Policy(ABC):
    name: str = "Unnamed"
    slug: str = "unnamed"
    description: str = ""

    @classmethod
    @abstractmethod
    def parameter_defs(cls) -> List[ParameterDef]: ...

    @abstractmethod
    def run(self, robot: Robot, world: World, params: dict, trace: Trace) -> PolicyResult:
        """Do the task. Raise a PolicyError subclass to abort; return a
        PolicyResult with ok=False for a graceful failure."""
        ...

    def metadata(self) -> dict:
        return {
            "name": self.name, "slug": self.slug, "description": self.description,
            "parameters": [p.to_dict() for p in self.parameter_defs()],
        }
