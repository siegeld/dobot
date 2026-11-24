"""Hand-coded robot policies.

A *policy* is a task-level Python program ("pick up the red block") that
composes the primitives in ``primitives.py`` against two narrow protocols,
``Robot`` and ``World`` (``types.py``). The same policy source runs on the
kinematic simulator (``dobot_ros.sim``) and the real arm (``dobot_ros.real``).
See POLICIES.md.
"""

from dobot_ros.policies.types import (  # noqa: F401
    Robot, World, WorldObject, Trace, TraceEvent, PolicyResult,
    PolicyError, PolicyAbort, GraspMissed, NotFound, FloorViolation,
    WorkspaceViolation, Unreachable, Collision,
)
from dobot_ros.policies.base import Policy  # noqa: F401
from dobot_ros.policies.registry import PolicyRegistry  # noqa: F401
