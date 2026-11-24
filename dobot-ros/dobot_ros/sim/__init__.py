"""Kinematic simulator for the CR5 + AG-105.

Implements the same ``Robot`` / ``World`` protocols as the real adapters in
``dobot_ros.real`` so a policy runs unchanged on either. No physics engine:
cartesian interpolation, floor/workspace guards, a geometric grasp rule, and
a perception noise model. See POLICIES.md.
"""
