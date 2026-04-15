"""Servo-streaming control (ServoP-based).

This module provides a manual streaming-servo tester that complements the
existing point-to-point motion commands. It does NOT replace any existing
control path; MovJ, MovL, jog(), pick, and friends continue to work exactly
as before. See VLA.md for context on when to use each mode.
"""

from dobot_ros.servo.tester import (
    ServoTester,
    ServoConfig,
    ServoStatus,
)
from dobot_ros.servo.patterns import (
    Pattern,
    CirclePattern,
    LissajousPattern,
    SquareWavePattern,
    SineWavePattern,
    build_pattern,
)

__all__ = [
    "ServoTester", "ServoConfig", "ServoStatus",
    "Pattern", "CirclePattern", "LissajousPattern",
    "SquareWavePattern", "SineWavePattern", "build_pattern",
]
