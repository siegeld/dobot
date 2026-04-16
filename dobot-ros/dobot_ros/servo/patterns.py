"""Pattern generators for the servo tester.

Each pattern emits an absolute cartesian pose over time. Patterns return pose
relative to the "anchor" pose captured when the pattern was launched — so a
circle drawn around the robot's starting position, not around (0,0,0).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List


@dataclass
class Pattern:
    """Base class. Override `target_at(t)` to return the delta from anchor."""
    duration: float = 0.0  # 0 = infinite / hold

    def target_at(self, t: float) -> List[float]:
        """Return 6-D pose offset from the anchor at elapsed time t (seconds)."""
        return [0.0] * 6

    def finished(self, t: float) -> bool:
        return self.duration > 0 and t >= self.duration


@dataclass
class CirclePattern(Pattern):
    """Trace a circle in the XY plane of the user frame.

    Z, RX, RY, RZ are held. Positive period = CCW from +X.
    """
    radius_mm: float = 40.0
    period_s: float = 4.0
    plane: str = "xy"  # "xy" | "xz" | "yz"

    def target_at(self, t: float) -> List[float]:
        theta = 2.0 * math.pi * t / self.period_s
        # Circle centered on (anchor - radius) along the first axis.
        # At t=0: offset=(0,0) — starts at anchor with no jump.
        # Peak excursion from anchor is 2*radius along the first axis.
        c = self.radius_mm * (math.cos(theta) - 1.0)
        s = self.radius_mm * math.sin(theta)
        if self.plane == "xy":
            return [c, s, 0.0, 0.0, 0.0, 0.0]
        if self.plane == "xz":
            return [c, 0.0, s, 0.0, 0.0, 0.0]
        return [0.0, c, s, 0.0, 0.0, 0.0]


@dataclass
class LissajousPattern(Pattern):
    """Lissajous curve in XY: x = A sin(a*t + phi), y = B sin(b*t)."""
    amplitude_x_mm: float = 40.0
    amplitude_y_mm: float = 40.0
    freq_x_hz: float = 0.25
    freq_y_hz: float = 0.33
    phase_deg: float = 90.0

    def target_at(self, t: float) -> List[float]:
        ax = 2.0 * math.pi * self.freq_x_hz
        ay = 2.0 * math.pi * self.freq_y_hz
        phi = math.radians(self.phase_deg)
        x = self.amplitude_x_mm * math.sin(ax * t + phi)
        y = self.amplitude_y_mm * math.sin(ay * t)
        return [x, y, 0.0, 0.0, 0.0, 0.0]


@dataclass
class SquareWavePattern(Pattern):
    """Step back and forth along one axis. Measures settling / overshoot.

    axis in {"x","y","z","rx","ry","rz"}.
    """
    axis: str = "x"
    amplitude: float = 30.0   # mm for xyz, deg for rotations
    period_s: float = 4.0

    _axis_index = {"x": 0, "y": 1, "z": 2, "rx": 3, "ry": 4, "rz": 5}

    def target_at(self, t: float) -> List[float]:
        half = self.period_s / 2.0
        phase = (t % self.period_s) < half
        out = [0.0] * 6
        idx = self._axis_index[self.axis.lower()]
        out[idx] = self.amplitude if phase else -self.amplitude
        return out


@dataclass
class SineWavePattern(Pattern):
    """Single-axis sine. Sweep frequency manually to find tracking bandwidth."""
    axis: str = "x"
    amplitude: float = 30.0   # mm or deg
    freq_hz: float = 0.5

    _axis_index = {"x": 0, "y": 1, "z": 2, "rx": 3, "ry": 4, "rz": 5}

    def target_at(self, t: float) -> List[float]:
        omega = 2.0 * math.pi * self.freq_hz
        val = self.amplitude * math.sin(omega * t)
        out = [0.0] * 6
        out[self._axis_index[self.axis.lower()]] = val
        return out


_MAX_XYZ_AMPLITUDE_MM = 200.0
_MAX_ROT_AMPLITUDE_DEG = 30.0


def build_pattern(name: str, params: dict) -> Pattern:
    """Dispatch a pattern by name with keyword params from the web UI.
    Clamps amplitudes to prevent dangerous excursions."""
    name = name.lower()
    cls = {
        "circle": CirclePattern,
        "lissajous": LissajousPattern,
        "square": SquareWavePattern,
        "sine": SineWavePattern,
    }.get(name)
    if cls is None:
        raise ValueError(f"unknown pattern: {name}")

    params = dict(params or {})

    # Clamp XYZ amplitudes/radii.
    for key in ("radius_mm", "amplitude_x_mm", "amplitude_y_mm", "amplitude"):
        if key in params:
            params[key] = min(float(params[key]), _MAX_XYZ_AMPLITUDE_MM)

    # Prevent division by zero in period/frequency.
    if "period_s" in params:
        params["period_s"] = max(0.1, float(params["period_s"]))
    if "freq_hz" in params:
        params["freq_hz"] = max(0.0, float(params["freq_hz"]))  # 0 is safe (sine=0)

    # Validate axis for square/sine patterns.
    if "axis" in params:
        axis = str(params["axis"]).lower()
        if axis not in ("x", "y", "z", "rx", "ry", "rz"):
            raise ValueError(f"invalid axis: {axis}")
        params["axis"] = axis
        # For rotation axes, clamp amplitude to rotation limit.
        if axis in ("rx", "ry", "rz") and "amplitude" in params:
            params["amplitude"] = min(float(params["amplitude"]), _MAX_ROT_AMPLITUDE_DEG)

    return cls(**params)
