"""Unit tests for pattern generators. Pure math — no threads."""

from __future__ import annotations

import math

from dobot_ros.servo.patterns import (
    CirclePattern, LissajousPattern, SquareWavePattern, SineWavePattern,
    build_pattern,
)


def test_circle_starts_at_origin_and_returns():
    p = CirclePattern(radius_mm=40.0, period_s=4.0, plane="xy")
    # At t=0 the circle starts at the anchor (offset [0,0,...]).
    off = p.target_at(0.0)
    assert off == [0.0] * 6 or (abs(off[0]) < 1e-9 and abs(off[1]) < 1e-9)

    # After one full period, we return to the start.
    full = p.target_at(4.0)
    assert all(abs(full[i] - 0.0) < 1e-6 for i in range(3))

    # After a quarter period, Y should be at ~radius (approx), X at ~ -radius.
    q = p.target_at(1.0)
    assert abs(q[1] - 40.0) < 1e-3
    assert abs(q[0] - (-40.0)) < 1e-3


def test_circle_plane_switch():
    p = CirclePattern(radius_mm=10.0, period_s=4.0, plane="xz")
    off = p.target_at(1.0)
    assert abs(off[1]) < 1e-9            # Y untouched
    assert abs(off[2] - 10.0) < 1e-3     # Z is the second circle axis


def test_square_wave_toggles():
    p = SquareWavePattern(axis="x", amplitude=30.0, period_s=2.0)
    assert p.target_at(0.1)[0] == 30.0
    assert p.target_at(1.1)[0] == -30.0
    assert p.target_at(2.1)[0] == 30.0


def test_sine_amplitude_bounded():
    p = SineWavePattern(axis="y", amplitude=25.0, freq_hz=1.0)
    for t in [0.0, 0.1, 0.25, 0.5, 0.75, 1.0]:
        off = p.target_at(t)
        assert abs(off[1]) <= 25.0 + 1e-9
        # All other axes should stay zero.
        for i in [0, 2, 3, 4, 5]:
            assert off[i] == 0.0


def test_lissajous_shape():
    p = LissajousPattern(
        amplitude_x_mm=20.0, amplitude_y_mm=15.0,
        freq_x_hz=0.5, freq_y_hz=1.0, phase_deg=0.0,
    )
    for t in [0.0, 0.3, 0.7]:
        off = p.target_at(t)
        assert abs(off[0]) <= 20.0 + 1e-9
        assert abs(off[1]) <= 15.0 + 1e-9


def test_build_pattern_dispatch():
    c = build_pattern("circle", {"radius_mm": 5, "period_s": 2})
    assert isinstance(c, CirclePattern)
    assert c.radius_mm == 5

    import pytest
    with pytest.raises(ValueError):
        build_pattern("nope", {})


def test_pattern_duration_none_means_infinite():
    p = CirclePattern(radius_mm=10.0, period_s=4.0)
    assert p.finished(0.0) is False
    assert p.finished(100.0) is False  # duration=0 => never finished


def test_pattern_duration_respected():
    p = SineWavePattern(axis="x", amplitude=1.0, freq_hz=1.0, duration=2.0)
    assert not p.finished(1.99)
    assert p.finished(2.01)
