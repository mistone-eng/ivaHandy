"""Patterns are pure: they must return to the start and respect the envelope."""
import math

import pytest

from handy_api import patterns

HAND = {'open': -0.514, 'close': 0.125, 'neutral': 0.0}


def _flat(p, v):
    return list(v[0]) + list(v[1]) if p.kind == 'twist' else list(v.values())


def _integrate(p, dt=0.001):
    """Integrate the unitless velocity over the pattern -> net displacement (unitless * s)."""
    t, total = 0.0, None
    while t < p.duration:
        flat = _flat(p, p.sample(t))
        total = [b * dt for b in flat] if total is None else [
            a + b * dt for a, b in zip(total, flat)]
        t += dt
    return total


@pytest.mark.parametrize('name', ['figure8', 'helix', 'snake', 'hello'])
def test_builtin_patterns_return_to_start(name):
    p = patterns.builtin(HAND)[name]
    disp = _integrate(p)
    # drift-corrected: the arm ends where it started (vs. ~0.95 before the correction)
    assert max(abs(x) for x in disp) < 0.01, disp


@pytest.mark.parametrize('name', ['figure8', 'helix', 'snake', 'hello'])
def test_correction_keeps_most_of_the_motion(name):
    p = patterns.builtin(HAND)[name]
    steps = range(int(p.duration * 20))
    peak = max(max(abs(x) for x in _flat(p, p.sample(k / 20))) for k in steps)
    assert 0.85 < peak <= 1.0


@pytest.mark.parametrize('name', ['figure8', 'helix', 'snake', 'hello'])
def test_commands_are_unitless_and_ramped(name):
    p = patterns.builtin(HAND)[name]
    for k in range(0, int(p.duration * 20) + 1):
        v = p.sample(k / 20)
        flat = list(v[0]) + list(v[1]) if p.kind == 'twist' else list(v.values())
        assert all(abs(x) <= 1.0 + 1e-9 for x in flat)
    start = p.sample(0.0)
    flat = list(start[0]) + list(start[1]) if p.kind == 'twist' else list(start.values())
    assert all(abs(x) < 1e-9 for x in flat)  # envelope starts at zero


def test_envelope_shape():
    assert patterns.envelope(0.0, 10) == 0.0
    assert patterns.envelope(5.0, 10) == 1.0
    assert patterns.envelope(10.0, 10) == pytest.approx(0.0)
    assert 0.0 < patterns.envelope(0.5, 10) < 1.0


def test_hello_gripper_waypoints_stay_in_range_and_end_neutral():
    p = patterns.builtin(HAND)['hello']
    times = [t for t, _ in p.gripper]
    assert times == sorted(times)
    assert all(HAND['open'] - 1e-9 <= x <= HAND['close'] + 1e-9 for _, x in p.gripper)
    assert p.gripper[-1][1] == HAND['neutral']


def test_custom_pattern_and_kind_validation():
    p = patterns.custom('wiggle', 4.0, 'joint', lambda t: {'joint_7': math.cos(t)})
    assert p.start_pose is None and set(p.sample(2.0)) == {'joint_7'}
    with pytest.raises(ValueError):
        patterns.custom('bad', 1.0, 'teleport', lambda t: None)
