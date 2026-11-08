"""Motion patterns as pure functions of time (no ROS): what ServoClient.play() streams.

A Pattern produces *unitless* velocity commands in [-1, 1] (fractions of Servo's scale) for
`duration` seconds: either a twist ((vx, vy, vz), (wx, wy, wz)) in the base frame, or joint
velocities {joint: v}. Velocities are derivatives of sines (cosines) over whole periods, so the arm
ends where it started; `envelope` ramps them in and out smoothly.

Amplitude = peak velocity / angular frequency: Servo's scales cap the velocity, so slower periods
travel further.
"""
import math
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence, Tuple

TWO_PI = 2.0 * math.pi
RAMP = 1.0  # s


def envelope(t: float, duration: float, ramp: float = RAMP) -> float:
    """0 -> 1 -> 0 with sin^2 ramps of `ramp` seconds at both ends."""
    if t < ramp:
        return math.sin(0.5 * math.pi * t / ramp) ** 2
    if t > duration - ramp:
        return math.sin(0.5 * math.pi * max(0.0, duration - t) / ramp) ** 2
    return 1.0


Twist = Tuple[Tuple[float, float, float], Tuple[float, float, float]]


@dataclass
class Pattern:
    name: str
    duration: float
    kind: str                                   # 'twist' or 'joint'
    fn: Callable[[float], object]               # t -> Twist or {joint: v}, unitless
    description: str = ''
    start_pose: Optional[str] = None            # named pose to start from (None = current)
    gripper: List[Tuple[float, float]] = field(default_factory=list)  # (t, position) waypoints

    def __post_init__(self):
        if self.kind not in ('twist', 'joint'):
            raise ValueError("kind must be 'twist' or 'joint'")
        # Drift correction. The envelope suppresses the start and end of each wave, so
        # integral(envelope * raw) != 0 and the arm would not return to its start. Subtract a
        # constant bias, weighted by the same envelope, that cancels the net displacement exactly,
        # then rescale so commands stay within [-1, 1].
        self._keys = None
        dt, t = 0.002, 0.0
        disp = env = None
        while t < self.duration:
            e = envelope(t, self.duration)
            raw = self._flatten(self.fn(t))
            disp = [e * r * dt for r in raw] if disp is None else [
                d + e * r * dt for d, r in zip(disp, raw)]
            env = e * dt if env is None else env + e * dt
            t += dt
        self._bias = [d / env for d in disp] if env else [0.0] * len(disp or [])
        self._norm = 1.0 + max((abs(b) for b in self._bias), default=0.0)

    def _flatten(self, value) -> List[float]:
        if self.kind == 'twist':
            lin, ang = value
            return [float(v) for v in lin] + [float(v) for v in ang]
        if self._keys is None:
            self._keys = list(value)
        return [float(value[k]) for k in self._keys]

    def sample(self, t: float):
        """Command at time t: envelope applied, drift-corrected, within [-1, 1]."""
        k = envelope(t, self.duration) / self._norm
        flat = [k * (r - b) for r, b in zip(self._flatten(self.fn(t)), self._bias)]
        if self.kind == 'twist':
            return tuple(flat[:3]), tuple(flat[3:])
        return dict(zip(self._keys, flat))


def figure8(period: float = 8.0, cycles: int = 2, start_pose: Optional[str] = None) -> Pattern:
    w = TWO_PI / period
    return Pattern('figure8', period * cycles, 'twist',
                   lambda t: ((0.0, math.cos(w * t), math.cos(2 * w * t)), (0.0, 0.0, 0.0)),
                   'Lissajous figure-8 in the Y/Z plane', start_pose)


def helix(period: float = 6.0, cycles: int = 3, start_pose: Optional[str] = None) -> Pattern:
    w = TWO_PI / period
    wx = w / cycles  # one in-and-out drift over the whole helix
    return Pattern('helix', period * cycles, 'twist',
                   lambda t: ((0.4 * math.cos(wx * t), math.cos(w * t), -math.sin(w * t)),
                              (0.0, 0.0, 0.0)),
                   'circles in Y/Z, drifting along X', start_pose)


def snake(joints: Sequence[str], period: float = 8.0, cycles: int = 2,
          phase: float = math.pi / 3, start_pose: Optional[str] = None) -> Pattern:
    w = TWO_PI / period
    return Pattern('snake', period * cycles, 'joint',
                   lambda t: {j: math.cos(w * t - i * phase) for i, j in enumerate(joints)},
                   'a wave travelling down the joints', start_pose)


def breathing(open_pos: float, close_pos: float, duration: float, cycles: int = 3,
              neutral: float = 0.0, step: float = 0.1) -> List[Tuple[float, float]]:
    """Gripper waypoints: smooth open/close cycles, then back to neutral."""
    mid, amp = 0.5 * (open_pos + close_pos), 0.5 * (close_pos - open_pos)
    period = duration / cycles
    points = [(i * step + 0.5, mid - amp * math.cos(TWO_PI * i * step / period))
              for i in range(1, int(duration / step) + 1)]
    points.append((duration + 1.5, neutral))
    return points


def hello(hand: Dict[str, float], roll: str, nod: str, period: float = 6.0, cycles: int = 2,
          start_pose: Optional[str] = None) -> Pattern:
    w = TWO_PI / period
    duration = period * cycles
    return Pattern('hello', duration, 'joint',
                   lambda t: {roll: math.cos(w * t), nod: 0.5 * math.cos(0.5 * w * t)},
                   'wrist roll wave + nod, gripper breathing', start_pose,
                   gripper=breathing(hand['open'], hand['close'], duration, 3, hand['neutral']))


def custom(name: str, duration: float, kind: str, fn, start_pose: Optional[str] = None,
           description: str = '') -> Pattern:
    """Your own pattern: fn(t) returns a unitless Twist or {joint: v}; the envelope is added."""
    return Pattern(name, duration, kind, fn, description, start_pose)


def builtin(hand: Dict[str, float], joints: Dict[str, Sequence[str]],
            start_pose: Optional[str] = None) -> Dict[str, Pattern]:
    """The built-in patterns for a robot: joints = {'snake': [...], 'roll': [j], 'nod': [j]}
    (RobotConfig.pattern_joints), hand = RobotConfig.hand_positions."""
    return {
        'figure8': figure8(start_pose=start_pose),
        'helix': helix(start_pose=start_pose),
        'snake': snake(joints['snake'], start_pose=start_pose),
        'hello': hello(hand, joints['roll'][0], joints['nod'][0], start_pose=start_pose),
    }
