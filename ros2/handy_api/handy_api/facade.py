"""Handy: the one object most scripts need. Each part is also usable on its own.

    from handy_api import Handy

    with Handy() as h:                   # connect: joint states + Servo, servo mode
        h.move_to('wave')                # planned Pilz PTP move
        h.gripper.open()
        h.servo.set_twist(linear=(0, 0.02, 0))   # m/s; keep calling from your loop
        ...
        h.play('figure8')                # a built-in pattern

Parts: h.state (joints, end-effector pose, Servo status), h.tf (frame transforms), h.servo
(velocity streaming, joint go_to), h.motion (planned moves), h.gripper, h.modes (servo/plan).
"""
from typing import Optional, Union

from handy_api import patterns as pattern_lib
from handy_api.config import RobotConfig, load_handy_config
from handy_api.controllers import Modes
from handy_api.gripper import Gripper
from handy_api.motion import Planner
from handy_api.node import HandyNode
from handy_api.servo import ServoClient
from handy_api.state import State
from handy_api.transforms import Transforms


class Handy:
    def __init__(self, config: Optional[RobotConfig] = None, limits: str = 'full',
                 node_name: str = 'handy_api', connect: bool = True, timeout: float = 15.0):
        """config: a RobotConfig (default: Handy's, with `limits` 'full' or 'test' matching the
        running stack). connect: wait for joint states and Servo, and switch to servo mode."""
        self.config = config or load_handy_config(limits)
        self._hn = HandyNode(node_name)
        self.state = State(self._hn, self.config)
        self.tf = Transforms(self.state.tf_buffer, self.config)
        self.modes = Modes(self._hn, self.config)
        self.servo = ServoClient(self._hn, self.config, self.state)
        self.motion = Planner(self._hn, self.config, self.modes)
        self.gripper = Gripper(self._hn, self.config)
        self.patterns = pattern_lib.builtin(self.config.hand_positions)
        if connect:
            self.connect(timeout)

    @property
    def log(self):
        return self._hn.logger

    def connect(self, timeout: float = 15.0):
        self.state.wait_for_joints(timeout)
        self.servo.wait_ready(timeout)
        self.modes.servo()

    # ---- convenience -------------------------------------------------------------------
    def move_to(self, target, **kwargs):
        """Planned move (see Planner.move_to): named pose, {joint: rad} or Pose."""
        return self.motion.move_to(target, **kwargs)

    def go_to(self, target: Union[str, dict], speed: float = 0.5, **kwargs) -> bool:
        """Joint move through Servo (no planning): named pose or {joint: rad}."""
        if isinstance(target, str):
            target = self.config.named_poses[target]
        return self.servo.go_to(target, speed=speed, **kwargs)

    def play(self, pattern: Union[str, pattern_lib.Pattern], speed: float = 1.0,
             recenter: bool = True) -> bool:
        """Run a pattern (name of a built-in or a Pattern) through Servo."""
        p = self.patterns[pattern] if isinstance(pattern, str) else pattern
        if p.start_pose and not self.go_to(p.start_pose, speed=speed):
            self.log.warn(f'{p.name}: could not reach {p.start_pose!r}')
            return False
        self.log.info(f'{p.name}: {p.description} ({p.duration:.0f} s)')
        if p.gripper:
            self.gripper.follow(p.gripper, wait=False)
        self.servo.stream(p.sample, p.duration, p.kind, speed)
        if recenter and p.start_pose:
            self.go_to(p.start_pose, speed=speed)
        return True

    def stop(self):
        self.servo.stop()

    def close(self):
        try:
            self.servo.stop()
        finally:
            self._hn.shutdown()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False
