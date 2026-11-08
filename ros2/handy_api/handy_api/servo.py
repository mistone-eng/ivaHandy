"""MoveIt Servo streaming: velocity commands for vision loops, patterns and quick joint moves.

Commands are given in real units (m/s, rad/s) and converted to Servo's unitless commands with the
servo.yaml scales. A 50 Hz keep-alive repeats the latest command, so a caller (e.g. a camera loop
at 15 Hz) only updates when it has new information. If a command isn't refreshed within
`config.command_timeout`, it is dropped and Servo stops the arm by itself (dead-man behavior).

Servo only moves the arm while servo_controller is active (Modes.servo()).
"""
import threading
import time
from typing import Callable, Dict, Optional

import numpy as np
from control_msgs.msg import JointJog
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import Trigger

from handy_api import safety
from handy_api.config import RobotConfig
from handy_api.node import HandyNode, wait_future, wait_until

RATE = 50.0  # Hz


class ServoClient:
    def __init__(self, hn: HandyNode, config: RobotConfig, state=None):
        self._hn, self._cfg, self._state = hn, config, state
        node = hn.node
        self._twist_pub = node.create_publisher(TwistStamped, config.servo_twist_topic, 10)
        self._joint_pub = node.create_publisher(JointJog, config.servo_joint_topic, 10)
        self._start = node.create_client(Trigger, config.servo_start_service,
                                         callback_group=hn.cb_group)
        self._lock = threading.Lock()
        self._cmd = None  # ('twist', lin, ang, frame) | ('joint', {j: v}); unitless
        self._cmd_time = 0.0
        node.create_timer(1.0 / RATE, self._tick, callback_group=hn.cb_group)

    # ---- lifecycle -------------------------------------------------------------------
    def start(self, timeout: float = 10.0):
        """Make sure Servo is running (the launch files start it; this is idempotent)."""
        if not self._start.wait_for_service(timeout_sec=timeout):
            raise TimeoutError('MoveIt Servo (servo_node) not available -- is the stack running?')
        wait_future(self._start.call_async(Trigger.Request()), timeout, 'start_servo')

    def wait_ready(self, timeout: float = 10.0):
        if self._state is not None:
            wait_until(lambda: self._state.servo_status() is not None, timeout, 'Servo status')

    # ---- commands in real units ------------------------------------------------------
    def set_twist(self, linear=(0.0, 0.0, 0.0), angular=(0.0, 0.0, 0.0),
                  frame: Optional[str] = None):
        """End-effector velocity: linear m/s, angular rad/s, in `frame` (default: base frame).

        Clamped to Servo's scale (servo.yaml) and, if configured, to the workspace box.
        Keep calling it (any rate > 1/command_timeout); stop with stop().
        """
        frame = frame or self._cfg.base_frame
        lin = safety.clamp_norm(linear, self._cfg.servo_scale_linear)
        ang = safety.clamp_norm(angular, self._cfg.servo_scale_rotational)
        if self._cfg.workspace is not None and frame == self._cfg.base_frame and self._state:
            try:
                lin = safety.limit_to_workspace(self._state.eef_position(), lin,
                                                self._cfg.workspace)
            except Exception:  # no TF yet: refuse to move rather than ignore the box
                lin = np.zeros(3)
        self.set_twist_unitless(lin / self._cfg.servo_scale_linear,
                                ang / self._cfg.servo_scale_rotational, frame)

    def set_joint_velocity(self, velocities: Dict[str, float]):
        """Joint velocities in rad/s, clamped to Servo's joint scale. Keep calling; stop()."""
        v = safety.clamp_joint_velocities(velocities, self._cfg.servo_scale_joint)
        scale = self._cfg.servo_scale_joint
        self.set_joint_velocity_unitless({j: x / scale for j, x in v.items()})

    # ---- commands as fractions of Servo's scale ([-1, 1]) -----------------------------
    def set_twist_unitless(self, linear, angular, frame: Optional[str] = None):
        lin = np.clip(np.asarray(linear, float), -1.0, 1.0)
        ang = np.clip(np.asarray(angular, float), -1.0, 1.0)
        with self._lock:
            self._cmd = ('twist', lin, ang, frame or self._cfg.base_frame)
            self._cmd_time = time.monotonic()

    def set_joint_velocity_unitless(self, velocities: Dict[str, float]):
        unknown = set(velocities) - set(self._cfg.arm_joints)
        if unknown:
            raise ValueError(f'not arm joints: {sorted(unknown)}')
        with self._lock:
            self._cmd = ('joint', {j: float(np.clip(v, -1.0, 1.0)) for j, v in velocities.items()})
            self._cmd_time = time.monotonic()

    def stop(self):
        """Stop streaming; Servo halts the arm (and an explicit zero command is sent once)."""
        with self._lock:
            self._cmd = None
        self._publish(('twist', np.zeros(3), np.zeros(3), self._cfg.base_frame))

    def active(self) -> bool:
        with self._lock:
            return self._cmd is not None

    # ---- blocking helpers ---------------------------------------------------------------
    def stream(self, sample: Callable[[float], object], duration: float, kind: str,
               speed: float = 1.0, settle: float = 0.5):
        """Stream sample(t) (unitless twist ((lin), (ang)) or {joint: v}) for `duration` s."""
        start = time.monotonic()
        while (t := time.monotonic() - start) < duration:
            value = sample(t)
            if kind == 'twist':
                lin, ang = value
                self.set_twist_unitless(speed * np.asarray(lin), speed * np.asarray(ang))
            else:
                self.set_joint_velocity_unitless({j: speed * v for j, v in value.items()})
            time.sleep(1.0 / RATE)
        self.stop()
        time.sleep(settle)

    def go_to(self, target: Dict[str, float], speed: float = 0.5, tol: float = 0.02,
              gain: float = 8.0, stall_time: float = 3.0, max_time: float = 120.0) -> bool:
        """Closed-loop joint move through Servo (no planning). Returns True when within `tol`.

        Full speed until within 1/gain rad, then tapers; gives up when the largest error stops
        shrinking for `stall_time` s (Servo halted: limit, collision, singularity).
        """
        if self._state is None:
            raise RuntimeError('go_to needs State')
        safety.check_joint_target(target, self._cfg.joint_limits)
        start = time.monotonic()
        best_err, best_time = float('inf'), start
        try:
            while time.monotonic() - start < max_time:
                joints = self._state.joints()
                errs = {j: goal - joints.get(j, goal) for j, goal in target.items()}
                max_err = max(abs(e) for e in errs.values())
                if max_err <= tol:
                    return True
                now = time.monotonic()
                if max_err < best_err - 0.005:
                    best_err, best_time = max_err, now
                elif now - best_time > stall_time:
                    return False
                self.set_joint_velocity_unitless(
                    {j: float(np.clip(gain * e, -1.0, 1.0)) * speed for j, e in errs.items()})
                time.sleep(1.0 / RATE)
            return False
        finally:
            self.stop()

    # ---- keep-alive -----------------------------------------------------------------------
    def _tick(self):
        with self._lock:
            cmd = self._cmd
            if cmd is not None and time.monotonic() - self._cmd_time > self._cfg.command_timeout:
                self._cmd = cmd = None  # caller stopped refreshing: let Servo halt
        if cmd is not None:
            self._publish(cmd)

    def _publish(self, cmd):
        stamp = self._hn.now().to_msg()  # Servo ignores zero stamps
        if cmd[0] == 'twist':
            _, lin, ang, frame = cmd
            msg = TwistStamped()
            msg.header.stamp, msg.header.frame_id = stamp, frame
            msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = (float(v) for v in lin)
            msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = (float(v) for v in ang)
            self._twist_pub.publish(msg)
        else:
            msg = JointJog()
            msg.header.stamp, msg.header.frame_id = stamp, self._cfg.base_frame
            msg.joint_names = list(cmd[1])
            msg.velocities = list(cmd[1].values())
            self._joint_pub.publish(msg)
