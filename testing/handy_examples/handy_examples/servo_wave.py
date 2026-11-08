#!/usr/bin/env python3
"""Sine-wave MoveIt Servo demo for the Handy arm: what can Servo do in real time?

Patterns (each starts and ends at the WAVE pose, arm reaching forward):
  figure8  end effector traces a Lissajous figure-8 in the Y/Z plane (Cartesian twist)
  helix    circle in Y/Z while drifting in and out along X (Cartesian twist)
  snake    joints 4-7 oscillate with phase offsets: a wave travels down the arm (joint jog)
  hello    wrist roll waves while the wrist nods, and the gripper "breathes" open/closed

Every pattern streams velocities that are the derivative of a sine (a cosine), over whole
periods, so the arm returns to where it started; a smooth ramp in/out avoids jerks.

Run with the stack up (needs limits:=full, WAVE is outside the +/-0.3 test window):
    handy sim
    ros2 run handy_examples servo_wave                        # all patterns
    ros2 run handy_examples servo_wave --only figure8 snake   # a subset
    ros2 run handy_examples servo_wave --speed 0.5            # slower (use <= 0.5 on the real arm)

Amplitudes are set by Servo's scales (servo.yaml: linear 0.05 m/s, joint 0.2 rad/s per unit)
and the period: amplitude = peak velocity / angular frequency, so slower waves travel further.
Ctrl+C stops immediately; Servo halts ~0.1 s after commands stop.
"""
import argparse
import math
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.msg import JointJog
from geometry_msgs.msg import TwistStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from handy_examples.poses import HAND, HAND_JOINT, WAVE
from handy_examples.servo_demo import RATE, ServoDemo

TWO_PI = 2.0 * math.pi
RAMP = 1.0  # s, smooth amplitude ramp at the start and end of every pattern


def envelope(t, duration):
    """0 -> 1 -> 0 with sin^2 ramps of RAMP seconds at both ends."""
    if t < RAMP:
        return math.sin(0.5 * math.pi * t / RAMP) ** 2
    if t > duration - RAMP:
        return math.sin(0.5 * math.pi * max(0.0, duration - t) / RAMP) ** 2
    return 1.0


class ServoWave(ServoDemo):
    def __init__(self, speed):
        super().__init__(speed, node_name='servo_wave')
        self.hand_pub = self.create_publisher(
            JointTrajectory, '/hand_controller/joint_trajectory', 10)

    # ---- streaming helpers ----------------------------------------------------
    def stream_twist(self, seconds, fn, label):
        """Stream TwistStamped commands; fn(t) -> ((vx, vy, vz), (wx, wy, wz)), unitless."""
        self.get_logger().info(f'{label} ({seconds:.0f} s)')
        start = time.monotonic()
        while rclpy.ok() and (t := time.monotonic() - start) < seconds:
            rclpy.spin_once(self, timeout_sec=0.0)
            lin, ang = fn(t)
            k = envelope(t, seconds) * self.speed
            msg = self._stamp(TwistStamped())
            msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = (k * v for v in lin)
            msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = (k * v for v in ang)
            self.twist_pub.publish(msg)
            time.sleep(1.0 / RATE)
        self._spin_for(0.5)

    def stream_joints(self, seconds, fn, label):
        """Stream JointJog commands; fn(t) -> {joint: velocity}, unitless."""
        self.get_logger().info(f'{label} ({seconds:.0f} s)')
        start = time.monotonic()
        while rclpy.ok() and (t := time.monotonic() - start) < seconds:
            rclpy.spin_once(self, timeout_sec=0.0)
            k = envelope(t, seconds) * self.speed
            vels = fn(t)
            msg = self._stamp(JointJog())
            msg.joint_names = list(vels)
            msg.velocities = [k * v for v in vels.values()]
            self.joint_pub.publish(msg)
            time.sleep(1.0 / RATE)
        self._spin_for(0.5)

    def breathe_gripper(self, cycles=3, period=2.0):
        """Send one smooth sine trajectory to hand_controller (runs while the arm moves)."""
        mid = 0.5 * (HAND['open'] + HAND['close'])
        amp = 0.5 * (HAND['close'] - HAND['open'])
        traj = JointTrajectory(joint_names=[HAND_JOINT])
        steps = int(cycles * period / 0.1)
        for i in range(1, steps + 1):
            t = i * 0.1
            pos = mid - amp * math.cos(TWO_PI * t / period)  # starts fully open, smooth
            traj.points.append(JointTrajectoryPoint(
                positions=[pos],
                time_from_start=Duration(sec=int(t + 0.5), nanosec=int(((t + 0.5) % 1) * 1e9))))
        traj.points.append(JointTrajectoryPoint(
            positions=[HAND['neutral']],
            time_from_start=Duration(sec=int(steps * 0.1 + 1.5))))
        self.hand_pub.publish(traj)

    # ---- patterns -------------------------------------------------------------
    def figure8(self, period=8.0, cycles=2):
        w = TWO_PI / period
        # y = sin(wt), z = sin(2wt)/2  ->  velocities are their derivatives (normalized)
        self.stream_twist(
            period * cycles,
            lambda t: ((0.0, math.cos(w * t), math.cos(2 * w * t)), (0.0, 0.0, 0.0)),
            'figure-8 in the Y/Z plane')

    def helix(self, period=6.0, cycles=3):
        w = TWO_PI / period
        wx = w / cycles  # one in-and-out drift over the whole helix
        self.stream_twist(
            period * cycles,
            lambda t: ((0.4 * math.cos(wx * t), math.cos(w * t), -math.sin(w * t)),
                       (0.0, 0.0, 0.0)),
            'helix: circles in Y/Z, drifting along X')

    def snake(self, period=8.0, cycles=2):
        w = TWO_PI / period
        joints = ['joint_4', 'joint_5', 'joint_6', 'joint_7']
        phase = math.pi / 3  # each joint lags the previous one: a travelling wave
        self.stream_joints(
            period * cycles,
            lambda t: {j: math.cos(w * t - i * phase) for i, j in enumerate(joints)},
            'snake: a wave travelling down joints 4-7')

    def hello(self, period=6.0, cycles=2):
        w = TWO_PI / period
        self.breathe_gripper(cycles=3, period=period * cycles / 3)
        self.stream_joints(
            period * cycles,
            lambda t: {'joint_7': math.cos(w * t), 'joint_6': 0.5 * math.cos(0.5 * w * t)},
            'hello: wrist roll wave + nod, gripper breathing')

    def run(self, patterns):
        self.wait_for_joints()
        if not self.go_to(WAVE, label='wave pose'):
            return
        for name in patterns:
            if not rclpy.ok():
                break
            getattr(self, name)()
            self.go_to(WAVE, label='wave pose')  # re-center: removes drift before the next one
        self.get_logger().info('Wave demo complete')


PATTERNS = ['figure8', 'helix', 'snake', 'hello']


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--speed', type=float, default=1.0,
                        help="0-1 fraction of Servo's scale "
                             '(default 1.0; use <= 0.5 on the real arm)')
    parser.add_argument('--only', nargs='+', choices=PATTERNS, default=PATTERNS,
                        help='run only these patterns, in this order')
    args = parser.parse_args()
    rclpy.init()
    node = ServoWave(max(0.0, min(1.0, args.speed)))
    try:
        node.run(args.only)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
