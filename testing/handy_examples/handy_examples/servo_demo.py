#!/usr/bin/env python3
"""Hands-free MoveIt Servo demo for the Handy arm (ROS 2 Humble).

Sequence:
  1. Joint-space: jog to a bent "ready" pose (away from limits/singularities)
  2. Cartesian: trace a square with the end effector (Y/Z plane)
  3. Cartesian: rotate the wrist back and forth about Z
  4. Joint-space: return to the ready pose

Run with the stack up:
    ros2 launch handy_bringup demo.launch.py limits:=full
    ros2 run handy_examples servo_demo              # full demo
    ros2 run handy_examples servo_demo --speed 0.3  # slower (recommended on the real arm)

Requires limits:=full (the ready pose is outside the +/-0.3 test window).
Ctrl+C stops immediately; Servo halts ~0.1 s after commands stop.
"""
import argparse
import time

import rclpy
from control_msgs.msg import JointJog
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8

from handy_examples.poses import READY, SERVO_STATUS as STATUS

RATE = 50.0  # Hz


class ServoDemo(Node):
    def __init__(self, speed, node_name="servo_demo"):
        super().__init__(node_name)
        self.speed = speed
        self.joints = {}
        self.status = 0
        self.twist_pub = self.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
        self.joint_pub = self.create_publisher(JointJog, "/servo_node/delta_joint_cmds", 10)
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.create_subscription(Int8, "/servo_node/status", self._on_status, 10)

    # ---- callbacks / helpers -------------------------------------------
    def _on_js(self, msg):
        self.joints.update(zip(msg.name, msg.position))

    def _on_status(self, msg):
        if msg.data != self.status and msg.data in STATUS:
            self.get_logger().info(f"Servo status: {STATUS[msg.data]}")
        self.status = msg.data

    def _spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(1.0 / RATE)

    def _stamp(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()  # Humble Servo ignores stamp 0
        msg.header.frame_id = "base_link"
        return msg

    # ---- motions ---------------------------------------------------------
    def wait_for_joints(self):
        self.get_logger().info("Waiting for /joint_states ...")
        while rclpy.ok() and not all(j in self.joints for j in READY):
            rclpy.spin_once(self, timeout_sec=0.1)

    def go_to(self, target, tol=0.02, gain=8.0, stall_time=3.0, max_time=120.0,
              label='ready pose'):
        """Closed-loop joint jog to a target pose (P-control on Servo joint jog).

        Runs at full speed until within 1/gain rad of the goal, then tapers. Instead of a fixed
        timeout (which depends on Servo's scale and speed), it gives up only when the largest
        joint error has stopped shrinking for `stall_time` seconds (Servo halted: joint limit,
        collision, singularity) or after `max_time` as a last resort.
        """
        self.get_logger().info(f"Joint jog -> {label}")
        start = time.monotonic()
        best_err, best_time = float("inf"), start
        while rclpy.ok() and time.monotonic() - start < max_time:
            rclpy.spin_once(self, timeout_sec=0.0)
            names, vels = [], []
            max_err = 0.0
            for name, goal in target.items():
                err = goal - self.joints.get(name, goal)
                max_err = max(max_err, abs(err))
                names.append(name)
                vels.append(max(-1.0, min(1.0, gain * err)) * self.speed)
            if max_err <= tol:
                self.get_logger().info(
                    f"Reached {label} in {time.monotonic() - start:.1f} s")
                return True
            now = time.monotonic()
            if max_err < best_err - 0.005:
                best_err, best_time = max_err, now
            elif now - best_time > stall_time:
                self.get_logger().warn(
                    f"Stalled {max_err:.3f} rad from the {label} "
                    f"(Servo status: {STATUS.get(self.status, self.status)})")
                return False
            msg = self._stamp(JointJog())
            msg.joint_names, msg.velocities = names, vels
            self.joint_pub.publish(msg)
            time.sleep(1.0 / RATE)
        self.get_logger().warn(f"{label} not reached within {max_time:.0f} s")
        return False

    def twist(self, lin=(0.0, 0.0, 0.0), ang=(0.0, 0.0, 0.0), seconds=2.0, label=""):
        if label:
            self.get_logger().info(f"Cartesian: {label}")
        end = time.monotonic() + seconds
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            msg = self._stamp(TwistStamped())
            lx, ly, lz = (v * self.speed for v in lin)
            ax, ay, az = (v * self.speed for v in ang)
            msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = lx, ly, lz
            msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = ax, ay, az
            self.twist_pub.publish(msg)
            time.sleep(1.0 / RATE)
        self._spin_for(0.5)  # let Servo settle between legs

    def run(self):
        self.wait_for_joints()
        if not self.go_to(READY):
            return
        self._spin_for(1.0)

        leg = 2.0
        self.twist(lin=(0, +1, 0), seconds=leg, label="square 1/4 (+Y)")
        self.twist(lin=(0, 0, +1), seconds=leg, label="square 2/4 (+Z)")
        self.twist(lin=(0, -1, 0), seconds=leg, label="square 3/4 (-Y)")
        self.twist(lin=(0, 0, -1), seconds=leg, label="square 4/4 (-Z)")

        self.twist(ang=(0, 0, +1), seconds=1.5, label="wrist rotate +Z")
        self.twist(ang=(0, 0, -1), seconds=3.0, label="wrist rotate -Z")
        self.twist(ang=(0, 0, +1), seconds=1.5, label="wrist rotate back")

        self.go_to(READY)
        self.get_logger().info("Demo complete")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--speed", type=float, default=0.5,
                        help="0-1 fraction of Servo's scale (use 0.3 on the real arm)")
    args = parser.parse_args()
    rclpy.init()
    node = ServoDemo(max(0.0, min(1.0, args.speed)))
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
