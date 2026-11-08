#!/usr/bin/env python3
"""Stream properly-stamped test commands to MoveIt Servo (Humble).

Usage (with the stack up, e.g. `handy sim`):
    ros2 run handy_examples servo_jog joint            # jog joint_2 at +0.5
    ros2 run handy_examples servo_jog joint joint_4 -0.3
    ros2 run handy_examples servo_jog twist            # +Z at 0.3 in base_link
    ros2 run handy_examples servo_jog twist x 0.2      # linear x/y/z or angular rx/ry/rz
Ctrl+C to stop; Servo halts ~0.1 s after commands stop.
"""
import sys

import rclpy
from control_msgs.msg import JointJog
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node


class ServoJog(Node):
    def __init__(self, mode, axis, value):
        super().__init__("servo_jog")
        self.mode, self.axis, self.value = mode, axis, value
        if mode == "joint":
            self.pub = self.create_publisher(JointJog, "/servo_node/delta_joint_cmds", 10)
        else:
            self.pub = self.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
        self.create_timer(1.0 / 50.0, self.tick)  # 50 Hz
        self.get_logger().info(f"Streaming {mode} {axis}={value} (Ctrl+C to stop)")

    def tick(self):
        now = self.get_clock().now().to_msg()  # the key part: a real, current stamp
        if self.mode == "joint":
            msg = JointJog()
            msg.header.stamp = now
            msg.header.frame_id = "base_link"
            msg.joint_names = [self.axis]
            msg.velocities = [self.value]
        else:
            msg = TwistStamped()
            msg.header.stamp = now
            msg.header.frame_id = "base_link"
            target = {
                "x": msg.twist.linear, "y": msg.twist.linear, "z": msg.twist.linear,
                "rx": msg.twist.angular, "ry": msg.twist.angular, "rz": msg.twist.angular,
            }[self.axis]
            setattr(target, self.axis[-1], self.value)
        self.pub.publish(msg)


def main():
    args = sys.argv[1:]
    mode = args[0] if args else "joint"
    if mode == "joint":
        axis = args[1] if len(args) > 1 else "joint_2"
        value = float(args[2]) if len(args) > 2 else 0.5
    else:
        axis = args[1] if len(args) > 1 else "z"
        value = float(args[2]) if len(args) > 2 else 0.3
    rclpy.init()
    node = ServoJog(mode, axis, value)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
