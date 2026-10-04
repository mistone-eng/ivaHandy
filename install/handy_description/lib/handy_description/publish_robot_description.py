#!/usr/bin/env python3
import sys
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String


def main():
    rclpy.init()
    node = rclpy.create_node('publish_robot_description')
    qos = QoSProfile(depth=1)
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    pub = node.create_publisher(String, 'robot_description', qos)

    if len(sys.argv) < 2:
        node.get_logger().error('No robot_description provided as argument')
        return

    # The whole URDF is passed as a single argument from the launch substitution
    data = sys.argv[1]
    msg = String()
    msg.data = data

    # publish a few times to ensure late subscribers receive it
    for _ in range(5):
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.1)

    node.get_logger().info('Published /robot_description (transient_local)')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
