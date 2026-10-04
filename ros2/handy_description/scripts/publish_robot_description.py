#!/usr/bin/env python3
import os
import subprocess
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory


def main():
    rclpy.init()
    node = Node('publish_robot_description')
    try:
        pkg_share = get_package_share_directory('handy_description')
        urdf_path = os.path.join(pkg_share, 'urdf', 'handy_description.urdf.xacro')
        proc = subprocess.run(['xacro', urdf_path], capture_output=True, text=True)
        if proc.returncode != 0:
            node.get_logger().error('xacro failed: %s' % proc.stderr)
            return 1
        xml = proc.stdout
        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        pub = node.create_publisher(String, '/robot_description', qos)
        msg = String()
        msg.data = xml
        # publish several times to ensure late subscribers receive it
        for _ in range(3):
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.05)
        node.get_logger().info('Published /robot_description with transient_local QoS')
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
