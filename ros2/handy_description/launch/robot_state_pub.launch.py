import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('handy_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'handy_description.urdf.xacro')
    robot_description = Command([FindExecutable(name='xacro'), ' ', urdf_path])

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        ),
    ])