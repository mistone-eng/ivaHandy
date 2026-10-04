import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('handy_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'handy_description.urdf.xacro')
    rviz_config = os.path.join(pkg_share, 'config', 'urdf.rviz')

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='False',
        description='Use the joint_state_publisher GUI.'
    )

    use_gui = LaunchConfiguration('gui')
    xacro_cmd = Command([FindExecutable(name='xacro'), ' ', urdf_path])

    publish_description = Node(
        package='handy_description',
        executable='publish_robot_description.py',
        output='screen'
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(use_gui),
        output='screen',
        parameters=[{'robot_description': xacro_cmd}]
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(use_gui),
        output='screen',
        parameters=[{'robot_description': xacro_cmd}]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': xacro_cmd}],
        output='screen'
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        publish_description,
        gui_arg,
        joint_state_publisher,
        joint_state_publisher_gui,
        robot_state_publisher,
        rviz2,
    ])
