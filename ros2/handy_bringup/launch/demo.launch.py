"""Full Handy stack: hardware (handy_control) + MoveIt + RViz + Servo.

This is the ONLY launch file that composes layers; each included file also runs on its own.

  ros2 launch handy_bringup demo.launch.py                              # mock, test limits
  ros2 launch handy_bringup demo.launch.py limits:=full
  ros2 launch handy_bringup demo.launch.py use_mock:=false              # REAL (use `handy real`)
  ros2 launch handy_bringup demo.launch.py rviz:=false servo:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include(package, launch_file, arguments, condition=None):
    path = os.path.join(get_package_share_directory(package), 'launch', launch_file)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path),
        launch_arguments=arguments.items(),
        condition=condition,
    )


def generate_launch_description():
    use_mock = LaunchConfiguration('use_mock')
    limits = LaunchConfiguration('limits')

    return LaunchDescription([
        DeclareLaunchArgument('use_mock', default_value='true',
                              description='true = mock hardware, false = real Dynamixels'),
        DeclareLaunchArgument('limits', default_value='test',
                              description="joint limits for MoveIt/Servo: 'test' or 'full'"),
        DeclareLaunchArgument('rviz', default_value='true', description='start RViz'),
        DeclareLaunchArgument('servo', default_value='true', description='start MoveIt Servo'),

        _include('handy_control', 'control.launch.py', {'use_mock': use_mock}),
        _include('handy_moveit_config', 'move_group.launch.py', {'limits': limits}),
        _include('handy_moveit_config', 'moveit_rviz.launch.py', {'limits': limits},
                 IfCondition(LaunchConfiguration('rviz'))),
        _include('handy_moveit_config', 'servo.launch.py', {'limits': limits},
                 IfCondition(LaunchConfiguration('servo'))),
    ])
