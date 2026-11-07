"""RViz with the MoveIt MotionPlanning panel for Handy.

The panel opens with the arm on Pilz PTP (see launch_utils.rviz_planner_defaults).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from handy_moveit_config.launch_utils import (
    build_moveit_config, declare_limits_arg, rviz_planner_defaults)


def _launch(context):
    config = build_moveit_config(LaunchConfiguration('limits').perform(context))
    rviz_config = os.path.join(
        get_package_share_directory('handy_moveit_config'), 'config', 'moveit.rviz')
    return [Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        arguments=['-d', rviz_config],
        parameters=[
            config.planning_pipelines,
            config.robot_description_kinematics,
            config.joint_limits,
            rviz_planner_defaults(),
        ],
    )]


def generate_launch_description():
    return LaunchDescription([declare_limits_arg(), OpaqueFunction(function=_launch)])
