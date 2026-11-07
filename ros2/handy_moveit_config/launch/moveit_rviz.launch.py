"""RViz with the MoveIt MotionPlanning panel for Handy."""
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils.launches import generate_moveit_rviz_launch

from handy_moveit_config.launch_utils import build_moveit_config, declare_limits_arg


def _launch(context):
    config = build_moveit_config(LaunchConfiguration('limits').perform(context))
    return list(generate_moveit_rviz_launch(config).entities)


def generate_launch_description():
    return LaunchDescription([declare_limits_arg(), OpaqueFunction(function=_launch)])
