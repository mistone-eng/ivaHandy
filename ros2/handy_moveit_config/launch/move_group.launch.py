"""MoveIt move_group for Handy (expects handy_control/control.launch.py to be running)."""
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils.launches import generate_move_group_launch

from handy_moveit_config.launch_utils import build_moveit_config, declare_limits_arg


def _launch(context):
    config = build_moveit_config(LaunchConfiguration('limits').perform(context))
    return list(generate_move_group_launch(config).entities)


def generate_launch_description():
    return LaunchDescription([declare_limits_arg(), OpaqueFunction(function=_launch)])
