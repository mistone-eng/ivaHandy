"""Shared MoveIt configuration for every Handy launch file.

Import from a launch file with:
    from handy_moveit_config.launch_utils import build_moveit_config, declare_limits_arg
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from moveit_configs_utils import MoveItConfigsBuilder

LIMIT_FILES = {
    'test': 'joint_limits_test.yaml',  # +/-0.3 rad window -- default for first runs
    'full': 'joint_limits.yaml',       # vetted full range
}


# Pilz is the default planning pipeline. Humble's RViz MotionPlanning panel cannot be
# preconfigured to select a planner (PTP) -- pick it in the Context tab; see the README.
# The hand (joint_8/9) never uses a motion planner: it is commanded through hand_controller.
DEFAULT_PIPELINE = 'pilz_industrial_motion_planner'


def declare_limits_arg():
    return DeclareLaunchArgument(
        'limits', default_value='test',
        description="MoveIt/Servo joint limits: 'test' = small window, 'full' = vetted range")


def build_moveit_config(limits='test'):
    """Return the MoveItConfigs for Handy with the chosen joint-limit set."""
    if limits not in LIMIT_FILES:
        raise RuntimeError(f"limits must be one of {sorted(LIMIT_FILES)}, got '{limits}'")
    config_dir = os.path.join(get_package_share_directory('handy_moveit_config'), 'config')
    return (
        MoveItConfigsBuilder('handy_description', package_name='handy_moveit_config')
        .joint_limits(file_path=os.path.join(config_dir, LIMIT_FILES[limits]))
        .planning_pipelines(pipelines=['ompl', 'pilz_industrial_motion_planner'],
                            default_planning_pipeline=DEFAULT_PIPELINE)
        .pilz_cartesian_limits(file_path='config/pilz_cartesian_limits.yaml')
        .to_moveit_configs()
    )
