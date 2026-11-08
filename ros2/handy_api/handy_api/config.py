"""Robot configuration: every name, frame, topic, limit and scale the API uses, in one place.

Nothing else in handy_api hard-codes Handy. `load_handy_config()` builds Handy's configuration from
the installed handy_moveit_config files (servo.yaml scales, joint limits) and handy_api.poses;
another arm only needs its own RobotConfig. Pure Python (ament_index + yaml), no rclpy.
"""
import os
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

import yaml

from handy_api import poses

Limits = Dict[str, Tuple[float, float]]


@dataclass(frozen=True)
class Workspace:
    """Axis-aligned box (meters, in the base frame) the end effector may not leave under Servo."""

    min_xyz: Tuple[float, float, float]
    max_xyz: Tuple[float, float, float]


@dataclass(frozen=True)
class RobotConfig:
    # kinematics / naming
    arm_joints: Tuple[str, ...]
    hand_joint: str
    planning_group: str
    base_frame: str
    eef_frame: str
    joint_limits: Limits
    named_poses: Dict[str, Dict[str, float]]
    hand_positions: Dict[str, float]
    # MoveIt Servo (servo.yaml): unitless command 1.0 == these speeds
    servo_scale_linear: float      # m/s
    servo_scale_rotational: float  # rad/s
    servo_scale_joint: float       # rad/s
    servo_twist_topic: str = '/servo_node/delta_twist_cmds'
    servo_joint_topic: str = '/servo_node/delta_joint_cmds'
    servo_status_topic: str = '/servo_node/status'
    servo_start_service: str = '/servo_node/start_servo'
    # ros2_control / MoveIt
    joint_states_topic: str = '/joint_states'
    controller_manager: str = '/controller_manager'
    servo_controller: str = 'servo_controller'
    plan_controller: str = 'arm_controller'
    hand_action: str = '/hand_controller/follow_joint_trajectory'
    move_group_action: str = '/move_action'
    planning_pipeline: str = 'pilz_industrial_motion_planner'
    # safety (API-side; Servo and the controllers have their own limits too)
    workspace: Optional[Workspace] = None
    command_timeout: float = 0.5   # s: a velocity command not refreshed for this long is dropped
    status_names: Dict[int, str] = field(default_factory=lambda: dict(poses.SERVO_STATUS))
    # built-in patterns: which joints they use and the named pose they start from
    pattern_joints: Dict[str, Tuple[str, ...]] = field(default_factory=dict)  # snake/roll/nod
    pattern_start_pose: Optional[str] = None


def _share(package):
    from ament_index_python.packages import get_package_share_directory
    return get_package_share_directory(package)


def load_joint_limits(path) -> Limits:
    with open(path) as f:
        data = yaml.safe_load(f)['joint_limits']
    return {j: (float(v['min_position']), float(v['max_position']))
            for j, v in data.items() if v.get('has_position_limits', True)}


def load_servo_scales(path) -> Tuple[float, float, float]:
    with open(path) as f:
        scale = yaml.safe_load(f)['scale']
    return float(scale['linear']), float(scale['rotational']), float(scale['joint'])


def load_handy_config(limits: str = 'full', workspace: Optional[Workspace] = None) -> RobotConfig:
    """Handy's configuration. limits='full' or 'test' must match the running stack (`limits:=`)."""
    if limits not in ('full', 'test'):
        raise ValueError(f"limits must be 'full' or 'test', got {limits!r}")
    cfg_dir = os.path.join(_share('handy_moveit_config'), 'config')
    limits_file = 'joint_limits.yaml' if limits == 'full' else 'joint_limits_test.yaml'
    lin, rot, jnt = load_servo_scales(os.path.join(cfg_dir, 'servo.yaml'))
    return RobotConfig(
        arm_joints=tuple(poses.ARM_JOINTS),
        hand_joint=poses.HAND_JOINT,
        planning_group='arm',
        base_frame='base_link',
        eef_frame='link_eef',
        joint_limits=load_joint_limits(os.path.join(cfg_dir, limits_file)),
        named_poses={'ready': dict(poses.READY), 'wave': dict(poses.WAVE),
                     'home': dict(poses.HOME)},
        hand_positions=dict(poses.HAND),
        servo_scale_linear=lin,
        servo_scale_rotational=rot,
        servo_scale_joint=jnt,
        workspace=workspace,
        pattern_joints={'snake': ('joint_4', 'joint_5', 'joint_6', 'joint_7'),
                        'roll': ('joint_7',), 'nod': ('joint_6',)},
        pattern_start_pose='wave',
    )
