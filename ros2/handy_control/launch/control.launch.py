"""Bring up ros2_control for Handy, optionally with MoveIt (move_group + RViz).

  ros2 launch handy_control control.launch.py                      # real servos, test limits, MoveIt + RViz
  ros2 launch handy_control control.launch.py limits:=full         # full vetted joint range
  ros2 launch handy_control control.launch.py use_mock:=true       # fake hardware
  ros2 launch handy_control control.launch.py moveit:=false        # controllers only

limits only affects MoveIt (joint_limits_test.yaml vs joint_limits.yaml).
Do not run this together with handy_moveit_config's demo launch: both start a controller manager.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch, generate_moveit_rviz_launch


def _moveit(context, *args, **kwargs):
    if LaunchConfiguration('moveit').perform(context).lower() != 'true':
        return []

    limits = LaunchConfiguration('limits').perform(context)
    if limits not in ('test', 'full'):
        raise RuntimeError("limits must be 'test' or 'full', got '%s'" % limits)
    use_mock = LaunchConfiguration('use_mock').perform(context)

    config_dir = os.path.join(get_package_share_directory('handy_moveit_config'), 'config')
    limits_file = 'joint_limits_test.yaml' if limits == 'test' else 'joint_limits.yaml'

    moveit_config = (
        MoveItConfigsBuilder('handy_description', package_name='handy_moveit_config')
        .robot_description(mappings={'use_mock': use_mock})
        .joint_limits(file_path=os.path.join(config_dir, limits_file))
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"],
                            default_planning_pipeline="pilz_industrial_motion_planner")
        .pilz_cartesian_limits(file_path="config/pilz_cartesian_limits.yaml")
        .to_moveit_configs()
    )
    actions = list(generate_move_group_launch(moveit_config).entities)
    if LaunchConfiguration('rviz').perform(context).lower() == 'true':
        actions += list(generate_moveit_rviz_launch(moveit_config).entities)
    return actions


def generate_launch_description():
    use_mock = LaunchConfiguration('use_mock')

    pkg_share = get_package_share_directory('handy_control')
    controller_config = os.path.join(pkg_share, 'config', 'handy_controllers.yaml')

    # the wrapper xacro in handy_moveit_config picks mock vs. dynamixel_handy_interface
    moveit_share = get_package_share_directory('handy_moveit_config')
    xacro_file = os.path.join(moveit_share, 'config', 'handy_description.urdf.xacro')
    robot_description_content = Command(['xacro ', xacro_file, ' use_mock:=', use_mock])
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen',
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_config],
        output='screen',
    )

    spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    trajectory_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller'],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_mock', default_value='false',
                              description='true = mock_components, false = Dynamixel hardware'),
        DeclareLaunchArgument('moveit', default_value='true',
                              description='start move_group (and RViz) alongside the controllers'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='start the MoveIt RViz panel (only if moveit:=true)'),
        DeclareLaunchArgument('limits', default_value='test',
                              description="MoveIt joint limits: 'test' = small window, 'full' = vetted range"),
        robot_state_publisher,
        controller_manager,
        RegisterEventHandler(
            OnProcessStart(
                target_action=controller_manager,
                on_start=[spawner, trajectory_spawner],
            )
        ),
        OpaqueFunction(function=_moveit),
    ])