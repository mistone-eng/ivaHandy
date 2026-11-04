"""Bring up ros2_control for Handy, optionally with MoveIt (move_group + RViz) and MoveIt Servo.

  ros2 launch handy_control control.launch.py                      # real servos, test limits, MoveIt + RViz + Servo
  ros2 launch handy_control control.launch.py limits:=full         # full vetted joint range
  ros2 launch handy_control control.launch.py use_mock:=true       # fake hardware
  ros2 launch handy_control control.launch.py moveit:=false        # controllers only
  ros2 launch handy_control control.launch.py servo:=false         # MoveIt without Servo

limits affects MoveIt and Servo (joint_limits_test.yaml vs joint_limits.yaml).
Do not run this together with handy_moveit_config's demo launch: both start a controller manager.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, LaunchConfiguration
from launch_param_builder import ParameterBuilder
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

    if LaunchConfiguration('servo').perform(context).lower() == 'true':
        servo_params = {
            'moveit_servo': ParameterBuilder('handy_moveit_config')
            .yaml('config/servo.yaml')
            .to_dict()
        }
        servo_node = Node(
            package='moveit_servo',
            executable='servo_node_main',
            name='servo_node',
            parameters=[
                servo_params,
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
            ],
            output='screen',
        )
        # `ros2 service call` waits for the service, so this fires once Servo is ready.
        start_servo = ExecuteProcess(
            cmd=['ros2', 'service', 'call',
                 '/servo_node/start_servo', 'std_srvs/srv/Trigger', '{}'],
            output='screen',
        )
        actions += [
            servo_node,
            RegisterEventHandler(
                OnProcessStart(target_action=servo_node, on_start=[start_servo])
            ),
        ]
    return actions


def generate_launch_description():
    use_mock = LaunchConfiguration('use_mock')

    # Controllers come from handy_moveit_config so they match moveit_controllers.yaml
    # (arm_controller: joint_1-7, hand_controller: joint_8; joint_9 is a mimic and never commanded).
    moveit_share = get_package_share_directory('handy_moveit_config')
    controller_config = os.path.join(moveit_share, 'config', 'ros2_controllers.yaml')

    # the wrapper xacro in handy_moveit_config picks mock vs. dynamixel_handy_interface
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

    def spawner(name, *extra):
        return Node(package='controller_manager', executable='spawner',
                    arguments=[name, *extra], output='screen')

    # servo_controller and arm_controller claim the same joints, so only one is active.
    # Servo is the default; arm_controller is loaded inactive for MoveIt plans.
    spawners = [
        spawner('joint_state_broadcaster'),
        spawner('hand_controller'),
        spawner('servo_controller'),
        spawner('arm_controller', '--inactive'),
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_mock', default_value='false',
                              description='true = mock_components, false = Dynamixel hardware'),
        DeclareLaunchArgument('moveit', default_value='true',
                              description='start move_group (and RViz) alongside the controllers'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='start the MoveIt RViz panel (only if moveit:=true)'),
        DeclareLaunchArgument('servo', default_value='true',
                              description='start MoveIt Servo (only if moveit:=true)'),
        DeclareLaunchArgument('limits', default_value='test',
                              description="MoveIt joint limits: 'test' = small window, 'full' = vetted range"),
        robot_state_publisher,
        controller_manager,
        RegisterEventHandler(
            OnProcessStart(
                target_action=controller_manager,
                on_start=spawners,
            )
        ),
        OpaqueFunction(function=_moveit),
    ])