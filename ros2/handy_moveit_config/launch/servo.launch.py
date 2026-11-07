"""MoveIt Servo for Handy. Streams joint positions to /servo_controller/commands.

Expects handy_control/control.launch.py to be running with servo_controller active.
Servo is started automatically (start_servo service) once the node is up.
"""
from launch import LaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_param_builder import ParameterBuilder
from launch_ros.actions import Node

from handy_moveit_config.launch_utils import build_moveit_config, declare_limits_arg


def _launch(context):
    config = build_moveit_config(LaunchConfiguration('limits').perform(context))
    servo_params = {
        'moveit_servo': ParameterBuilder('handy_moveit_config').yaml('config/servo.yaml').to_dict()
    }
    servo_node = Node(
        package='moveit_servo',
        executable='servo_node_main',
        name='servo_node',
        parameters=[
            servo_params,
            config.robot_description,
            config.robot_description_semantic,
            config.robot_description_kinematics,
            config.joint_limits,
        ],
        output='screen',
    )
    # `ros2 service call` waits for the service, so this fires once Servo is ready.
    start_servo = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/servo_node/start_servo', 'std_srvs/srv/Trigger', '{}'],
        output='screen',
    )
    return [
        servo_node,
        RegisterEventHandler(OnProcessStart(target_action=servo_node, on_start=[start_servo])),
    ]


def generate_launch_description():
    return LaunchDescription([declare_limits_arg(), OpaqueFunction(function=_launch)])
