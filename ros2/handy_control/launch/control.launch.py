"""Hardware layer for Handy: robot_state_publisher + ros2_control + controllers.

Nothing MoveIt-related lives here; compose the full stack with handy_bringup/demo.launch.py.

  ros2 launch handy_control control.launch.py                  # mock hardware (default)
  ros2 launch handy_control control.launch.py use_mock:=false  # REAL Dynamixels

Controllers (config/ros2_controllers.yaml):
  joint_state_broadcaster  active
  hand_controller          active   (joint_8; joint_9 mirrors it)
  servo_controller         active   (joint_1-7 position stream from MoveIt Servo)
  arm_controller           inactive (joint_1-7 trajectories from MoveIt plans)
servo_controller and arm_controller claim the same joints, so only one can be active.
Switch with `handy mode plan|servo`.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_mock = LaunchConfiguration('use_mock')

    xacro_file = os.path.join(
        get_package_share_directory('handy_description'), 'urdf', 'handy.urdf.xacro')
    controller_config = os.path.join(
        get_package_share_directory('handy_control'), 'config', 'ros2_controllers.yaml')

    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_file, ' use_mock:=', use_mock]), value_type=str)
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

    spawners = [
        spawner('joint_state_broadcaster'),
        spawner('hand_controller'),
        spawner('servo_controller'),
        spawner('arm_controller', '--inactive'),
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_mock', default_value='true',
                              description='true = mock_components, false = real Dynamixel hardware'),
        robot_state_publisher,
        controller_manager,
        RegisterEventHandler(OnProcessStart(target_action=controller_manager, on_start=spawners)),
    ])
