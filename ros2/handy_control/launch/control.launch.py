import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('handy_control')
    controller_config = os.path.join(pkg_share, 'config', 'handy_controllers.yaml')

    desc_pkg_share = get_package_share_directory('handy_description')
    xacro_file = os.path.join(desc_pkg_share, 'urdf', 'handy_description.urdf.xacro')
    robot_description_content = Command(['xacro ', xacro_file])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen'
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_config],
        output='screen'
    )

    spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    trajectory_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller'],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        RegisterEventHandler(
            OnProcessStart(
                target_action=controller_manager,
                on_start=[
                    spawner,
                    trajectory_spawner,
                ]
            )
        )
    ])