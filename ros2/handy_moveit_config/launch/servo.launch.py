from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_param_builder import ParameterBuilder
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "handy_description", package_name="handy_moveit_config"
    ).to_moveit_configs()

    servo_params = {
        "moveit_servo": ParameterBuilder("handy_moveit_config")
        .yaml("config/servo.yaml")
        .to_dict()
    }

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        output="screen",
    )

    start_servo = ExecuteProcess(
        cmd=["ros2", "service", "call",
             "/servo_node/start_servo", "std_srvs/srv/Trigger", "{}"],
        output="screen",
    )

    return LaunchDescription([
        servo_node,
        RegisterEventHandler(
            OnProcessStart(target_action=servo_node, on_start=[start_servo])
        ),
    ])