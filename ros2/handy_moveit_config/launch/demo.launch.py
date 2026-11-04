import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("handy_description", package_name="handy_moveit_config").to_moveit_configs()
    ld = generate_demo_launch(moveit_config)

    # MoveIt Servo is part of the default bringup and starts automatically.
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("handy_moveit_config"),
                    "launch",
                    "servo.launch.py",
                )
            )
        )
    )
    return ld