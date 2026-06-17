import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory("go2_behaviours"),
        "config",
        "dog_sounds_params.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package="go2_behaviours",
                executable="dog_sounds_node",
                name="dog_sounds_node",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
