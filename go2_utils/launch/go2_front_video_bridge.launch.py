import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory("go2_utils"),
        "config",
        "front_video_bridge_params.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package="go2_utils",
                executable="front_video_bridge_node",
                name="front_video_bridge_node",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
