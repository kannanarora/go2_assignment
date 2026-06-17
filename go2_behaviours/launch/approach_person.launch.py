import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    behaviour_params = os.path.join(
        get_package_share_directory("go2_behaviours"),
        "config",
        "behaviour_params.yaml",
    )
    cmd_vel_bridge_params = os.path.join(
        get_package_share_directory("go2_utils"),
        "config",
        "cmd_vel_bridge_params.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package="go2_behaviours",
                executable="sport_client_wrapper_node",
                name="sport_client_wrapper_node",
                output="screen",
                parameters=[behaviour_params],
            ),
            Node(
                package="go2_utils",
                executable="cmd_vel_bridge_node",
                name="cmd_vel_bridge_node",
                output="screen",
                parameters=[cmd_vel_bridge_params],
            ),
            Node(
                package="go2_behaviours",
                executable="approach_person_node",
                name="approach_person_node",
                output="screen",
                parameters=[behaviour_params],
            ),
        ]
    )
