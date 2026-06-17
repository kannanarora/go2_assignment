import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    behaviour_params = os.path.join(
        get_package_share_directory("go2_behaviours"),
        "config",
        "behaviour_params.yaml",
    )
    person_tracking_launch = os.path.join(
        get_package_share_directory("go2_utils"),
        "launch",
        "person_tracking.launch.py",
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(person_tracking_launch),
            ),
            Node(
                package="go2_behaviours",
                executable="sport_client_wrapper_node",
                name="sport_client_wrapper_node",
                output="screen",
                parameters=[behaviour_params],
            ),
            Node(
                package="go2_behaviours",
                executable="mux_node",
                name="mux_node",
                output="screen",
            ),
            Node(
                package="go2_behaviours",
                executable="avoid_people_node",
                name="avoid_people_node",
                output="screen",
                parameters=[behaviour_params],
            ),
        ]
    )
