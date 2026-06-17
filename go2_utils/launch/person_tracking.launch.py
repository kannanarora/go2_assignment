import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory("go2_utils")
    gstreamer_camera_params = os.path.join(
        share_dir,
        "config",
        "gstreamer_camera_params.yaml",
    )
    tracker_params = os.path.join(
        share_dir,
        "config",
        "person_tracker_params.yaml",
    )
    utils_launch = os.path.join(
        share_dir,
        "launch",
        "utils.launch.py",
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(utils_launch),
            ),
            Node(
                package="go2_utils",
                executable="gstreamer_camera_node",
                name="gstreamer_camera_node",
                output="screen",
                parameters=[gstreamer_camera_params],
            ),
            Node(
                package="go2_utils",
                executable="person_tracker_node",
                name="person_tracker_node",
                output="screen",
                parameters=[tracker_params],
            ),
        ]
    )
