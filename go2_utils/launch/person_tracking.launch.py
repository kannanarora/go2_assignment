import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory("go2_utils")
    camera_source = LaunchConfiguration("camera_source")
    video_client_camera_params = os.path.join(
        share_dir,
        "config",
        "video_client_camera_params.yaml",
    )
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
            DeclareLaunchArgument(
                "camera_source",
                default_value="video_client",
                description="Camera source: video_client or gstreamer",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(utils_launch),
            ),
            Node(
                package="go2_utils",
                executable="video_client_camera_node",
                name="video_client_camera_node",
                output="screen",
                parameters=[video_client_camera_params],
                condition=IfCondition(
                    PythonExpression(["'", camera_source, "' == 'video_client'"])
                ),
            ),
            Node(
                package="go2_utils",
                executable="gstreamer_camera_node",
                name="gstreamer_camera_node",
                output="screen",
                parameters=[gstreamer_camera_params],
                condition=IfCondition(
                    PythonExpression(["'", camera_source, "' == 'gstreamer'"])
                ),
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
