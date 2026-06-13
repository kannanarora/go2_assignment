import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory("go2_behaviours"),
        "config",
        "behaviour_params.yaml",
    )
    cmd_vel_bridge_params_file = os.path.join(
        get_package_share_directory("go2_utils"),
        "config",
        "cmd_vel_bridge_params.yaml",
    )

    utils_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("go2_utils"), "launch", "go2_utils.launch.py"]
            )
        )
    )

    return LaunchDescription(
        [
            utils_launch,
            Node(
                package="go2_behaviours",
                executable="sport_client_wrapper_node",
                name="sport_client_wrapper_node",
                output="screen",
                parameters=[params_file],
            ),
            Node(
                package="go2_utils",
                executable="cmd_vel_bridge_node",
                name="cmd_vel_bridge_node",
                output="screen",
                parameters=[cmd_vel_bridge_params_file],
            ),
            Node(
                package="go2_behaviours",
                executable="wander_node",
                name="wander_node",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
