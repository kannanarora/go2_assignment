import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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

    enable_voice = LaunchConfiguration("enable_voice")
    enable_random_bark = LaunchConfiguration("enable_random_bark")
    enable_dog_sounds = LaunchConfiguration("enable_dog_sounds")

    utils_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("go2_utils"), "launch", "utils.launch.py"]
            )
        )
    )

    whisper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("go2_utils"), "launch", "go2_whisper.launch.py"]
            )
        ),
        condition=IfCondition(enable_voice),
    )

    sound_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("go2_behaviours"), "launch", "sound.launch.py"]
            )
        ),
        launch_arguments={
            "trigger_topic": "/trigger_behaviour",
            "enable_random_bark": enable_random_bark,
            "enable_dog_sounds": enable_dog_sounds,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("enable_voice", default_value="true"),
            DeclareLaunchArgument(
                "enable_random_bark",
                default_value="false",
                description="Publish random bark tokens on /trigger_behaviour",
            ),
            DeclareLaunchArgument(
                "enable_dog_sounds",
                default_value="true",
                description="Run ambient/event dog sounds observer node",
            ),
            utils_launch,
            whisper_launch,
            sound_launch,
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
                executable="mux_node",
                name="mux_node",
                output="screen",
            ),
            Node(
                package="go2_behaviours",
                executable="wander_node",
                name="wander_node",
                output="screen",
                parameters=[params_file],
            ),
            Node(
                package="go2_behaviours",
                executable="obstacle_avoid_node",
                name="obstacle_avoid_node",
                output="screen",
                parameters=[params_file],
            ),
            Node(
                package="go2_behaviours",
                executable="voice_command_mapper_node",
                name="voice_command_mapper_node",
                output="screen",
                parameters=[params_file],
                condition=IfCondition(enable_voice),
            ),
        ]
    )
