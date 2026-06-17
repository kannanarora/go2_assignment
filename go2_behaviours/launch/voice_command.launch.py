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
    enable_whisper = LaunchConfiguration("enable_whisper")
    enable_random_bark = LaunchConfiguration("enable_random_bark")

    whisper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("go2_utils"), "launch", "go2_whisper.launch.py"]
            )
        ),
        condition=IfCondition(enable_whisper),
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
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("enable_whisper", default_value="true"),
        DeclareLaunchArgument("enable_random_bark", default_value="true"),
        whisper_launch,
        sound_launch,
        Node(
            package="go2_behaviours",
            executable="voice_command_mapper_node",
            name="voice_command_mapper_node",
            output="screen",
            parameters=[params_file],
        ),
    ])
