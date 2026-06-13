from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    audio_topic = LaunchConfiguration("audio_topic")
    output_path = LaunchConfiguration("output_path")
    raw_output_path = LaunchConfiguration("raw_output_path")
    record_seconds = LaunchConfiguration("record_seconds")

    return LaunchDescription([
        DeclareLaunchArgument("audio_topic", default_value="/audiosender"),
        DeclareLaunchArgument(
            "output_path", default_value="/tmp/go2_deepfilter_clean.wav"
        ),
        DeclareLaunchArgument(
            "raw_output_path", default_value="/tmp/go2_deepfilter_raw.wav"
        ),
        DeclareLaunchArgument("record_seconds", default_value="6.0"),
        Node(
            package="go2_utils",
            executable="go2_deepfilter_test_node",
            name="go2_deepfilter_test_node",
            output="screen",
            parameters=[{
                "audio_topic": audio_topic,
                "output_path": output_path,
                "raw_output_path": raw_output_path,
                "record_seconds": record_seconds,
            }],
        ),
    ])
