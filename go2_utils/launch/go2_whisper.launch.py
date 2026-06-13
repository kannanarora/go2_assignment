from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    audio_topic = LaunchConfiguration("audio_topic")
    text_topic = LaunchConfiguration("text_topic")
    model_name = LaunchConfiguration("model_name")
    enable_denoise = LaunchConfiguration("enable_denoise")
    min_rms = LaunchConfiguration("min_rms")
    chunk_seconds = LaunchConfiguration("chunk_seconds")
    fp16 = LaunchConfiguration("fp16")

    return LaunchDescription([
        DeclareLaunchArgument("audio_topic", default_value="/audiosender"),
        DeclareLaunchArgument("text_topic", default_value="/go2/whisper/text"),
        DeclareLaunchArgument("model_name", default_value="base.en"),
        DeclareLaunchArgument("enable_denoise", default_value="true"),
        DeclareLaunchArgument("min_rms", default_value="500"),
        DeclareLaunchArgument("chunk_seconds", default_value="1.0"),
        DeclareLaunchArgument("fp16", default_value="false"),
        Node(
            package="go2_utils",
            executable="go2_whisper_node",
            name="go2_whisper_node",
            output="screen",
            parameters=[{
                "audio_topic": audio_topic,
                "text_topic": text_topic,
                "model_name": model_name,
                "enable_denoise": enable_denoise,
                "min_rms": min_rms,
                "chunk_seconds": chunk_seconds,
                "fp16": fp16,
            }],
        ),
    ])
