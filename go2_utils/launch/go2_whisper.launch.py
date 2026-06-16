from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    audio_topic = LaunchConfiguration("audio_topic")
    text_topic = LaunchConfiguration("text_topic")
    model_name = LaunchConfiguration("model_name")
    beam_size = LaunchConfiguration("beam_size")
    enable_denoise = LaunchConfiguration("enable_denoise")
    atten_lim_db = LaunchConfiguration("atten_lim_db")
    min_rms = LaunchConfiguration("min_rms")
    endpoint_silence = LaunchConfiguration("endpoint_silence")
    device = LaunchConfiguration("device")

    return LaunchDescription([
        DeclareLaunchArgument("audio_topic", default_value="/audiosender"),
        DeclareLaunchArgument("text_topic", default_value="/go2/whisper/text"),
        DeclareLaunchArgument("model_name", default_value="base.en"),
        DeclareLaunchArgument("beam_size", default_value="2"),
        DeclareLaunchArgument("enable_denoise", default_value="true"),
        DeclareLaunchArgument("atten_lim_db", default_value="12.0"),
        DeclareLaunchArgument("min_rms", default_value="800"),
        DeclareLaunchArgument("endpoint_silence", default_value="0.6"),
        DeclareLaunchArgument("device", default_value=""),
        Node(
            package="go2_utils",
            executable="whisper_node",
            name="go2_whisper_node",
            output="screen",
            parameters=[{
                "audio_topic": audio_topic,
                "text_topic": text_topic,
                "model_name": model_name,
                "beam_size": beam_size,
                "enable_denoise": enable_denoise,
                "atten_lim_db": atten_lim_db,
                "min_rms": min_rms,
                "endpoint_silence": endpoint_silence,
                "device": device,
            }],
        ),
    ])
