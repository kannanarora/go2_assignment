from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    audio_topic = LaunchConfiguration("audio_topic")
    text_topic = LaunchConfiguration("text_topic")
    model_name = LaunchConfiguration("model_name")
    backend = LaunchConfiguration("backend")
    vad_filter = LaunchConfiguration("vad_filter")
    enable_denoise = LaunchConfiguration("enable_denoise")
    atten_lim_db = LaunchConfiguration("atten_lim_db")
    output_gain = LaunchConfiguration("output_gain")
    min_rms = LaunchConfiguration("min_rms")
    chunk_seconds = LaunchConfiguration("chunk_seconds")
    fp16 = LaunchConfiguration("fp16")

    return LaunchDescription([
        DeclareLaunchArgument("audio_topic", default_value="/audiosender"),
        DeclareLaunchArgument("text_topic", default_value="/go2/whisper/text"),
        DeclareLaunchArgument("model_name", default_value="base.en"),
        DeclareLaunchArgument("backend", default_value="faster"),
        DeclareLaunchArgument("vad_filter", default_value="true"),
        DeclareLaunchArgument("enable_denoise", default_value="true"),
        DeclareLaunchArgument("atten_lim_db", default_value="12.0"),
        DeclareLaunchArgument("output_gain", default_value="1.0"),
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
                "backend": backend,
                "vad_filter": vad_filter,
                "enable_denoise": enable_denoise,
                "atten_lim_db": atten_lim_db,
                "output_gain": output_gain,
                "min_rms": min_rms,
                "chunk_seconds": chunk_seconds,
                "fp16": fp16,
            }],
        ),
    ])
