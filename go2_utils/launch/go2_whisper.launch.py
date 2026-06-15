from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    audio_topic = LaunchConfiguration("audio_topic")
    text_topic = LaunchConfiguration("text_topic")
    model_name = LaunchConfiguration("model_name")
    backend = LaunchConfiguration("backend")
    beam_size = LaunchConfiguration("beam_size")
    enable_denoise = LaunchConfiguration("enable_denoise")
    atten_lim_db = LaunchConfiguration("atten_lim_db")
    use_silero = LaunchConfiguration("use_silero")
    vad_threshold = LaunchConfiguration("vad_threshold")
    min_rms = LaunchConfiguration("min_rms")
    endpoint_silence = LaunchConfiguration("endpoint_silence")
    vad_filter = LaunchConfiguration("vad_filter")
    device = LaunchConfiguration("device")

    return LaunchDescription([
        DeclareLaunchArgument("audio_topic", default_value="/audiosender"),
        DeclareLaunchArgument("text_topic", default_value="/go2/whisper/text"),
        DeclareLaunchArgument("model_name", default_value="base.en"),
        DeclareLaunchArgument("backend", default_value="faster"),
        DeclareLaunchArgument("beam_size", default_value="2"),
        DeclareLaunchArgument("enable_denoise", default_value="true"),
        DeclareLaunchArgument("atten_lim_db", default_value="12.0"),
        DeclareLaunchArgument("use_silero", default_value="true"),
        DeclareLaunchArgument("vad_threshold", default_value="0.3"),
        DeclareLaunchArgument("min_rms", default_value="800"),
        DeclareLaunchArgument("endpoint_silence", default_value="0.6"),
        DeclareLaunchArgument("vad_filter", default_value="false"),
        DeclareLaunchArgument("device", default_value=""),
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
                "beam_size": beam_size,
                "enable_denoise": enable_denoise,
                "atten_lim_db": atten_lim_db,
                "use_silero": use_silero,
                "vad_threshold": vad_threshold,
                "min_rms": min_rms,
                "endpoint_silence": endpoint_silence,
                "vad_filter": vad_filter,
                "device": device,
            }],
        ),
    ])
