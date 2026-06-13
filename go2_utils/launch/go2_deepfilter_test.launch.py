from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    audio_topic = LaunchConfiguration("audio_topic")
    output_path = LaunchConfiguration("output_path")
    raw_output_path = LaunchConfiguration("raw_output_path")
    record_seconds = LaunchConfiguration("record_seconds")
    start_delay = LaunchConfiguration("start_delay")
    channel_mode = LaunchConfiguration("channel_mode")
    normalize = LaunchConfiguration("normalize")
    atten_lim_db = LaunchConfiguration("atten_lim_db")

    return LaunchDescription([
        DeclareLaunchArgument("audio_topic", default_value="/audiosender"),
        DeclareLaunchArgument(
            "output_path", default_value="/tmp/go2_deepfilter_clean.wav"
        ),
        DeclareLaunchArgument(
            "raw_output_path", default_value="/tmp/go2_deepfilter_raw.wav"
        ),
        DeclareLaunchArgument("record_seconds", default_value="10.0"),
        DeclareLaunchArgument("start_delay", default_value="3.0"),
        DeclareLaunchArgument("channel_mode", default_value="mono"),
        DeclareLaunchArgument("normalize", default_value="false"),
        DeclareLaunchArgument("atten_lim_db", default_value="0.0"),
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
                "start_delay": start_delay,
                "channel_mode": channel_mode,
                "normalize": normalize,
                "atten_lim_db": atten_lim_db,
            }],
        ),
    ])
