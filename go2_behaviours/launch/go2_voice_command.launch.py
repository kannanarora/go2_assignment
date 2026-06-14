from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    text_topic = LaunchConfiguration("text_topic")
    trigger_topic = LaunchConfiguration("trigger_topic")
    cooldown_sec = LaunchConfiguration("cooldown_sec")
    enable_wake_word = LaunchConfiguration("enable_wake_word")
    wake_window_sec = LaunchConfiguration("wake_window_sec")

    return LaunchDescription([
        DeclareLaunchArgument("text_topic", default_value="/go2/whisper/text"),
        DeclareLaunchArgument("trigger_topic", default_value="/trigger_behaviour"),
        DeclareLaunchArgument("cooldown_sec", default_value="2.0"),
        DeclareLaunchArgument("enable_wake_word", default_value="true"),
        DeclareLaunchArgument("wake_window_sec", default_value="8.0"),
        Node(
            package="go2_behaviours",
            executable="voice_command_mapper_node",
            name="voice_command_mapper_node",
            output="screen",
            parameters=[{
                "text_topic": text_topic,
                "trigger_topic": trigger_topic,
                "cooldown_sec": cooldown_sec,
                "enable_wake_word": enable_wake_word,
                "wake_window_sec": wake_window_sec,
            }],
        ),
        Node(
            package="go2_behaviours",
            executable="sport_client_wrapper_node",
            name="sport_client_wrapper_node",
            output="screen",
        ),
    ])
