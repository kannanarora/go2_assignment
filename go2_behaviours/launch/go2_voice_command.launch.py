from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    text_topic = LaunchConfiguration("text_topic")
    trigger_topic = LaunchConfiguration("trigger_topic")
    cooldown_sec = LaunchConfiguration("cooldown_sec")

    return LaunchDescription([
        DeclareLaunchArgument("text_topic", default_value="/go2/whisper/text"),
        DeclareLaunchArgument("trigger_topic", default_value="/trigger_behaviour"),
        DeclareLaunchArgument("cooldown_sec", default_value="2.0"),
        Node(
            package="go2_behaviours",
            executable="voice_command_mapper_node",
            name="voice_command_mapper_node",
            output="screen",
            parameters=[{
                "text_topic": text_topic,
                "trigger_topic": trigger_topic,
                "cooldown_sec": cooldown_sec,
            }],
        ),
        Node(
            package="go2_behaviours",
            executable="sport_client_wrapper_node",
            name="sport_client_wrapper_node",
            output="screen",
        ),
    ])
