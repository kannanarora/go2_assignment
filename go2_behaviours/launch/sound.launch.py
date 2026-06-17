"""
Sound subsystem: generic AudioHub player + optional autonomous barking.

  sound_player_node   plays a clip when its token arrives on trigger_topic
  random_bark_node    (optional) publishes "bark" at random intervals

Clips must already be in AudioHub - provision them first with:
  ros2 launch go2_utils audiohub_player.launch.py \
      wav_file:=<abs path>/bark2.wav file_name:=bark2

Edit SOUND_MAP below to add more token -> file_name pairs.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# token (spoken/published) -> file_name registered in AudioHub
SOUND_MAP = [
    "bark:bark",
    "speak:bark",
    "dance:bark2",
    "sit:bark",
    "lie_down:crying1",
    "hello:bark",
    "stretch:stretch_1",
]


def generate_launch_description():
    trigger_topic = LaunchConfiguration("trigger_topic")
    enable_random_bark = LaunchConfiguration("enable_random_bark")

    return LaunchDescription([
        DeclareLaunchArgument("trigger_topic", default_value="/trigger_behaviour"),
        DeclareLaunchArgument(
            "enable_random_bark",
            default_value="false",
            description="Also run random_bark_node for autonomous barking",
        ),
        Node(
            package="go2_behaviours",
            executable="sound_player_node",
            name="sound_player_node",
            output="screen",
            parameters=[{
                "trigger_topic": trigger_topic,
                "sound_map": SOUND_MAP,
            }],
        ),
        Node(
            package="go2_behaviours",
            executable="random_bark_node",
            name="random_bark_node",
            output="screen",
            condition=IfCondition(enable_random_bark),
            parameters=[{
                "trigger_topic": trigger_topic,
            }],
        ),
    ])
