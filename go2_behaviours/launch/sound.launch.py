"""
Sound subsystem: generic trick sounds + optional ambient dog sounds.

  sound_player_node   plays one-shot trick clips from /trigger_behaviour
  dog_sounds_node     (optional) ambient panting from /cmd_vel + events
  random_bark_node    (optional) publishes "bark" at random intervals

Clips must already be in AudioHub - provision them first with:
  ros2 launch go2_utils audiohub_player.launch.py \
      wav_file:=<abs path>/bark2.wav file_name:=bark2

Edit SOUND_MAP below to add more token -> file_name pairs.
"""

import os

from ament_index_python.packages import get_package_share_directory
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
    enable_dog_sounds = LaunchConfiguration("enable_dog_sounds")
    dog_sounds_params = os.path.join(
        get_package_share_directory("go2_behaviours"),
        "config",
        "dog_sounds_params.yaml",
    )

    return LaunchDescription([
        DeclareLaunchArgument("trigger_topic", default_value="/trigger_behaviour"),
        DeclareLaunchArgument(
            "enable_dog_sounds",
            default_value="true",
            description="Run ambient/event dog_sounds_node observer",
        ),
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
            executable="dog_sounds_node",
            name="dog_sounds_node",
            output="screen",
            condition=IfCondition(enable_dog_sounds),
            parameters=[dog_sounds_params],
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
