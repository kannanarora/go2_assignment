"""
Sound subsystem: independent dog sounds.

  dog_sounds_node   owns AudioHub playback for behaviour + sound-only events
  random_bark_node  publishes "bark" at random intervals on /dog_sound_trigger

Clips must already be in AudioHub - provision them first with:
  ros2 launch go2_utils audiohub_player.launch.py \
      wav_file:=<abs path>/bark2.wav file_name:=bark2
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    trigger_topic = LaunchConfiguration("trigger_topic")
    sound_trigger_topic = LaunchConfiguration("sound_trigger_topic")
    enable_random_bark = LaunchConfiguration("enable_random_bark")
    enable_dog_sounds = LaunchConfiguration("enable_dog_sounds")
    dog_sounds_params = os.path.join(
        get_package_share_directory("go2_behaviours"),
        "config",
        "dog_sounds_params.yaml",
    )

    return LaunchDescription([
        DeclareLaunchArgument("trigger_topic", default_value="/trigger_behaviour"),
        DeclareLaunchArgument("sound_trigger_topic", default_value="/dog_sound_trigger"),
        DeclareLaunchArgument(
            "enable_dog_sounds",
            default_value="true",
            description="Run ambient/event dog_sounds_node observer",
        ),
        DeclareLaunchArgument(
            "enable_random_bark",
            default_value="true",
            description="Run random_bark_node on the sound-only trigger topic",
        ),
        Node(
            package="go2_behaviours",
            executable="dog_sounds_node",
            name="dog_sounds_node",
            output="screen",
            condition=IfCondition(enable_dog_sounds),
            parameters=[
                dog_sounds_params,
                {
                    "behaviour_trigger_topic": trigger_topic,
                    "sound_trigger_topic": sound_trigger_topic,
                },
            ],
        ),
        Node(
            package="go2_behaviours",
            executable="random_bark_node",
            name="random_bark_node",
            output="screen",
            condition=IfCondition(enable_random_bark),
            parameters=[{
                "trigger_topic": sound_trigger_topic,
                "min_interval_s": 8.0,
                "max_interval_s": 18.0,
            }],
        ),
    ])
