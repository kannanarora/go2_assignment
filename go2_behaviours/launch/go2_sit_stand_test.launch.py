from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    utils_launch = os.path.join(
        get_package_share_directory("go2_utils"),
        "launch",
        "go2_utils.launch.py",
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(utils_launch),
        ),

        Node(
            package="go2_behaviours",
            executable="sport_client_wrapper_node",
            name="sport_client_wrapper_node",
            output="screen",
            parameters=[{
                "trigger_topic": "/trigger_behaviour",
                "request_topic": "/api/sport/request",
                "command_cooldown_s": 0.25,
            }],
        ),

        Node(
            package="go2_behaviours",
            executable="front_safety_sit_node",
            name="front_safety_sit_node",
            output="screen",
            parameters=[{
                "scan_topic": "/front_scan",
                "trigger_topic": "/trigger_behaviour",

                "sit_threshold_m": 0.8,
                "clear_threshold_m": 1.2,
                "front_half_angle_deg": 15.0,

                "required_blocked_frames": 2,
                "required_clear_frames": 4,

                "sit_command": "sit",
                "stand_command": "rise_sit",
                "enable_stand_command": True,

                # Keep this True until you have verified the logs.
                "dry_run": True,

                "log_rate_hz": 2.0,
                "use_sim_time": False,
            }],
        ),
    ])