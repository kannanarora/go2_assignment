import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory("go2_utils"),
        "config",
        "webrtc_camera_params.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "data_channel_id",
                default_value="-1",
                description="WebRTC data channel id. Use -1 for aiortc auto id.",
            ),
            DeclareLaunchArgument(
                "enable_audio_transceiver",
                default_value="true",
                description="Whether to include a sendrecv audio transceiver.",
            ),
            Node(
                package="go2_utils",
                executable="webrtc_camera_node",
                name="webrtc_camera_node",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "data_channel_id": LaunchConfiguration("data_channel_id"),
                        "enable_audio_transceiver": LaunchConfiguration(
                            "enable_audio_transceiver"
                        ),
                    },
                ],
            ),
        ]
    )
