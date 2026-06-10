from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # wav_file_arg = DeclareLaunchArgument(
    #     'wav_file',
    #     default_value='/home/unitree/bark.wav',
    #     description='Absolute path to the 48 kHz mono WAV file to play',
    # )

    audio_player_node = Node(
        package='go2_behaviours',
        executable='audio_player_node',
        name='audio_player_node',
        output='screen',
        parameters=[{
            'wav_file': LaunchConfiguration('wav_file'),
        }],
    )

    return LaunchDescription([
        # wav_file_arg,
        audio_player_node,
    ])
