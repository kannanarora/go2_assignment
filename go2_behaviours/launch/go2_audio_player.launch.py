from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    wav_file_arg = DeclareLaunchArgument(
        'wav_file',
        default_value=PathJoinSubstitution(
            [FindPackageShare('go2_behaviours'), 'sounds', 'bark.wav']
        ),
        description='Absolute path to the 48 kHz mono WAV file to play',
    )

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
        wav_file_arg,
        audio_player_node,
    ])
