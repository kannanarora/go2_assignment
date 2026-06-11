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
        description='Absolute path to the WAV file to upload and play',
    )

    file_name_arg = DeclareLaunchArgument(
        'file_name',
        default_value='go2_bark',
        description='Name to register the file under in AudioHub',
    )

    audiohub_player_node = Node(
        package='go2_behaviours',
        executable='audiohub_player_node',
        name='audiohub_player_node',
        output='screen',
        parameters=[{
            'wav_file':  LaunchConfiguration('wav_file'),
            'file_name': LaunchConfiguration('file_name'),
        }],
    )

    return LaunchDescription([
        wav_file_arg,
        file_name_arg,
        audiohub_player_node,
    ])
