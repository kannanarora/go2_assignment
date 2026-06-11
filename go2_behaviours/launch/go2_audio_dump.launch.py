from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('channel_mode',    default_value='mono'),
        DeclareLaunchArgument('noise_reduce',    default_value='false'),
        DeclareLaunchArgument('record_seconds',  default_value='10.0'),
        DeclareLaunchArgument('output_path',     default_value='/tmp/go2_audio.wav'),
        DeclareLaunchArgument('audio_topic',     default_value='/audiosender'),

        Node(
            package='go2_behaviours',
            executable='audio_dump_node',
            name='audio_dump_node',
            output='screen',
            parameters=[{
                'channel_mode':   LaunchConfiguration('channel_mode'),
                'noise_reduce':   LaunchConfiguration('noise_reduce'),
                'record_seconds': LaunchConfiguration('record_seconds'),
                'output_path':    LaunchConfiguration('output_path'),
                'audio_topic':    LaunchConfiguration('audio_topic'),
            }],
        ),
    ])
