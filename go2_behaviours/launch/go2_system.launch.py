import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('go2_behaviours'),
        'config',
        'behaviour_params.yaml',
    )
    cmd_vel_bridge_params_file = os.path.join(
        get_package_share_directory('go2_utils'),
        'config',
        'cmd_vel_bridge_params.yaml',
    )

    enable_voice = LaunchConfiguration('enable_voice')

    utils_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('go2_utils'), 'launch', 'go2_utils.launch.py']
            )
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_voice',
            default_value='true',
            description='Launch go2_whisper_node for voice commands',
        ),
        utils_launch,
        Node(
            package='go2_utils',
            executable='go2_whisper_node',
            name='go2_whisper_node',
            output='screen',
            condition=IfCondition(enable_voice),
            parameters=[{
                'audio_topic': '/audiosender',
                'text_topic': '/go2/whisper/text',
                'model_name': 'base.en',
                'min_rms': 500,
                'initial_prompt': 'sit stand hello stop walk bark',
            }],
        ),
        Node(
            package='go2_behaviours',
            executable='voice_command_mapper_node',
            name='voice_command_mapper_node',
            output='screen',
            condition=IfCondition(enable_voice),
            parameters=[params_file],
        ),
        Node(
            package='go2_behaviours',
            executable='sport_client_wrapper_node',
            name='sport_client_wrapper_node',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='go2_utils',
            executable='cmd_vel_bridge_node',
            name='cmd_vel_bridge_node',
            output='screen',
            parameters=[cmd_vel_bridge_params_file],
        ),
        Node(
            package='go2_behaviours',
            executable='wander_node',
            name='wander_node',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='go2_behaviours',
            executable='safety_monitor_node',
            name='safety_monitor_node',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='go2_behaviours',
            executable='behaviour_executor_node',
            name='behaviour_executor_node',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='go2_behaviours',
            executable='behaviour_planner_node',
            name='behaviour_planner_node',
            output='screen',
            parameters=[params_file],
        ),
    ])
