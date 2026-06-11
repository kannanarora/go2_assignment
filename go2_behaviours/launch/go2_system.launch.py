import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
	params_file = os.path.join(
		get_package_share_directory('go2_behaviours'),
		'config',
		'behaviour_params.yaml'
	)

	wav_file_arg = DeclareLaunchArgument(
		'wav_file',
		default_value=PathJoinSubstitution(
			[FindPackageShare('go2_behaviours'), 'sounds', 'bark.wav']
		),
		description='Absolute path to the 48 kHz mono WAV file to play',
	)

	return LaunchDescription([
		wav_file_arg,
		Node(
			package='go2_behaviours',
			executable='sport_client_wrapper_node',
			name='sport_client_wrapper_node',
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
		Node(
			package='go2_behaviours',
			executable='audio_player_node',
			name='audio_player_node',
			output='screen',
			parameters=[{
				'wav_file': LaunchConfiguration('wav_file'),
			}],
		),
	])
