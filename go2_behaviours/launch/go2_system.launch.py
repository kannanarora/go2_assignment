import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
	params_file = os.path.join(
		get_package_share_directory('go2_behaviours'),
		'config',
		'behaviour_params.yaml'
	)

	return LaunchDescription([
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
