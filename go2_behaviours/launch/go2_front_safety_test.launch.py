from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='go2_behaviours',
            executable='sport_client_wrapper_node',
            name='sport_client_wrapper_node',
            output='screen',
        ),
        Node(
            package='go2_behaviours',
            executable='front_safety_sit_node',
            name='front_safety_sit_node',
            output='screen',
        ),
    ])
