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
            executable='move_node',
            name='move_node',
            output='screen',
        ),
        Node(
            package='go2_behaviours',
            executable='wander_node',
            name='wander_node',
            output='screen',
        ),
    ])
