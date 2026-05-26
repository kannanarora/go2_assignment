from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # Translates string commands into Unitree SDK requests
        Node(
            package='go2_behaviours',
            executable='sport_client_wrapper_node',
            name='sport_client_wrapper_node',
            output='screen',
        ),

        # Walks forward and stops when a wall is within 0.5 m
        Node(
            package='go2_behaviours',
            executable='wall_stop_test_node',
            name='wall_stop_test_node',
            output='screen',
        ),

    ])
