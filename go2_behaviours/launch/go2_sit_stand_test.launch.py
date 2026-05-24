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
            executable='sit_stand_loop_node',
            name='sit_stand_loop_node',
            output='screen',
        ),
    ])
