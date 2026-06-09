from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory("go2_toolbox")
    urdf_path = os.path.join(pkg_share, "urdf", "go2_description.urdf")

    with open(urdf_path, "r") as f:
        robot_description = f.read()

    return LaunchDescription([

        Node(
            package="go2_toolbox",
            executable="odom_to_tf_bridge",
            name="odom_to_tf_bridge",
            output="screen",
            parameters=[{
                "input_odom_topic": "/utlidar/robot_odom",
                "output_odom_topic": "/odom",
                "odom_frame": "odom",
                "base_frame": "base_link",
                "publish_odom": True,
                "publish_tf": True,
            }],
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": False,
            }],
        ),

        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name="pointcloud_to_laserscan",
            output="screen",
            remappings=[
                ("cloud_in", "/utlidar/cloud"),
                ("scan", "/scan"),
            ],
            parameters=[{
                "target_frame": "base_link",
                "transform_tolerance": 0.3,
                "min_height": -0.10,
                "max_height": 0.35,
                "angle_min": -3.14159,
                "angle_max": 3.14159,
                "angle_increment": 0.0174533,
                "scan_time": 0.066,
                "range_min": 0.2,
                "range_max": 10.0,
                "use_inf": True,
            }],
        ),

        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[{
                "use_sim_time": False,
                "odom_frame": "odom",
                "map_frame": "map",
                "base_frame": "base_link",
                "scan_topic": "/scan",
                "mode": "mapping",
                "debug_logging": True,
                "max_laser_range": 10.0,
                "map_update_interval": 1.0,
                "minimum_travel_distance": 0.0,
                "minimum_travel_heading": 0.0,
                "transform_timeout": 0.5,
                "tf_buffer_duration": 30.0,
            }],
        ),
    ])