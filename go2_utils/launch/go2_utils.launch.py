from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory("go2_utils")
    urdf_path = os.path.join(pkg_share, "urdf", "go2_description.urdf")

    with open(urdf_path, "r") as f:
        robot_description = f.read()

    return LaunchDescription([
        Node(
            package="go2_utils",
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
                ("cloud_in", "/utlidar/cloud_base"),
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
                "range_min": 0.25,
                "range_max": 8.0,
                "use_inf": True,
            }],
        ),
        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name="front_pointcloud_to_laserscan",
            output="screen",
            remappings=[
                ("cloud_in", "/utlidar/cloud_deskewed"),
                ("scan", "/front_scan"),
            ],
            parameters=[{
                "target_frame": "base_footprint",
                "transform_tolerance": 0.5,

                "angle_min": -1.5708,
                "angle_max": 1.5708,
                "angle_increment": 0.0174533,

                # Ignore floor and very low noise.
                "min_height": 0.08,
                "max_height": 1.20,

                # Ignore robot body / legs / near-field junk.
                "range_min": 0.60,
                "range_max": 6.0,

                "scan_time": 0.10,
                "use_inf": True,
            }],
        ),
        # #Node(
        # #    package="slam_utils",
        # #    executable="async_slam_utils_node",
        # #    name="slam_utils",
        # #    output="screen",
        # #    parameters=[{
        # #        "use_sim_time": False,
        # #        "odom_frame": "odom",
        # #        "map_frame": "map",
        # #        "base_frame": "base_link",
        # #        "scan_topic": "/scan",
        # #        "mode": "mapping",

        #         "debug_logging": False,
        #         "max_laser_range": 8.0,
        #         "map_update_interval": 2.0,

        #         "minimum_time_interval": 0.2,
        #         "minimum_travel_distance": 0.20,
        #         "minimum_travel_heading": 0.20,

        #         "transform_timeout": 0.5,
        #         "tf_buffer_duration": 30.0,

        #         "use_scan_matching": True,
        #         "use_scan_barycenter": True,

        #         "scan_buffer_size": 10,
        #         "scan_buffer_maximum_scan_distance": 8.0,

        #         "do_loop_closing": False,
        #         "resolution": 0.05,
        #     }],
        # ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": False,
            }],
        ),
        Node(
            package="go2_utils",
            executable="cloud_throttle",
            name="cloud_throttle",
            output="screen",
            parameters=[{
                "input_topic": "/utlidar/cloud_deskewed",
                "output_topic": "/utlidar/cloud_deskewed_viz",
                "publish_rate": 2.0,
            }],
        ),
    ])
