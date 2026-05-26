"""
Converts a PointCloud2 topic into a LaserScan in a forward-facing sector.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2


class PointCloudToLaserScanNode(Node):
    def __init__(self):
        super().__init__('pointcloud_to_laserscan_node')

        self.cloud_topic = self.declare_parameter(
            'cloud_topic', '/utlidar/cloud_deskewed'
        ).value
        self.angle_min_deg = float(self.declare_parameter('angle_min_deg', -105.0).value)
        self.angle_max_deg = float(self.declare_parameter('angle_max_deg', 105.0).value)
        self.angle_increment_deg = float(
            self.declare_parameter('angle_increment_deg', 1.0).value
        )
        self.range_min = float(self.declare_parameter('range_min', 0.05).value)
        self.range_max = float(self.declare_parameter('range_max', 30.0).value)
        self.output_frame = self.declare_parameter('output_frame', '').value

        self.scan_pub = self.create_publisher(LaserScan, 'front_scan', 10)
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            self.cloud_topic,
            self.cloud_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            'PointCloudToLaserScanNode listening on %s (%.1f..%.1f deg).'
            % (self.cloud_topic, self.angle_min_deg, self.angle_max_deg)
        )

    def cloud_callback(self, msg: PointCloud2):
        angle_min = math.radians(self.angle_min_deg)
        angle_max = math.radians(self.angle_max_deg)
        angle_inc = math.radians(self.angle_increment_deg)

        if angle_inc <= 0.0 or angle_max <= angle_min:
            self.get_logger().warn('Invalid angle configuration, skipping scan publish.')
            return

        count = int(math.floor((angle_max - angle_min) / angle_inc)) + 1
        ranges = [math.inf] * count

        for x, y, z in point_cloud2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True
        ):
            angle = math.atan2(y, x)
            if angle < angle_min or angle > angle_max:
                continue

            dist = math.hypot(x, y)
            if dist < self.range_min or dist > self.range_max:
                continue

            index = int((angle - angle_min) / angle_inc)
            if 0 <= index < count and dist < ranges[index]:
                ranges[index] = dist

        scan = LaserScan()
        scan.header.stamp = msg.header.stamp
        scan.header.frame_id = self.output_frame or msg.header.frame_id
        scan.angle_min = angle_min
        scan.angle_max = angle_max
        scan.angle_increment = angle_inc
        scan.time_increment = 0.0
        scan.scan_time = 0.0
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges

        self.scan_pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudToLaserScanNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
