"""
Computes the closest depth in a forward-facing angular sector from a point cloud.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32


class FrontDepthNode(Node):
    def __init__(self):
        super().__init__('front_depth_node')

        self.cloud_topic = self.declare_parameter('cloud_topic', '/utlidar/cloud').value
        self.fov_deg = float(self.declare_parameter('fov_deg', 210.0).value)
        self.frame_id = self.declare_parameter('frame_id', 'base_link').value

        self.depth_pub = self.create_publisher(Float32, 'front_depth', 10)
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            self.cloud_topic,
            self.cloud_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            'FrontDepthNode listening on %s (fov_deg=%.1f).'
            % (self.cloud_topic, self.fov_deg)
        )

    def cloud_callback(self, msg: PointCloud2):
        half_fov_rad = math.radians(self.fov_deg / 2.0)

        min_range = None
        for x, y, z in point_cloud2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True
        ):
            if x <= 0.0:
                continue

            angle = abs(math.atan2(y, x))
            if angle > half_fov_rad:
                continue

            dist = math.hypot(x, y)
            if min_range is None or dist < min_range:
                min_range = dist

        if min_range is None:
            return

        out = Float32()
        out.data = float(min_range)
        self.depth_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = FrontDepthNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
