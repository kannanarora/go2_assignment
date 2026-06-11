#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import PointCloud2


class CloudThrottle(Node):
    def __init__(self):
        super().__init__("cloud_throttle")

        self.declare_parameter("input_topic", "/utlidar/cloud_deskewed")
        self.declare_parameter("output_topic", "/utlidar/cloud_deskewed_viz")
        self.declare_parameter("publish_rate", 2.0)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        publish_rate = float(self.get_parameter("publish_rate").value)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.latest_msg = None

        self.pub = self.create_publisher(PointCloud2, output_topic, qos)
        self.sub = self.create_subscription(PointCloud2, input_topic, self.cb, qos)

        self.timer = self.create_timer(1.0 / publish_rate, self.publish_latest)

        self.get_logger().info(
            f"Throttling {input_topic} -> {output_topic} at {publish_rate:.2f} Hz"
        )

    def cb(self, msg):
        self.latest_msg = msg

    def publish_latest(self):
        if self.latest_msg is not None:
            self.pub.publish(self.latest_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CloudThrottle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()