#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan


def finite_min(values):
    finite = [v for v in values if math.isfinite(v)]
    if not finite:
        return float("inf")
    return min(finite)


class FrontScanLogger(Node):
    def __init__(self):
        super().__init__("front_scan_logger")

        self.declare_parameter("scan_topic", "/front_scan")
        self.declare_parameter("log_rate", 2.0)

        self.scan_topic = self.get_parameter("scan_topic").value
        self.latest_scan = None

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_cb,
            qos,
        )

        log_rate = float(self.get_parameter("log_rate").value)
        self.timer = self.create_timer(1.0 / log_rate, self.log_scan)

        self.get_logger().info(f"Logging sectors from {self.scan_topic}")

    def scan_cb(self, msg):
        self.latest_scan = msg

    def angle_to_index(self, scan, angle_rad):
        i = int(round((angle_rad - scan.angle_min) / scan.angle_increment))
        return max(0, min(i, len(scan.ranges) - 1))

    def sector_min(self, scan, deg_min, deg_max):
        i0 = self.angle_to_index(scan, math.radians(deg_min))
        i1 = self.angle_to_index(scan, math.radians(deg_max))

        if i0 > i1:
            i0, i1 = i1, i0

        return finite_min(scan.ranges[i0:i1 + 1])

    def fmt(self, value):
        if math.isfinite(value):
            return f"{value:.2f}m"
        return "inf"

    def log_scan(self):
        scan = self.latest_scan
        if scan is None:
            self.get_logger().warn("No /front_scan received yet")
            return

        right = self.sector_min(scan, -90, -60)
        front_right = self.sector_min(scan, -60, -15)
        front = self.sector_min(scan, -15, 15)
        front_left = self.sector_min(scan, 15, 60)
        left = self.sector_min(scan, 60, 90)

        self.get_logger().info(
            "right=%s  front_right=%s  front=%s  front_left=%s  left=%s"
            % (
                self.fmt(right),
                self.fmt(front_right),
                self.fmt(front),
                self.fmt(front_left),
                self.fmt(left),
            )
        )


def main(args=None):
    rclpy.init(args=args)
    node = FrontScanLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()