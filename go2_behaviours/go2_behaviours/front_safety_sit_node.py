#!/usr/bin/env python3

"""
Sits the robot when an obstacle is too close in /front_scan.

Expected input:
  /front_scan  sensor_msgs/msg/LaserScan

This is intended for the Go2 setup:
  /utlidar/cloud_deskewed -> pointcloud_to_laserscan -> /front_scan
  target_frame: base_footprint

`inf` values are treated as clear space.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)

from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class FrontSafetySitNode(Node):
    def __init__(self):
        super().__init__("front_safety_sit_node")

        self.scan_topic = self.declare_parameter("scan_topic", "/front_scan").value
        self.trigger_topic = self.declare_parameter(
            "trigger_topic", "/trigger_behaviour"
        ).value

        self.sit_threshold_m = float(
            self.declare_parameter("sit_threshold_m", 0.8).value
        )
        self.clear_threshold_m = float(
            self.declare_parameter("clear_threshold_m", 1.2).value
        )

        self.front_half_angle_deg = float(
            self.declare_parameter("front_half_angle_deg", 15.0).value
        )

        self.required_blocked_frames = int(
            self.declare_parameter("required_blocked_frames", 2).value
        )
        self.required_clear_frames = int(
            self.declare_parameter("required_clear_frames", 4).value
        )

        self.sit_command = self.declare_parameter("sit_command", "sit").value
        self.stand_command = self.declare_parameter(
            "stand_command", "rise_sit"
        ).value

        self.enable_stand_command = bool(
            self.declare_parameter("enable_stand_command", True).value
        )

        self.dry_run = bool(self.declare_parameter("dry_run", True).value)

        self.log_rate_hz = float(self.declare_parameter("log_rate_hz", 2.0).value)
        self._last_log_time = 0.0

        if self.clear_threshold_m <= self.sit_threshold_m:
            self.clear_threshold_m = self.sit_threshold_m + 0.2
            self.get_logger().warn(
                "clear_threshold_m must be > sit_threshold_m; adjusted to %.2f"
                % self.clear_threshold_m
            )

        self._is_sitting = False
        self._blocked_count = 0
        self._clear_count = 0

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.cmd_pub = self.create_publisher(String, self.trigger_topic, 10)
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            qos,
        )

        self.get_logger().info(
            "FrontSafetySitNode listening on %s. sit<%.2fm, clear>%.2fm, "
            "front=±%.1fdeg, dry_run=%s"
            % (
                self.scan_topic,
                self.sit_threshold_m,
                self.clear_threshold_m,
                self.front_half_angle_deg,
                self.dry_run,
            )
        )

    def angle_to_index(self, scan: LaserScan, angle_rad: float) -> int:
        if scan.angle_increment == 0.0 or len(scan.ranges) == 0:
            return 0

        idx = int(round((angle_rad - scan.angle_min) / scan.angle_increment))
        return max(0, min(idx, len(scan.ranges) - 1))

    def front_min_range(self, scan: LaserScan) -> float:
        if not scan.ranges:
            return float("inf")

        half_angle_rad = math.radians(self.front_half_angle_deg)

        i0 = self.angle_to_index(scan, -half_angle_rad)
        i1 = self.angle_to_index(scan, half_angle_rad)

        if i0 > i1:
            i0, i1 = i1, i0

        valid_ranges = []

        for value in scan.ranges[i0 : i1 + 1]:
            if not math.isfinite(value):
                continue

            value = float(value)

            if value <= 0.0:
                continue

            if value < scan.range_min:
                continue

            if value > scan.range_max:
                continue

            valid_ranges.append(value)

        if not valid_ranges:
            return float("inf")

        return min(valid_ranges)

    def scan_callback(self, scan: LaserScan):
        front_range = self.front_min_range(scan)

        blocked = math.isfinite(front_range) and front_range < self.sit_threshold_m
        clear = (not math.isfinite(front_range)) or front_range > self.clear_threshold_m

        if blocked:
            self._blocked_count += 1
            self._clear_count = 0
        elif clear:
            self._clear_count += 1
            self._blocked_count = 0
        else:
            self._blocked_count = 0
            self._clear_count = 0

        if not self._is_sitting:
            if self._blocked_count >= self.required_blocked_frames:
                self.publish_command(self.sit_command)
                self._is_sitting = True
                self._blocked_count = 0
                self._clear_count = 0
        else:
            if (
                self.enable_stand_command
                and self._clear_count >= self.required_clear_frames
            ):
                self.publish_command(self.stand_command)
                self._is_sitting = False
                self._blocked_count = 0
                self._clear_count = 0

        self.maybe_log(front_range, blocked, clear)

    def maybe_log(self, front_range: float, blocked: bool, clear: bool):
        if self.log_rate_hz <= 0.0:
            return

        now = time.monotonic()
        if now - self._last_log_time < 1.0 / self.log_rate_hz:
            return

        self._last_log_time = now

        if math.isfinite(front_range):
            front_text = "%.2fm" % front_range
        else:
            front_text = "inf"

        self.get_logger().info(
            "front=%s blocked=%s clear=%s sitting=%s "
            "blocked_count=%d clear_count=%d"
            % (
                front_text,
                blocked,
                clear,
                self._is_sitting,
                self._blocked_count,
                self._clear_count,
            )
        )

    def publish_command(self, command: str):
        if self.dry_run:
            # self.get_logger().warn("[DRY RUN] Would send command: %s" % command)
            return

        msg = String()
        msg.data = command
        self.cmd_pub.publish(msg)
        # self.get_logger().warn("Sent command: %s" % command)


def main(args=None):
    rclpy.init(args=args)
    node = FrontSafetySitNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()