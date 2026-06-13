#!/usr/bin/env python3

import math
import time

import rclpy
from go2_interfaces.msg import PersonTrack
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from yolo_msgs.msg import DetectionArray


class PersonTrackerNode(Node):
    def __init__(self):
        super().__init__("person_tracker_node")

        self.yolo_topic = self.declare_parameter(
            "yolo_topic", "/yolo/tracking"
        ).value
        self.scan_topic = self.declare_parameter("scan_topic", "/front_scan").value
        self.person_track_topic = self.declare_parameter(
            "person_track_topic", "/person_track"
        ).value
        self.person_nearby_topic = self.declare_parameter(
            "person_nearby_topic", "/person_nearby"
        ).value

        self.image_width_px = float(
            self.declare_parameter("image_width_px", 640.0).value
        )
        self.camera_horizontal_fov_deg = float(
            self.declare_parameter("camera_horizontal_fov_deg", 90.0).value
        )
        self.scan_sector_half_angle_deg = float(
            self.declare_parameter("scan_sector_half_angle_deg", 5.0).value
        )
        self.nearby_threshold_m = float(
            self.declare_parameter("nearby_threshold_m", 1.5).value
        )
        self.detection_timeout_s = float(
            self.declare_parameter("detection_timeout_s", 0.7).value
        )
        self.no_detection_publish_hz = float(
            self.declare_parameter("no_detection_publish_hz", 2.0).value
        )
        self.min_confidence = float(self.declare_parameter("min_confidence", 0.35).value)

        self.latest_scan = None
        self.last_detection_time = 0.0
        self.last_visible = False

        scan_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            scan_qos,
        )
        self.yolo_sub = self.create_subscription(
            DetectionArray,
            self.yolo_topic,
            self.yolo_callback,
            10,
        )
        self.track_pub = self.create_publisher(PersonTrack, self.person_track_topic, 10)
        self.nearby_pub = self.create_publisher(Bool, self.person_nearby_topic, 10)

        timer_period = 1.0 / max(self.no_detection_publish_hz, 0.1)
        self.no_detection_timer = self.create_timer(
            timer_period,
            self.publish_not_visible_if_stale,
        )

        self.get_logger().info(
            "PersonTrackerNode listening to %s and %s, publishing %s"
            % (self.yolo_topic, self.scan_topic, self.person_track_topic)
        )

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def yolo_callback(self, msg: DetectionArray):
        best = self.best_person_detection(msg.detections)

        if best is None:
            self.publish_not_visible(msg.header)
            return

        confidence = float(best.score)
        bbox = best.bbox
        image_x = float(bbox.center.position.x)
        image_y = float(bbox.center.position.y)
        bbox_width = float(bbox.size.x)
        bbox_height = float(bbox.size.y)

        bearing = self.image_x_to_bearing(image_x)
        distance = self.scan_distance_at_bearing(bearing)
        distance_valid = math.isfinite(distance)
        nearby = distance_valid and distance <= self.nearby_threshold_m

        out = PersonTrack()
        out.header = msg.header
        out.visible = True
        out.track_id = str(best.id)
        out.confidence = confidence
        out.bearing_rad = float(bearing)
        out.distance_valid = bool(distance_valid)
        out.distance_m = float(distance) if distance_valid else 0.0
        out.nearby = bool(nearby)
        out.image_x = image_x
        out.image_y = image_y
        out.bbox_width = bbox_width
        out.bbox_height = bbox_height

        self.track_pub.publish(out)
        self.publish_nearby(nearby)

        self.last_detection_time = time.monotonic()
        self.last_visible = True

    def best_person_detection(self, detections):
        best = None
        best_score = -1.0

        for detection in detections:
            class_name = str(detection.class_name).lower()
            is_person = class_name == "person" or int(detection.class_id) == 0
            score = float(detection.score)

            if not is_person or score < self.min_confidence:
                continue

            if score > best_score:
                best = detection
                best_score = score

        return best

    def image_x_to_bearing(self, image_x: float) -> float:
        half_width = max(self.image_width_px * 0.5, 1.0)
        centered_x = image_x - half_width
        normalized = max(-1.0, min(1.0, centered_x / half_width))
        half_fov_rad = math.radians(self.camera_horizontal_fov_deg) * 0.5

        # Image coordinates increase to the right, while robot yaw is positive left.
        return -normalized * half_fov_rad

    def scan_distance_at_bearing(self, bearing_rad: float) -> float:
        scan = self.latest_scan
        if scan is None or not scan.ranges:
            return float("inf")

        half_angle = math.radians(self.scan_sector_half_angle_deg)
        i0 = self.angle_to_index(scan, bearing_rad - half_angle)
        i1 = self.angle_to_index(scan, bearing_rad + half_angle)
        if i0 > i1:
            i0, i1 = i1, i0

        valid_ranges = []
        for value in scan.ranges[i0 : i1 + 1]:
            if not math.isfinite(value):
                continue
            value = float(value)
            if value <= 0.0 or value < scan.range_min or value > scan.range_max:
                continue
            valid_ranges.append(value)

        if not valid_ranges:
            return float("inf")
        return min(valid_ranges)

    def angle_to_index(self, scan: LaserScan, angle_rad: float) -> int:
        if scan.angle_increment == 0.0 or len(scan.ranges) == 0:
            return 0

        idx = int(round((angle_rad - scan.angle_min) / scan.angle_increment))
        return max(0, min(idx, len(scan.ranges) - 1))

    def publish_not_visible_if_stale(self):
        if not self.last_visible:
            return

        if time.monotonic() - self.last_detection_time < self.detection_timeout_s:
            return

        self.publish_not_visible()

    def publish_not_visible(self, header=None):
        out = PersonTrack()
        if header is not None:
            out.header = header
        else:
            out.header.stamp = self.get_clock().now().to_msg()

        out.visible = False
        out.track_id = ""
        out.confidence = 0.0
        out.bearing_rad = 0.0
        out.distance_valid = False
        out.distance_m = 0.0
        out.nearby = False

        self.track_pub.publish(out)
        self.publish_nearby(False)
        self.last_visible = False

    def publish_nearby(self, nearby: bool):
        msg = Bool()
        msg.data = bool(nearby)
        self.nearby_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PersonTrackerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
