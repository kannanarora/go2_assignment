#!/usr/bin/env python3

import math
import os
import time

import numpy as np
import rclpy
from go2_interfaces.msg import PersonTrack
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CompressedImage, Image, LaserScan
from std_msgs.msg import Bool


class PersonTrackerNode(Node):
    def __init__(self):
        super().__init__("person_tracker_node")

        self.image_topic = self.declare_parameter(
            "image_topic", "/camera/color/image"
        ).value
        self.scan_topic = self.declare_parameter("scan_topic", "/front_scan").value
        self.person_track_topic = self.declare_parameter(
            "person_track_topic", "/person_track"
        ).value
        self.person_nearby_topic = self.declare_parameter(
            "person_nearby_topic", "/person_nearby"
        ).value
        self.debug_image_topic = self.declare_parameter(
            "debug_image_topic", "/person_tracker/debug_image"
        ).value
        self.debug_compressed_image_topic = self.declare_parameter(
            "debug_compressed_image_topic",
            "/person_tracker/debug_image/compressed",
        ).value

        self.model_path = self.declare_parameter("model_path", "").value
        self.input_width = int(self.declare_parameter("input_width", 640).value)
        self.input_height = int(self.declare_parameter("input_height", 640).value)
        self.min_confidence = float(self.declare_parameter("min_confidence", 0.10).value)
        self.nms_threshold = float(self.declare_parameter("nms_threshold", 0.45).value)
        self.process_every_n_frames = int(
            self.declare_parameter("process_every_n_frames", 1).value
        )
        self.publish_debug_image = bool(
            self.declare_parameter("publish_debug_image", True).value
        )
        self.debug_compressed_max_fps = float(
            self.declare_parameter("debug_compressed_max_fps", 3.0).value
        )
        self.debug_jpeg_quality = int(
            self.declare_parameter("debug_jpeg_quality", 70).value
        )
        self.debug_log_interval_s = float(
            self.declare_parameter("debug_log_interval_s", 2.0).value
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

        try:
            import cv2
        except ImportError as exc:
            raise RuntimeError(
                "person_tracker_node requires OpenCV. Install python3-opencv."
            ) from exc

        self.cv2 = cv2
        self.net = None
        self.latest_scan = None
        self.last_detection_time = 0.0
        self.last_visible = False
        self.frame_count = 0
        self.last_debug_log_time = 0.0
        self.last_debug_compressed_publish_time = 0.0
        self.last_best_person_score = 0.0
        self.last_candidate_count = 0
        self.last_output_shape = None

        self.load_model()

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
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.track_pub = self.create_publisher(PersonTrack, self.person_track_topic, 10)
        self.nearby_pub = self.create_publisher(Bool, self.person_nearby_topic, 10)
        self.debug_pub = None
        self.debug_compressed_pub = None
        if self.publish_debug_image:
            self.debug_pub = self.create_publisher(Image, self.debug_image_topic, 1)
            self.debug_compressed_pub = self.create_publisher(
                CompressedImage,
                self.debug_compressed_image_topic,
                1,
            )

        timer_period = 1.0 / max(self.no_detection_publish_hz, 0.1)
        self.no_detection_timer = self.create_timer(
            timer_period,
            self.publish_not_visible_if_stale,
        )

        self.get_logger().info(
            "PersonTrackerNode listening to %s and %s, publishing %s"
            % (self.image_topic, self.scan_topic, self.person_track_topic)
        )

    def load_model(self):
        if not self.model_path:
            self.get_logger().warn(
                "No model_path set. person_tracker_node will publish no detections "
                "until configured with a YOLO ONNX model."
            )
            return

        if not os.path.exists(self.model_path):
            self.get_logger().error("YOLO ONNX model not found: %s" % self.model_path)
            return

        self.net = self.cv2.dnn.readNetFromONNX(self.model_path)
        self.get_logger().info("Loaded YOLO ONNX model: %s" % self.model_path)

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def image_callback(self, msg: Image):
        self.frame_count += 1
        if self.process_every_n_frames > 1:
            if self.frame_count % self.process_every_n_frames != 0:
                return

        if self.net is None:
            self.publish_not_visible(msg.header)
            return

        image = self.image_msg_to_bgr(msg)
        if image is None:
            return

        detection = self.detect_best_person(image)
        if detection is None:
            self.maybe_publish_debug_image(image, msg.header, None)
            self.log_detection_debug(image.shape)
            self.publish_not_visible(msg.header)
            return

        x, y, w, h, confidence = detection
        image_x = x + w * 0.5
        image_y = y + h * 0.5

        bearing = self.image_x_to_bearing(image_x, image.shape[1])
        distance = self.scan_distance_at_bearing(bearing)
        distance_valid = math.isfinite(distance)
        nearby = distance_valid and distance <= self.nearby_threshold_m

        out = PersonTrack()
        out.header = msg.header
        out.visible = True
        out.track_id = "person"
        out.confidence = float(confidence)
        out.bearing_rad = float(bearing)
        out.distance_valid = bool(distance_valid)
        out.distance_m = float(distance) if distance_valid else 0.0
        out.nearby = bool(nearby)
        out.image_x = float(image_x)
        out.image_y = float(image_y)
        out.bbox_width = float(w)
        out.bbox_height = float(h)

        self.track_pub.publish(out)
        self.publish_nearby(nearby)
        self.maybe_publish_debug_image(image, msg.header, detection)

        self.last_detection_time = time.monotonic()
        self.last_visible = True

    def image_msg_to_bgr(self, msg: Image):
        if msg.encoding not in ("bgr8", "rgb8"):
            self.get_logger().warn("Unsupported image encoding: %s" % msg.encoding)
            return None

        channels = 3
        image = np.frombuffer(msg.data, dtype=np.uint8)
        expected = int(msg.height * msg.width * channels)
        if image.size < expected:
            self.get_logger().warn("Image data shorter than expected")
            return None

        image = image[:expected].reshape((msg.height, msg.width, channels))
        if msg.encoding == "rgb8":
            image = self.cv2.cvtColor(image, self.cv2.COLOR_RGB2BGR)

        return image

    def detect_best_person(self, image):
        blob, scale, pad_x, pad_y = self.make_blob(image)
        self.net.setInput(blob)
        output = self.net.forward()
        self.last_output_shape = tuple(output.shape)

        boxes, scores = self.parse_yolo_output(output, image.shape, scale, pad_x, pad_y)
        if not boxes:
            return None

        indices = self.cv2.dnn.NMSBoxes(
            boxes,
            scores,
            self.min_confidence,
            self.nms_threshold,
        )

        if len(indices) == 0:
            return None

        best_index = int(np.array(indices).flatten()[0])
        x, y, w, h = boxes[best_index]
        return x, y, w, h, scores[best_index]

    def make_blob(self, image):
        height, width = image.shape[:2]
        scale = min(self.input_width / width, self.input_height / height)
        resized_w = int(round(width * scale))
        resized_h = int(round(height * scale))

        resized = self.cv2.resize(image, (resized_w, resized_h))
        canvas = np.full(
            (self.input_height, self.input_width, 3),
            114,
            dtype=np.uint8,
        )

        pad_x = (self.input_width - resized_w) // 2
        pad_y = (self.input_height - resized_h) // 2
        canvas[pad_y : pad_y + resized_h, pad_x : pad_x + resized_w] = resized

        blob = self.cv2.dnn.blobFromImage(
            canvas,
            1.0 / 255.0,
            (self.input_width, self.input_height),
            swapRB=True,
            crop=False,
        )
        return blob, scale, pad_x, pad_y

    def parse_yolo_output(self, output, image_shape, scale, pad_x, pad_y):
        output = np.squeeze(output)
        self.last_best_person_score = 0.0
        self.last_candidate_count = 0

        if output.ndim != 2:
            return [], []

        if output.shape[0] < output.shape[1]:
            output = output.T

        boxes = []
        scores = []
        image_h, image_w = image_shape[:2]

        for row in output:
            parsed = self.parse_yolo_row(row, apply_threshold=False)
            if parsed is None:
                continue

            cx, cy, w, h, score = parsed
            self.last_best_person_score = max(self.last_best_person_score, score)
            if score < self.min_confidence:
                continue

            self.last_candidate_count += 1
            x = (cx - w * 0.5 - pad_x) / scale
            y = (cy - h * 0.5 - pad_y) / scale
            w = w / scale
            h = h / scale

            x = max(0.0, min(float(image_w - 1), x))
            y = max(0.0, min(float(image_h - 1), y))
            w = max(1.0, min(float(image_w) - x, w))
            h = max(1.0, min(float(image_h) - y, h))

            boxes.append([int(x), int(y), int(w), int(h)])
            scores.append(float(score))

        return boxes, scores

    def parse_yolo_row(self, row, apply_threshold=True):
        if len(row) < 6:
            return None

        cx, cy, w, h = row[0:4]

        # YOLOv5-style output: x, y, w, h, objectness, class scores...
        if len(row) >= 85:
            objectness = float(row[4])
            person_score = float(row[5])
            score = objectness * person_score
        else:
            # YOLOv8/YOLOv11-style output: x, y, w, h, class scores...
            person_score = float(row[4])
            score = person_score

        if apply_threshold and score < self.min_confidence:
            return None

        if max(abs(float(cx)), abs(float(cy)), abs(float(w)), abs(float(h))) <= 2.0:
            cx = float(cx) * self.input_width
            cy = float(cy) * self.input_height
            w = float(w) * self.input_width
            h = float(h) * self.input_height

        return float(cx), float(cy), float(w), float(h), float(score)

    def log_detection_debug(self, image_shape):
        now = time.monotonic()
        if now - self.last_debug_log_time < self.debug_log_interval_s:
            return

        self.last_debug_log_time = now
        self.get_logger().info(
            "No person accepted: best_score=%.3f threshold=%.3f "
            "candidates=%d output_shape=%s image=%dx%d"
            % (
                self.last_best_person_score,
                self.min_confidence,
                self.last_candidate_count,
                self.last_output_shape,
                image_shape[1],
                image_shape[0],
            )
        )

    def maybe_publish_debug_image(self, image, header, detection):
        wants_raw = (
            self.debug_pub is not None
            and self.debug_pub.get_subscription_count() > 0
        )
        wants_compressed = (
            self.debug_compressed_pub is not None
            and self.should_publish_debug_compressed()
        )
        if not wants_raw and not wants_compressed:
            return

        debug = image.copy()
        label = "no person %.2f" % self.last_best_person_score

        if detection is not None:
            x, y, w, h, confidence = detection
            self.cv2.rectangle(
                debug,
                (int(x), int(y)),
                (int(x + w), int(y + h)),
                (0, 255, 0),
                2,
            )
            label = "person %.2f" % confidence

        self.cv2.putText(
            debug,
            label,
            (12, 28),
            self.cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2,
            self.cv2.LINE_AA,
        )

        if wants_compressed:
            msg = self.debug_to_compressed_msg(debug, header)
            if msg is not None:
                self.debug_compressed_pub.publish(msg)
                self.last_debug_compressed_publish_time = time.monotonic()

        if not wants_raw:
            return

        msg = Image()
        msg.header = header
        msg.height = int(debug.shape[0])
        msg.width = int(debug.shape[1])
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = int(debug.shape[1] * 3)
        msg.data = debug.tobytes()
        self.debug_pub.publish(msg)

    def should_publish_debug_compressed(self):
        if self.debug_compressed_max_fps <= 0.0:
            return True
        return (
            time.monotonic() - self.last_debug_compressed_publish_time
            >= 1.0 / self.debug_compressed_max_fps
        )

    def debug_to_compressed_msg(self, debug, header):
        quality = max(1, min(100, self.debug_jpeg_quality))
        ok, encoded = self.cv2.imencode(
            ".jpg",
            debug,
            [int(self.cv2.IMWRITE_JPEG_QUALITY), quality],
        )
        if not ok:
            return None

        msg = CompressedImage()
        msg.header = header
        msg.format = "jpeg"
        msg.data = encoded.tobytes()
        return msg

    def image_x_to_bearing(self, image_x: float, image_width_px: int) -> float:
        half_width = max(image_width_px * 0.5, 1.0)
        centered_x = image_x - half_width
        normalized = max(-1.0, min(1.0, centered_x / half_width))
        half_fov_rad = math.radians(self.camera_horizontal_fov_deg) * 0.5

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
