#!/usr/bin/env python3

import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image


class VideoClientCameraNode(Node):
    def __init__(self):
        super().__init__("video_client_camera_node")

        self.output_topic = self.declare_parameter(
            "output_topic", "/camera/color/image"
        ).value
        self.compressed_output_topic = self.declare_parameter(
            "compressed_output_topic", "/camera/color/image/compressed"
        ).value
        self.frame_id = self.declare_parameter("frame_id", "front_camera").value
        self.network_interface = self.declare_parameter(
            "network_interface", "eth0"
        ).value
        self.output_width = int(self.declare_parameter("output_width", 640).value)
        self.output_height = int(self.declare_parameter("output_height", 360).value)
        self.publish_raw = bool(self.declare_parameter("publish_raw", True).value)
        self.publish_compressed = bool(
            self.declare_parameter("publish_compressed", True).value
        )
        self.raw_max_fps = float(self.declare_parameter("raw_max_fps", 10.0).value)
        self.compressed_max_fps = float(
            self.declare_parameter("compressed_max_fps", 5.0).value
        )
        self.request_timeout_s = float(
            self.declare_parameter("request_timeout_s", 1.0).value
        )
        self.poll_hz = float(self.declare_parameter("poll_hz", 30.0).value)

        try:
            import cv2
        except ImportError as exc:
            raise RuntimeError(
                "video_client_camera_node requires OpenCV. Install python3-opencv."
            ) from exc

        try:
            from unitree_sdk2py.core.channel import ChannelFactoryInitialize
            from unitree_sdk2py.go2.video.video_client import VideoClient
        except ImportError as exc:
            raise RuntimeError(
                "video_client_camera_node requires unitree_sdk2py. "
                "Install Unitree's Python SDK on the Jetson."
            ) from exc

        self.cv2 = cv2
        self.last_raw_publish_time = 0.0
        self.last_compressed_publish_time = 0.0
        self.logged_first_frame = False
        self.last_error_log_time = 0.0

        ChannelFactoryInitialize(0, self.network_interface)
        self.client = VideoClient()
        self.client.SetTimeout(self.request_timeout_s)
        self.client.Init()

        self.image_pub = self.create_publisher(
            Image,
            self.output_topic,
            qos_profile_sensor_data,
        )
        self.compressed_pub = self.create_publisher(
            CompressedImage,
            self.compressed_output_topic,
            qos_profile_sensor_data,
        )

        timer_period = 1.0 / max(self.poll_hz, 0.1)
        self.timer = self.create_timer(timer_period, self.poll_camera)

        self.get_logger().info(
            "Publishing Unitree VideoClient camera on %s -> raw:%s compressed:%s"
            % (
                self.network_interface,
                self.output_topic if self.publish_raw else "disabled",
                self.compressed_output_topic if self.publish_compressed else "disabled",
            )
        )

    def poll_camera(self):
        now = self.get_clock().now()
        now_s = now.nanoseconds * 1e-9
        stamp = now.to_msg()

        wants_raw = (
            self.publish_raw
            and self.image_pub.get_subscription_count() > 0
            and self.should_publish(now_s, self.last_raw_publish_time, self.raw_max_fps)
        )
        wants_compressed = (
            self.publish_compressed
            and self.compressed_pub.get_subscription_count() > 0
            and self.should_publish(
                now_s,
                self.last_compressed_publish_time,
                self.compressed_max_fps,
            )
        )

        if not wants_raw and not wants_compressed:
            return

        code, data = self.client.GetImageSample()
        if code != 0:
            self.log_error_throttled("GetImageSample failed with code: %s" % code)
            return

        jpeg_bytes = bytes(data)
        if wants_compressed:
            self.compressed_pub.publish(
                self.jpeg_bytes_to_compressed_msg(jpeg_bytes, stamp)
            )
            self.last_compressed_publish_time = now_s

        if not wants_raw:
            return

        image = self.decode_jpeg(jpeg_bytes)
        if image is None:
            return

        if not self.logged_first_frame:
            self.logged_first_frame = True
            self.get_logger().info(
                "VideoClient camera frame: width=%d height=%d encoding=bgr8"
                % (image.shape[1], image.shape[0])
            )

        image = self.resize_if_needed(image)
        self.image_pub.publish(self.bgr_to_image_msg(image, stamp))
        self.last_raw_publish_time = now_s

    def should_publish(self, now_s, last_publish_s, max_fps):
        if max_fps <= 0.0:
            return True
        return now_s - last_publish_s >= 1.0 / max_fps

    def jpeg_bytes_to_compressed_msg(self, jpeg_bytes, stamp):
        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.format = "jpeg"
        msg.data = jpeg_bytes
        return msg

    def decode_jpeg(self, jpeg_bytes):
        image_data = np.frombuffer(jpeg_bytes, dtype=np.uint8)
        image = self.cv2.imdecode(image_data, self.cv2.IMREAD_COLOR)
        if image is None:
            self.log_error_throttled("Failed to decode VideoClient JPEG frame")
        return image

    def resize_if_needed(self, image):
        if self.output_width <= 0 or self.output_height <= 0:
            return image

        if image.shape[1] == self.output_width and image.shape[0] == self.output_height:
            return image

        return self.cv2.resize(
            image,
            (self.output_width, self.output_height),
            interpolation=self.cv2.INTER_AREA,
        )

    def bgr_to_image_msg(self, image, stamp):
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.height = int(image.shape[0])
        msg.width = int(image.shape[1])
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = int(image.shape[1] * 3)
        msg.data = image.tobytes()
        return msg

    def log_error_throttled(self, text):
        now = time.monotonic()
        if now - self.last_error_log_time < 2.0:
            return

        self.last_error_log_time = now
        self.get_logger().warn(text)


def main(args=None):
    rclpy.init(args=args)
    node = VideoClientCameraNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
