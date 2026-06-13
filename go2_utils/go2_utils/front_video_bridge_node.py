#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from unitree_go.msg import Go2FrontVideoData


class FrontVideoBridgeNode(Node):
    def __init__(self):
        super().__init__("front_video_bridge_node")

        self.input_topic = self.declare_parameter(
            "input_topic", "/frontvideostream"
        ).value
        self.output_topic = self.declare_parameter(
            "output_topic", "/camera/image_raw"
        ).value
        self.compressed_output_topic = self.declare_parameter(
            "compressed_output_topic", "/camera/image_raw/compressed"
        ).value
        self.frame_id = self.declare_parameter("frame_id", "front_camera").value
        self.stream = self.declare_parameter("stream", "video720p").value
        self.publish_raw = bool(self.declare_parameter("publish_raw", True).value)
        self.publish_compressed = bool(
            self.declare_parameter("publish_compressed", True).value
        )
        self.raw_max_fps = float(self.declare_parameter("raw_max_fps", 10.0).value)
        self.compressed_max_fps = float(
            self.declare_parameter("compressed_max_fps", 5.0).value
        )
        self.jpeg_quality = int(self.declare_parameter("jpeg_quality", 70).value)
        self.log_decode_errors = bool(
            self.declare_parameter("log_decode_errors", False).value
        )

        try:
            import av
        except ImportError as exc:
            raise RuntimeError(
                "front_video_bridge_node requires PyAV. Install python3-av "
                "or add av to the robot Python environment."
            ) from exc

        self._av = av
        self.decoder = av.CodecContext.create("h264", "r")
        self._last_raw_publish_time = 0.0
        self._last_compressed_publish_time = 0.0
        self._warned_no_cv2 = False

        self._cv2 = None
        if self.publish_compressed:
            try:
                import cv2

                self._cv2 = cv2
            except ImportError:
                self.get_logger().warn(
                    "Compressed image output requires python3-opencv. "
                    "Raw image output will still be published."
                )

        self.image_pub = self.create_publisher(Image, self.output_topic, 10)
        self.compressed_pub = self.create_publisher(
            CompressedImage,
            self.compressed_output_topic,
            10,
        )
        self.video_sub = self.create_subscription(
            Go2FrontVideoData,
            self.input_topic,
            self.video_callback,
            qos_profile_sensor_data,
            raw=True,
        )

        self.get_logger().info(
            "FrontVideoBridgeNode decoding %s.%s -> raw:%s compressed:%s"
            % (
                self.input_topic,
                self.stream,
                self.output_topic if self.publish_raw else "disabled",
                self.compressed_output_topic if self.publish_compressed else "disabled",
            )
        )

    def video_callback(self, serialized_msg: bytes):
        encoded = self.extract_stream_bytes(serialized_msg)
        encoded = self.strip_to_h264_start_code(encoded)

        if not encoded:
            return

        try:
            packet = self._av.Packet(encoded)
            frames = self.decoder.decode(packet)
        except Exception as exc:
            if self.log_decode_errors:
                self.get_logger().warn("H.264 decode failed: %s" % exc)
            return

        for frame in frames:
            self.publish_frame(frame)

    def publish_frame(self, frame):
        bgr = frame.to_ndarray(format="bgr24")
        now = time.monotonic()
        stamp = self.get_clock().now().to_msg()

        if self.publish_raw and self.should_publish(now, self._last_raw_publish_time, self.raw_max_fps):
            image_msg = self.bgr_to_image_msg(bgr, stamp)
            self.image_pub.publish(image_msg)
            self._last_raw_publish_time = now

        if (
            self.publish_compressed
            and self.should_publish(
                now,
                self._last_compressed_publish_time,
                self.compressed_max_fps,
            )
        ):
            compressed_msg = self.bgr_to_compressed_msg(bgr, stamp)
            if compressed_msg is not None:
                self.compressed_pub.publish(compressed_msg)
                self._last_compressed_publish_time = now

    def should_publish(self, now: float, last_publish_time: float, max_fps: float) -> bool:
        if max_fps <= 0.0:
            return True

        return now - last_publish_time >= 1.0 / max_fps

    def extract_stream_bytes(self, serialized_msg: bytes) -> bytes:
        streams = self.parse_front_video_cdr(serialized_msg)

        if self.stream == "video360p":
            data = streams["video360p"]
        elif self.stream == "video180p":
            data = streams["video180p"]
        else:
            data = streams["video720p"]

        if data:
            return data

        return self.find_h264_payload(serialized_msg)

    def parse_front_video_cdr(self, data: bytes) -> dict:
        streams = {
            "video720p": b"",
            "video360p": b"",
            "video180p": b"",
        }

        try:
            offset = 4
            little_endian = data[1] == 1
            offset = self.align(offset, 8)
            offset += 8

            for name in ("video720p", "video360p", "video180p"):
                offset = self.align(offset, 4)
                length = self.read_uint32(data, offset, little_endian)
                offset += 4

                if length < 0 or offset + length > len(data):
                    return streams

                streams[name] = data[offset : offset + length]
                offset += length

            return streams
        except Exception:
            return streams

    def find_h264_payload(self, data: bytes) -> bytes:
        start = data.find(b"\x00\x00\x00\x01")
        if start < 0:
            start = data.find(b"\x00\x00\x01")

        if start < 0:
            return b""

        return data[start:]

    def align(self, offset: int, alignment: int) -> int:
        remainder = offset % alignment
        if remainder == 0:
            return offset
        return offset + alignment - remainder

    def read_uint32(self, data: bytes, offset: int, little_endian: bool) -> int:
        byteorder = "little" if little_endian else "big"
        return int.from_bytes(data[offset : offset + 4], byteorder=byteorder)

    def strip_to_h264_start_code(self, data: bytes) -> bytes:
        if not data:
            return b""

        start = data.find(b"\x00\x00\x00\x01")
        if start < 0:
            start = data.find(b"\x00\x00\x01")

        if start < 0:
            return data

        return data[start:]

    def bgr_to_image_msg(self, bgr, stamp) -> Image:
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.height = int(bgr.shape[0])
        msg.width = int(bgr.shape[1])
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = int(msg.width * 3)
        msg.data = bgr.tobytes()
        return msg

    def bgr_to_compressed_msg(self, bgr, stamp):
        if self._cv2 is None:
            if not self._warned_no_cv2:
                self.get_logger().warn("Skipping compressed output: cv2 is unavailable")
                self._warned_no_cv2 = True
            return None

        quality = max(1, min(100, self.jpeg_quality))
        ok, encoded = self._cv2.imencode(
            ".jpg",
            bgr,
            [int(self._cv2.IMWRITE_JPEG_QUALITY), quality],
        )

        if not ok:
            return None

        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.format = "jpeg"
        msg.data = encoded.tobytes()
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = FrontVideoBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
