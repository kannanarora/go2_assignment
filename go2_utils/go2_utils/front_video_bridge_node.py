#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
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
        self.frame_id = self.declare_parameter("frame_id", "front_camera").value
        self.stream = self.declare_parameter("stream", "video720p").value
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

        self.image_pub = self.create_publisher(Image, self.output_topic, 10)
        self.video_sub = self.create_subscription(
            Go2FrontVideoData,
            self.input_topic,
            self.video_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            "FrontVideoBridgeNode decoding %s.%s -> %s"
            % (self.input_topic, self.stream, self.output_topic)
        )

    def video_callback(self, msg: Go2FrontVideoData):
        encoded = self.extract_stream_bytes(msg)
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
            image_msg = self.frame_to_image_msg(frame)
            self.image_pub.publish(image_msg)

    def extract_stream_bytes(self, msg: Go2FrontVideoData) -> bytes:
        if self.stream == "video360p":
            data = msg.video360p
        elif self.stream == "video180p":
            data = msg.video180p
        else:
            data = msg.video720p

        return bytes(data)

    def strip_to_h264_start_code(self, data: bytes) -> bytes:
        if not data:
            return b""

        start = data.find(b"\x00\x00\x00\x01")
        if start < 0:
            start = data.find(b"\x00\x00\x01")

        if start < 0:
            return data

        return data[start:]

    def frame_to_image_msg(self, frame) -> Image:
        bgr = frame.to_ndarray(format="bgr24")

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height = int(bgr.shape[0])
        msg.width = int(bgr.shape[1])
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = int(msg.width * 3)
        msg.data = bgr.tobytes()
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
