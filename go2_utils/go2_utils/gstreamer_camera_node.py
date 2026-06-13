#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image


class GStreamerCameraNode(Node):
    def __init__(self):
        super().__init__("gstreamer_camera_node")

        self.output_topic = self.declare_parameter(
            "output_topic", "/camera/color/image"
        ).value
        self.compressed_output_topic = self.declare_parameter(
            "compressed_output_topic", "/camera/color/image/compressed"
        ).value
        self.frame_id = self.declare_parameter("frame_id", "front_camera").value
        self.multicast_address = self.declare_parameter(
            "multicast_address", "230.1.1.1"
        ).value
        self.port = int(self.declare_parameter("port", 1720).value)
        self.interface = self.declare_parameter("interface", "eth0").value
        self.width = int(self.declare_parameter("width", 1280).value)
        self.height = int(self.declare_parameter("height", 720).value)
        self.publish_raw = bool(self.declare_parameter("publish_raw", True).value)
        self.publish_compressed = bool(
            self.declare_parameter("publish_compressed", True).value
        )
        self.jpeg_quality = int(self.declare_parameter("jpeg_quality", 70).value)

        try:
            import gi

            gi.require_version("Gst", "1.0")
            from gi.repository import Gst
        except ImportError as exc:
            raise RuntimeError(
                "gstreamer_camera_node requires python3-gi and GStreamer bindings."
            ) from exc

        Gst.init(None)
        self.Gst = Gst

        try:
            import cv2
        except ImportError:
            cv2 = None

        self.cv2 = cv2
        self.pipeline = None
        self.appsink = None
        self.logged_first_frame = False

        self.image_pub = self.create_publisher(Image, self.output_topic, 10)
        self.compressed_pub = self.create_publisher(
            CompressedImage,
            self.compressed_output_topic,
            10,
        )

        self.open_pipeline()

    def build_pipeline(self):
        return (
            "udpsrc address=%s port=%d multicast-iface=%s "
            "! queue "
            "! application/x-rtp, media=video, encoding-name=H264 "
            "! rtph264depay "
            "! h264parse "
            "! avdec_h264 "
            "! videoconvert "
            "! video/x-raw,width=%d,height=%d,format=BGR "
            "! appsink name=sink emit-signals=true max-buffers=1 drop=true sync=false"
        ) % (
            self.multicast_address,
            self.port,
            self.interface,
            self.width,
            self.height,
        )

    def open_pipeline(self):
        pipeline = self.build_pipeline()
        self.get_logger().info(
            "Opening Go2 GStreamer multicast camera on %s" % self.interface
        )

        self.pipeline = self.Gst.parse_launch(pipeline)
        self.appsink = self.pipeline.get_by_name("sink")
        self.appsink.connect("new-sample", self.on_new_sample)
        self.pipeline.set_state(self.Gst.State.PLAYING)

        self.get_logger().info(
            "Publishing Go2 camera stream -> raw:%s compressed:%s"
            % (
                self.output_topic if self.publish_raw else "disabled",
                self.compressed_output_topic if self.publish_compressed else "disabled",
            )
        )

    def on_new_sample(self, sink):
        sample = sink.emit("pull-sample")
        if sample is None:
            return self.Gst.FlowReturn.ERROR

        buffer = sample.get_buffer()
        caps = sample.get_caps()
        structure = caps.get_structure(0)
        width = int(structure.get_value("width"))
        height = int(structure.get_value("height"))

        ok, map_info = buffer.map(self.Gst.MapFlags.READ)
        if not ok:
            return self.Gst.FlowReturn.ERROR

        if not self.logged_first_frame:
            self.logged_first_frame = True
            self.get_logger().info(
                "GStreamer camera frame: width=%d height=%d encoding=bgr8"
                % (width, height)
            )

        try:
            frame_bytes = bytes(map_info.data)
            stamp = self.get_clock().now().to_msg()

            if self.publish_raw:
                self.image_pub.publish(
                    self.bgr_bytes_to_image_msg(frame_bytes, width, height, stamp)
                )

            if self.publish_compressed:
                msg = self.bgr_bytes_to_compressed_msg(
                    frame_bytes,
                    width,
                    height,
                    stamp,
                )
                if msg is not None:
                    self.compressed_pub.publish(msg)
        finally:
            buffer.unmap(map_info)

        return self.Gst.FlowReturn.OK

    def bgr_bytes_to_image_msg(self, frame_bytes, width, height, stamp):
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.height = int(height)
        msg.width = int(width)
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = int(msg.width * 3)
        msg.data = frame_bytes
        return msg

    def bgr_bytes_to_compressed_msg(self, frame_bytes, width, height, stamp):
        if self.cv2 is None:
            return None

        import numpy as np

        bgr = np.frombuffer(frame_bytes, dtype=np.uint8).reshape((height, width, 3))
        quality = max(1, min(100, self.jpeg_quality))
        ok, encoded = self.cv2.imencode(
            ".jpg",
            bgr,
            [int(self.cv2.IMWRITE_JPEG_QUALITY), quality],
        )
        if not ok:
            return None

        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.format = "jpeg"
        msg.data = encoded.tobytes()
        return msg

    def destroy_node(self):
        if self.pipeline is not None:
            self.pipeline.set_state(self.Gst.State.NULL)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GStreamerCameraNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
