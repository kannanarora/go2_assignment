#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image


class GStreamerCameraNode(Node):
    def __init__(self):
        super().__init__("gstreamer_camera_node")

        self.output_topic = self.declare_parameter(
            "output_topic", "/camera/color/image"
        ).value
        self.compressed_output_topic = self.declare_parameter(
            "compressed_output_topic", "/camera/color/image/compressed"
        ).value
        self.camera_info_topic = self.declare_parameter(
            "camera_info_topic", "/camera/color/camera_info"
        ).value
        self.frame_id = self.declare_parameter("frame_id", "front_camera").value
        self.multicast_address = self.declare_parameter(
            "multicast_address", "230.1.1.1"
        ).value
        self.port = int(self.declare_parameter("port", 1720).value)
        self.interface = self.declare_parameter("interface", "eth0").value
        self.width = int(self.declare_parameter("width", 1280).value)
        self.height = int(self.declare_parameter("height", 720).value)
        self.output_width = int(self.declare_parameter("output_width", 640).value)
        self.output_height = int(self.declare_parameter("output_height", 360).value)
        self.decoder = self.declare_parameter("decoder", "nvidia").value
        self.publish_raw = bool(self.declare_parameter("publish_raw", True).value)
        self.publish_compressed = bool(
            self.declare_parameter("publish_compressed", True).value
        )
        self.jitter_latency_ms = int(
            self.declare_parameter("jitter_latency_ms", 100).value
        )
        self.jitter_drop_on_latency = bool(
            self.declare_parameter("jitter_drop_on_latency", True).value
        )
        self.depay_wait_for_keyframe = bool(
            self.declare_parameter("depay_wait_for_keyframe", True).value
        )
        self.depay_request_keyframe = bool(
            self.declare_parameter("depay_request_keyframe", True).value
        )
        self.h264_config_interval = int(
            self.declare_parameter("h264_config_interval", -1).value
        )
        self.disable_dpb = bool(self.declare_parameter("disable_dpb", False).value)
        self.raw_max_fps = float(self.declare_parameter("raw_max_fps", 10.0).value)
        self.compressed_max_fps = float(
            self.declare_parameter("compressed_max_fps", 3.0).value
        )
        self.jpeg_quality = int(self.declare_parameter("jpeg_quality", 70).value)
        self.camera_horizontal_fov_deg = float(
            self.declare_parameter("camera_horizontal_fov_deg", 90.0).value
        )

        try:
            import gi

            gi.require_version("Gst", "1.0")
            gi.require_version("GstVideo", "1.0")
            from gi.repository import Gst
            from gi.repository import GstVideo
        except ImportError as exc:
            raise RuntimeError(
                "gstreamer_camera_node requires python3-gi and GStreamer bindings."
            ) from exc

        Gst.init(None)
        self.Gst = Gst
        self.GstVideo = GstVideo

        try:
            import cv2
        except ImportError as exc:
            raise RuntimeError(
                "gstreamer_camera_node requires OpenCV for resize/JPEG publishing. "
                "Install python3-opencv."
            ) from exc

        self.cv2 = cv2
        self.pipeline = None
        self.appsink = None
        self.logged_first_frame = False
        self.last_raw_publish_time = 0.0
        self.last_compressed_publish_time = 0.0

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
        self.camera_info_pub = self.create_publisher(
            CameraInfo,
            self.camera_info_topic,
            qos_profile_sensor_data,
        )

        self.open_pipeline()

    def build_pipeline(self):
        output_width = self.output_width if self.output_width > 0 else self.width
        output_height = self.output_height if self.output_height > 0 else self.height
        jitter_pipeline = self.build_jitter_pipeline()
        depay_pipeline = self.build_depay_pipeline()
        parser_pipeline = self.build_h264_parser_pipeline()
        decoder_pipeline = self.build_decoder_pipeline(output_width, output_height)

        return (
            "udpsrc address=%s port=%d multicast-iface=%s "
            "! application/x-rtp,media=video,encoding-name=H264,clock-rate=90000 "
            "%s "
            "%s "
            "%s "
            "%s "
            "! queue max-size-buffers=1 max-size-time=0 max-size-bytes=0 leaky=downstream "
            "! appsink name=sink emit-signals=true max-buffers=1 drop=true sync=false"
        ) % (
            self.multicast_address,
            self.port,
            self.interface,
            jitter_pipeline,
            depay_pipeline,
            parser_pipeline,
            decoder_pipeline,
        )

    def build_jitter_pipeline(self):
        latency = max(0, int(self.jitter_latency_ms))
        drop_on_latency = self.gst_bool(self.jitter_drop_on_latency)
        return (
            "! rtpjitterbuffer latency=%d drop-on-latency=%s do-lost=true"
            % (latency, drop_on_latency)
        )

    def build_depay_pipeline(self):
        properties = []
        if self.depay_wait_for_keyframe and self.element_has_property(
            "rtph264depay",
            "wait-for-keyframe",
        ):
            properties.append("wait-for-keyframe=true")
        if self.depay_request_keyframe and self.element_has_property(
            "rtph264depay",
            "request-keyframe",
        ):
            properties.append("request-keyframe=true")

        property_text = ""
        if properties:
            property_text = " " + " ".join(properties)
        return "! rtph264depay%s" % property_text

    def build_h264_parser_pipeline(self):
        if self.element_has_property("h264parse", "config-interval"):
            return "! h264parse config-interval=%d" % self.h264_config_interval

        return "! h264parse"

    def build_decoder_pipeline(self, output_width, output_height):
        decoder = str(self.decoder).lower()
        if decoder not in ("nvidia", "jetson", "hardware"):
            raise RuntimeError(
                "Unsupported decoder '%s'. This node is configured to require the "
                "Jetson NVIDIA decoder; use decoder: nvidia." % self.decoder
            )

        nvidia_converter = self.nvidia_converter_element()
        if not self.has_gst_element("nvv4l2decoder"):
            raise RuntimeError("Required GStreamer element 'nvv4l2decoder' not found.")
        if nvidia_converter is None:
            raise RuntimeError(
                "Required NVIDIA GStreamer converter not found "
                "(expected nvvidconv or nvvideoconvert)."
            )

        self.get_logger().info("Using NVIDIA GStreamer H264 decoder")
        decoder_properties = []
        if self.element_has_property("nvv4l2decoder", "enable-max-performance"):
            decoder_properties.append("enable-max-performance=1")
        if self.disable_dpb and self.element_has_property(
            "nvv4l2decoder",
            "disable-dpb",
        ):
            decoder_properties.append("disable-dpb=true")

        decoder_property_text = ""
        if decoder_properties:
            decoder_property_text = " " + " ".join(decoder_properties)

        return (
            "! nvv4l2decoder%s "
            "! %s "
            "! video/x-raw,width=%d,height=%d,format=BGRx "
            "! videoconvert "
            "! video/x-raw,width=%d,height=%d,format=BGR"
        ) % (
            decoder_property_text,
            nvidia_converter,
            output_width,
            output_height,
            output_width,
            output_height,
        )

    def has_gst_element(self, name):
        return self.Gst.ElementFactory.find(name) is not None

    def element_has_property(self, element_name, property_name):
        factory = self.Gst.ElementFactory.find(element_name)
        if factory is None:
            return False

        element = factory.create(None)
        if element is None:
            return False

        return element.find_property(property_name) is not None

    def gst_bool(self, value):
        return "true" if value else "false"

    def nvidia_converter_element(self):
        for name in ("nvvidconv", "nvvideoconvert"):
            if self.has_gst_element(name):
                return name
        return None

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
                "GStreamer camera frame: width=%d height=%d encoding=bgr8 buffer=%d"
                % (width, height, len(map_info.data))
            )

        try:
            now = self.get_clock().now()
            stamp = self.get_clock().now().to_msg()
            wants_raw = (
                self.publish_raw
                and self.image_pub.get_subscription_count() > 0
                and self.should_publish(
                    now.nanoseconds * 1e-9,
                    self.last_raw_publish_time,
                    self.raw_max_fps,
                )
            )
            wants_compressed = (
                self.publish_compressed
                and self.compressed_pub.get_subscription_count() > 0
                and self.should_publish(
                    now.nanoseconds * 1e-9,
                    self.last_compressed_publish_time,
                    self.compressed_max_fps,
                )
            )

            if not wants_raw and not wants_compressed:
                return self.Gst.FlowReturn.OK

            frame_bytes = self.sample_to_tightly_packed_bgr(
                map_info,
                caps,
                width,
                height,
            )

            frame_bytes, width, height = self.resize_if_needed(
                frame_bytes,
                width,
                height,
            )

            if wants_raw:
                self.image_pub.publish(
                    self.bgr_bytes_to_image_msg(frame_bytes, width, height, stamp)
                )
                self.publish_camera_info(width, height, stamp)
                self.last_raw_publish_time = now.nanoseconds * 1e-9

            if wants_compressed:
                msg = self.bgr_bytes_to_compressed_msg(
                    frame_bytes,
                    width,
                    height,
                    stamp,
                )
                self.compressed_pub.publish(msg)
                self.last_compressed_publish_time = now.nanoseconds * 1e-9
        except Exception as exc:
            self.get_logger().error("Fatal camera bridge error: %s" % exc)
            self.pipeline.set_state(self.Gst.State.NULL)
            return self.Gst.FlowReturn.ERROR
        finally:
            buffer.unmap(map_info)

        return self.Gst.FlowReturn.OK

    def sample_to_tightly_packed_bgr(self, map_info, caps, width, height):
        channels = 3
        row_bytes = int(width * channels)
        expected_size = int(row_bytes * height)
        data = map_info.data

        if len(data) == expected_size:
            return bytes(data)

        stride = self.buffer_stride(caps)
        if stride >= row_bytes and len(data) >= stride * height:
            rows = []
            for row in range(height):
                start = row * stride
                rows.append(data[start : start + row_bytes])
            return b"".join(rows)

        raise RuntimeError(
            "Cannot repack GStreamer BGR frame safely: buffer=%d expected=%d "
            "stride=%d row_bytes=%d height=%d"
            % (len(data), expected_size, stride, row_bytes, height)
        )

    def buffer_stride(self, caps):
        try:
            info = self.GstVideo.VideoInfo.new_from_caps(caps)
            stride = int(info.stride[0])
            if stride > 0:
                return stride
        except Exception as exc:
            raise RuntimeError("Could not read GStreamer video stride: %s" % exc) from exc

        raise RuntimeError("GStreamer video stride was missing or invalid.")

    def should_publish(self, now_s, last_publish_s, max_fps):
        if max_fps <= 0.0:
            return True
        return now_s - last_publish_s >= 1.0 / max_fps

    def resize_if_needed(self, frame_bytes, width, height):
        if self.output_width <= 0 or self.output_height <= 0:
            return frame_bytes, width, height

        if width == self.output_width and height == self.output_height:
            return frame_bytes, width, height

        import numpy as np

        bgr = np.frombuffer(frame_bytes, dtype=np.uint8).reshape((height, width, 3))
        resized = self.cv2.resize(
            bgr,
            (self.output_width, self.output_height),
            interpolation=self.cv2.INTER_AREA,
        )
        return resized.tobytes(), self.output_width, self.output_height

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
        import numpy as np

        bgr = np.frombuffer(frame_bytes, dtype=np.uint8).reshape((height, width, 3))
        quality = max(1, min(100, self.jpeg_quality))
        ok, encoded = self.cv2.imencode(
            ".jpg",
            bgr,
            [int(self.cv2.IMWRITE_JPEG_QUALITY), quality],
        )
        if not ok:
            raise RuntimeError("OpenCV failed to JPEG-encode camera frame.")

        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.format = "jpeg"
        msg.data = encoded.tobytes()
        return msg

    def publish_camera_info(self, width, height, stamp):
        if self.camera_info_pub.get_subscription_count() == 0:
            return

        fx = self.focal_length_px(width, self.camera_horizontal_fov_deg)
        fy = fx
        cx = float(width) * 0.5
        cy = float(height) * 0.5

        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.height = int(height)
        msg.width = int(width)
        msg.distortion_model = "plumb_bob"
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.k = [
            fx,
            0.0,
            cx,
            0.0,
            fy,
            cy,
            0.0,
            0.0,
            1.0,
        ]
        msg.r = [
            1.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            1.0,
        ]
        msg.p = [
            fx,
            0.0,
            cx,
            0.0,
            0.0,
            fy,
            cy,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ]
        self.camera_info_pub.publish(msg)

    def focal_length_px(self, width, horizontal_fov_deg):
        half_fov = max(1.0, min(179.0, horizontal_fov_deg)) * 0.5
        return float(width) * 0.5 / math.tan(math.radians(half_fov))

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
