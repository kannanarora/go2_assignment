#!/usr/bin/env python3

import asyncio
import base64
import binascii
import hashlib
import json
import os
import threading
import time
import uuid

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image


class WebRtcCameraNode(Node):
    def __init__(self):
        super().__init__("webrtc_camera_node")

        self.robot_ip = self.declare_parameter(
            "robot_ip", os.getenv("ROBOT_IP", "")
        ).value
        self.token = self.declare_parameter("token", os.getenv("ROBOT_TOKEN", "")).value
        self.output_topic = self.declare_parameter(
            "output_topic", "/camera/color/image"
        ).value
        self.compressed_output_topic = self.declare_parameter(
            "compressed_output_topic", "/camera/color/image/compressed"
        ).value
        self.frame_id = self.declare_parameter("frame_id", "front_camera").value
        self.publish_raw = bool(self.declare_parameter("publish_raw", True).value)
        self.publish_compressed = bool(
            self.declare_parameter("publish_compressed", True).value
        )
        self.raw_max_fps = float(self.declare_parameter("raw_max_fps", 10.0).value)
        self.compressed_max_fps = float(
            self.declare_parameter("compressed_max_fps", 5.0).value
        )
        self.jpeg_quality = int(self.declare_parameter("jpeg_quality", 70).value)

        self.image_pub = self.create_publisher(Image, self.output_topic, 10)
        self.compressed_pub = self.create_publisher(
            CompressedImage,
            self.compressed_output_topic,
            10,
        )

        self._cv2 = None
        if self.publish_compressed:
            try:
                import cv2

                self._cv2 = cv2
            except ImportError:
                self.get_logger().warn(
                    "Compressed image output requires python3-opencv."
                )

        self._last_raw_publish_time = 0.0
        self._last_compressed_publish_time = 0.0
        self._logged_first_frame = False
        self._thread = None
        self._loop = None
        self._client = None

        if not self.robot_ip:
            raise RuntimeError(
                "webrtc_camera_node needs robot_ip parameter or ROBOT_IP env var."
            )

        self.start_webrtc_thread()

    def start_webrtc_thread(self):
        self._thread = threading.Thread(target=self.run_webrtc_loop, daemon=True)
        self._thread.start()

    def run_webrtc_loop(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._client = Go2WebRtcCameraClient(
            robot_ip=self.robot_ip,
            token=self.token,
            frame_callback=self.publish_bgr_frame,
            log=self.get_logger(),
        )
        try:
            self._loop.run_until_complete(self._client.run())
        except Exception as exc:
            self.get_logger().error("WebRTC camera stopped: %s" % exc)

    def publish_bgr_frame(self, bgr):
        now = time.monotonic()
        stamp = self.get_clock().now().to_msg()

        if not self._logged_first_frame:
            self._logged_first_frame = True
            self.get_logger().info(
                "WebRTC camera frame: width=%d height=%d encoding=bgr8"
                % (bgr.shape[1], bgr.shape[0])
            )

        if self.publish_raw and self.should_publish(
            now,
            self._last_raw_publish_time,
            self.raw_max_fps,
        ):
            self.image_pub.publish(self.bgr_to_image_msg(bgr, stamp))
            self._last_raw_publish_time = now

        if self.publish_compressed and self.should_publish(
            now,
            self._last_compressed_publish_time,
            self.compressed_max_fps,
        ):
            msg = self.bgr_to_compressed_msg(bgr, stamp)
            if msg is not None:
                self.compressed_pub.publish(msg)
                self._last_compressed_publish_time = now

    def should_publish(self, now: float, last_publish_time: float, max_fps: float):
        if max_fps <= 0.0:
            return True
        return now - last_publish_time >= 1.0 / max_fps

    def bgr_to_image_msg(self, bgr, stamp):
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

    def destroy_node(self):
        if self._client is not None and self._loop is not None:
            try:
                asyncio.run_coroutine_threadsafe(self._client.close(), self._loop)
            except Exception:
                pass
        return super().destroy_node()


class Go2WebRtcCameraClient:
    def __init__(self, robot_ip: str, token: str, frame_callback, log):
        self.robot_ip = robot_ip
        self.token = token
        self.frame_callback = frame_callback
        self.log = log
        self.pc = None
        self.data_channel = None
        self.validation_result = "PENDING"
        self._video_request_count = 0

    async def run(self):
        try:
            from aiortc import RTCPeerConnection, RTCSessionDescription
        except ImportError as exc:
            raise RuntimeError(
                "webrtc_camera_node requires aiortc. Install it with pip."
            ) from exc

        self.RTCSessionDescription = RTCSessionDescription
        self.pc = RTCPeerConnection()
        self.data_channel = self.pc.createDataChannel("data", id=0)
        self.data_channel.on("open", self.on_data_channel_open)
        self.data_channel.on("message", self.on_data_channel_message)
        self.pc.on("track", self.on_track)
        self.pc.on("connectionstatechange", self.on_connection_state_change)
        self.pc.on("iceconnectionstatechange", self.on_ice_connection_state_change)
        self.pc.on("signalingstatechange", self.on_signaling_state_change)
        self.pc.addTransceiver("video", direction="recvonly")

        await self.connect()
        while self.pc.connectionState not in ("closed", "failed"):
            if self.pc.connectionState == "connected":
                self.request_video_if_possible()
            await asyncio.sleep(0.2)

    def on_connection_state_change(self):
        self.log.info("WebRTC connection state: %s" % self.pc.connectionState)

    def on_ice_connection_state_change(self):
        self.log.info("WebRTC ICE state: %s" % self.pc.iceConnectionState)

    def on_signaling_state_change(self):
        self.log.info("WebRTC signaling state: %s" % self.pc.signalingState)

    def on_data_channel_open(self):
        if self.data_channel.readyState != "open":
            self.data_channel._setReadyState("open")
        self.log.info("WebRTC data channel open")
        self.request_video()
        self.disable_traffic_saving(True)

    def on_data_channel_message(self, message):
        if not isinstance(message, str):
            return

        try:
            msg = json.loads(message)
        except json.JSONDecodeError:
            return

        if msg.get("type") == "validation":
            self.validate_robot_conn(msg)

    async def on_track(self, track):
        if track.kind != "video":
            return

        self.log.info("Receiving WebRTC video track")
        while True:
            try:
                frame = await asyncio.wait_for(track.recv(), timeout=5.0)
                self.frame_callback(frame.to_ndarray(format="bgr24"))
                await asyncio.sleep(0)
            except asyncio.TimeoutError:
                self.log.warn(
                    "WebRTC video track is open, but no frame arrived for 5s; "
                    "requesting video again"
                )
                self.request_video_if_possible()
            except Exception as exc:
                self.log.error("WebRTC video frame error: %s" % exc)
                break

    def validate_robot_conn(self, message):
        data = message.get("data", "")
        if data == "Validation Ok.":
            self.validation_result = "SUCCESS"
            self.log.info("Robot WebRTC validation successful")
            self.request_video()
            self.disable_traffic_saving(True)
            return

        self.log.info("Received WebRTC validation challenge")
        encrypted_key = self.encrypt_validation_key(data)
        self.publish("", encrypted_key, "validation")

    def request_video(self):
        self.publish("", "on", "vid")
        self._video_request_count += 1
        self.log.info("Requested WebRTC video")

    def request_video_if_possible(self):
        if self.data_channel is None or self.data_channel.readyState != "open":
            return

        if self._video_request_count >= 5:
            return

        self.request_video()

    def disable_traffic_saving(self, enabled: bool):
        data = {
            "req_type": "disable_traffic_saving",
            "instruction": "on" if enabled else "off",
        }
        self.publish("", data, "rtc_inner_req")
        self.log.info("Requested disable_traffic_saving=%s" % enabled)

    def publish(self, topic, data, msg_type="msg"):
        if self.data_channel is None:
            return
        if self.data_channel.readyState != "open":
            self.log.warn(
                "WebRTC data channel is %s, sending anyway" % self.data_channel.readyState
            )
            self.data_channel._setReadyState("open")

        payload = {"type": msg_type, "topic": topic, "data": data}
        self.data_channel.send(json.dumps(payload))

    async def connect(self):
        offer = await self.pc.createOffer()
        await self.pc.setLocalDescription(offer)
        sdp_offer = {
            "id": "STA_localNetwork",
            "sdp": self.pc.localDescription.sdp,
            "type": self.pc.localDescription.type,
            "token": self.token,
        }

        data1, data2 = self.fetch_robot_public_key()
        if data2 == 2:
            data1 = self.decrypt_con_notify_data(data1)

        public_key_pem = data1[10 : len(data1) - 10]
        path_ending = self.calc_local_path_ending(data1)
        aes_key = self.generate_aes_key()
        encrypted_body = {
            "data1": self.aes_encrypt(json.dumps(sdp_offer), aes_key),
            "data2": self.rsa_encrypt(aes_key, public_key_pem),
        }
        encrypted_answer = self.post_encrypted_sdp(path_ending, encrypted_body)
        answer_json = json.loads(self.aes_decrypt(encrypted_answer, aes_key))
        answer = self.RTCSessionDescription(
            sdp=answer_json["sdp"],
            type=answer_json["type"],
        )
        await self.pc.setRemoteDescription(answer)
        self.log.info("WebRTC camera connection established")

    def fetch_robot_public_key(self):
        import requests

        response = requests.post("http://%s:9991/con_notify" % self.robot_ip, timeout=10)
        response.raise_for_status()
        decoded = base64.b64decode(response.text).decode("utf-8")
        data = json.loads(decoded)
        return data["data1"], data.get("data2")

    def post_encrypted_sdp(self, path_ending, encrypted_body):
        import requests

        response = requests.post(
            "http://%s:9991/con_ing_%s" % (self.robot_ip, path_ending),
            data=json.dumps(encrypted_body),
            headers={"Content-Type": "application/x-www-form-urlencoded"},
            timeout=10,
        )
        response.raise_for_status()
        return response.text

    def decrypt_con_notify_data(self, encrypted_b64):
        try:
            from cryptography.hazmat.primitives.ciphers.aead import AESGCM
        except ImportError as exc:
            raise RuntimeError(
                "Firmware encrypted WebRTC signaling requires python3-cryptography."
            ) from exc

        key = bytes([232, 86, 130, 189, 22, 84, 155, 0, 142, 4, 166, 104, 43, 179, 235, 227])
        data = base64.b64decode(encrypted_b64)
        tag = data[-16:]
        nonce = data[-28:-16]
        ciphertext = data[:-28]
        return AESGCM(key).decrypt(nonce, ciphertext + tag, None).decode("utf-8")

    def generate_aes_key(self):
        return binascii.hexlify(uuid.uuid4().bytes).decode("utf-8")

    def aes_encrypt(self, data, key):
        from Crypto.Cipher import AES

        block_size = AES.block_size
        padding = block_size - len(data) % block_size
        padded = (data + chr(padding) * padding).encode("utf-8")
        encrypted = AES.new(key.encode("utf-8"), AES.MODE_ECB).encrypt(padded)
        return base64.b64encode(encrypted).decode("utf-8")

    def aes_decrypt(self, encrypted_data, key):
        from Crypto.Cipher import AES

        raw = base64.b64decode(encrypted_data)
        padded = AES.new(key.encode("utf-8"), AES.MODE_ECB).decrypt(raw)
        padding = padded[-1]
        return padded[:-padding].decode("utf-8")

    def rsa_encrypt(self, data, public_key_pem):
        from Crypto.Cipher import PKCS1_v1_5
        from Crypto.PublicKey import RSA

        key = RSA.import_key(base64.b64decode(public_key_pem))
        cipher = PKCS1_v1_5.new(key)
        max_chunk_size = key.size_in_bytes() - 11
        encrypted = bytearray()
        data_bytes = data.encode("utf-8")
        for i in range(0, len(data_bytes), max_chunk_size):
            encrypted.extend(cipher.encrypt(data_bytes[i : i + max_chunk_size]))
        return base64.b64encode(encrypted).decode("utf-8")

    def encrypt_validation_key(self, key):
        digest = hashlib.md5(("UnitreeGo2_%s" % key).encode("utf-8")).hexdigest()
        return base64.b64encode(bytes.fromhex(digest)).decode("utf-8")

    def calc_local_path_ending(self, data1):
        alphabet = ["A", "B", "C", "D", "E", "F", "G", "H", "I", "J"]
        chunks = [data1[-10:][i : i + 2] for i in range(0, 10, 2)]
        values = []
        for chunk in chunks:
            if len(chunk) > 1 and chunk[1] in alphabet:
                values.append(str(alphabet.index(chunk[1])))
        return "".join(values)

    async def close(self):
        if self.pc is not None:
            await self.pc.close()


def main(args=None):
    rclpy.init(args=args)
    node = WebRtcCameraNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
