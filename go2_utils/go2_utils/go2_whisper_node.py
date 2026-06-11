#!/usr/bin/env python3

import audioop
import threading
import time
from collections import deque

import numpy as np
import opuslib
import rclpy
import torch
import whisper
from rclpy.node import Node
from std_msgs.msg import String
from unitree_go.msg import AudioData


class Go2WhisperNode(Node):
    def __init__(self):
        super().__init__("go2_whisper_node")

        self.declare_parameter("audio_topic", "/audiosender")
        self.declare_parameter("text_topic", "/go2/whisper/text")
        self.declare_parameter("model_name", "tiny.en")
        self.declare_parameter("language", "en")
        self.declare_parameter("chunk_seconds", 3.0)
        self.declare_parameter("overlap_seconds", 0.5)
        self.declare_parameter("min_rms", 250)
        self.declare_parameter("min_text_length", 2)

        self.audio_topic = self.get_parameter("audio_topic").value
        self.text_topic = self.get_parameter("text_topic").value
        self.model_name = self.get_parameter("model_name").value
        self.language = self.get_parameter("language").value
        self.chunk_seconds = float(self.get_parameter("chunk_seconds").value)
        self.overlap_seconds = float(self.get_parameter("overlap_seconds").value)
        self.min_rms = int(self.get_parameter("min_rms").value)
        self.min_text_length = int(self.get_parameter("min_text_length").value)

        # Verified from your Go2 stream:
        # Opus, 48 kHz, stereo, 20 ms packets, 960 samples per packet.
        self.opus_rate = 48000
        self.opus_channels = 2
        self.opus_frame_size = 960
        self.whisper_rate = 16000

        self.decoder = opuslib.Decoder(self.opus_rate, self.opus_channels)

        self.pub = self.create_publisher(String, self.text_topic, 10)
        self.sub = self.create_subscription(
            AudioData,
            self.audio_topic,
            self.audio_callback,
            20,
        )

        self.lock = threading.Lock()
        self.buffer = bytearray()
        self.pending = deque()
        self.running = True
        self.last_text = ""

        self.chunk_bytes = int(self.chunk_seconds * self.whisper_rate * 2)
        self.overlap_bytes = int(self.overlap_seconds * self.whisper_rate * 2)

        device = "cuda" if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f"Loading Whisper model '{self.model_name}' on {device}")
        self.model = whisper.load_model(self.model_name, device=device)
        self.fp16 = device == "cuda"

        self.worker = threading.Thread(target=self.worker_loop, daemon=True)
        self.worker.start()

        self.get_logger().info(f"Listening on {self.audio_topic}")
        self.get_logger().info(f"Publishing text on {self.text_topic}")

    def audio_callback(self, msg: AudioData):
        try:
            pcm48_stereo = self.decoder.decode(
                bytes(msg.data),
                self.opus_frame_size,
                decode_fec=False,
            )
        except Exception as exc:
            self.get_logger().warn(f"Opus decode failed: {exc}")
            return

        # Stereo 48 kHz int16 -> mono 48 kHz int16
        pcm48_mono = audioop.tomono(pcm48_stereo, 2, 0.5, 0.5)

        # Gentle high-pass-ish cleanup is intentionally omitted here.
        # Whisper generally does better with clean resampling than heavy filtering.

        # Mono 48 kHz int16 -> mono 16 kHz int16
        pcm16_mono, _ = audioop.ratecv(
            pcm48_mono,
            2,
            1,
            self.opus_rate,
            self.whisper_rate,
            None,
        )

        with self.lock:
            self.buffer.extend(pcm16_mono)

            if len(self.buffer) >= self.chunk_bytes:
                chunk = bytes(self.buffer[-self.chunk_bytes:])
                self.pending.append(chunk)

                if self.overlap_bytes > 0:
                    self.buffer = self.buffer[-self.overlap_bytes:]
                else:
                    self.buffer.clear()

    def worker_loop(self):
        while self.running:
            chunk = None

            with self.lock:
                if self.pending:
                    chunk = self.pending.popleft()

            if chunk is None:
                time.sleep(0.05)
                continue

            rms = audioop.rms(chunk, 2)
            if rms < self.min_rms:
                continue

            try:
                audio_i16 = np.frombuffer(chunk, dtype=np.int16)
                audio_f32 = audio_i16.astype(np.float32) / 32768.0

                result = self.model.transcribe(
                    audio_f32,
                    language=self.language,
                    fp16=self.fp16,
                    verbose=False,
                    condition_on_previous_text=False,
                    no_speech_threshold=0.6,
                )

                text = result.get("text", "").strip()

                if len(text) < self.min_text_length:
                    continue

                if text == self.last_text:
                    continue

                self.last_text = text

                msg = String()
                msg.data = text
                self.pub.publish(msg)
                self.get_logger().info(f"Heard: {text}")

            except Exception as exc:
                self.get_logger().error(f"Whisper failed: {exc}")

    def destroy_node(self):
        self.running = False
        if hasattr(self, "worker") and self.worker.is_alive():
            self.worker.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Go2WhisperNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()