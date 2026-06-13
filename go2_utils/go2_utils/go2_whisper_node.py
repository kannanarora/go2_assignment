#!/usr/bin/env python3

"""
Decode the Go2 audio stream, denoise it with DeepFilterNet, and transcribe
it with Whisper.

Expected input:
  /audiosender  unitree_go/msg/AudioData

Expected Go2 audio format:
  Opus, 48 kHz, stereo, 20 ms packets, 960 samples per packet

Output:
  /go2/whisper/text  std_msgs/msg/String
"""

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
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from unitree_go.msg import AudioData


class DeepFilterDenoiser:
    """DeepFilterNet wrapper. Cleans mono 48 kHz audio in real time.
    """

    def __init__(self, atten_lim_db=None):
        from df.enhance import enhance, init_df

        self._enhance = enhance
        self.model, self.df_state, _ = init_df()
        self.sample_rate = self.df_state.sr()  # 48000
        self.atten_lim_db = atten_lim_db

    def enhance(self, mono_i16):
        audio = torch.from_numpy(mono_i16.astype(np.float32) / 32768.0).unsqueeze(0)
        cleaned = self._enhance(
            self.model, self.df_state, audio, atten_lim_db=self.atten_lim_db
        )
        return cleaned.squeeze(0).cpu().numpy()


class Go2WhisperNode(Node):
    def __init__(self):
        super().__init__("go2_whisper_node")

        self.declare_parameter("audio_topic", "/audiosender")
        self.declare_parameter("text_topic", "/go2/whisper/text")

        # Try "tiny.en" for fastest, "base.en" for better command recognition.
        self.declare_parameter("model_name", "base.en")
        self.declare_parameter("language", "en")

        # Low-latency command settings.
        self.declare_parameter("chunk_seconds", 1.0)
        self.declare_parameter("overlap_seconds", 0.15)
        self.declare_parameter("pending_chunks", 1)

        # Audio gates.
        self.declare_parameter("min_rms", 500)
        self.declare_parameter("min_text_length", 2)

        # Jetson CUDA works, but fp16 can produce NaNs on this setup.
        self.declare_parameter("fp16", False)

        # DeepFilterNet noise reduction.
        self.declare_parameter("enable_denoise", True)
        self.declare_parameter("atten_lim_db", 12.0)  # 0 means no limit
        self.declare_parameter("output_gain", 1.0)

        # Whisper command bias.
        self.declare_parameter(
            "initial_prompt",
            (
                "The robot only listens for short English voice commands: "
                "sit, sit down, stand, stand up, stop, lie down, hello, "
                "come here, come to me."
            ),
        )

        self.audio_topic = self.get_parameter("audio_topic").value
        self.text_topic = self.get_parameter("text_topic").value
        self.model_name = self.get_parameter("model_name").value
        self.language = self.get_parameter("language").value

        self.chunk_seconds = float(self.get_parameter("chunk_seconds").value)
        self.overlap_seconds = float(self.get_parameter("overlap_seconds").value)
        self.pending_chunks = int(self.get_parameter("pending_chunks").value)

        self.min_rms = int(self.get_parameter("min_rms").value)
        self.min_text_length = int(self.get_parameter("min_text_length").value)

        self.fp16 = bool(self.get_parameter("fp16").value)

        self.enable_denoise = bool(self.get_parameter("enable_denoise").value)
        atten = float(self.get_parameter("atten_lim_db").value)
        self.atten_lim_db = atten if atten > 0.0 else None
        self.output_gain = float(self.get_parameter("output_gain").value)

        self.initial_prompt = self.get_parameter("initial_prompt").value

        # Verified Go2 stream format.
        self.opus_rate = 48000
        self.opus_channels = 2
        self.opus_frame_size = 960  # 20 ms at 48 kHz
        self.whisper_rate = 16000

        self.decoder = opuslib.Decoder(self.opus_rate, self.opus_channels)

        self.pub = self.create_publisher(String, self.text_topic, 10)

        self.sub = self.create_subscription(
            AudioData,
            self.audio_topic,
            self.audio_callback,
            qos_profile_sensor_data,
        )

        self.lock = threading.Lock()
        self.buffer = bytearray()
        self.pending = deque(maxlen=max(1, self.pending_chunks))
        self.running = True
        self.last_text = ""

        # Buffer at 48 kHz so DeepFilterNet runs at its native rate, then
        # downsample to 16 kHz for Whisper after denoising.
        self.chunk_bytes = int(self.chunk_seconds * self.opus_rate * 2)
        self.overlap_bytes = int(self.overlap_seconds * self.opus_rate * 2)

        if self.overlap_bytes >= self.chunk_bytes:
            self.get_logger().warn(
                "overlap_seconds must be smaller than chunk_seconds; reducing overlap."
            )
            self.overlap_bytes = int(0.25 * self.chunk_bytes)

        device = "cuda" if torch.cuda.is_available() else "cpu"

        if device == "cpu" and self.fp16:
            self.get_logger().warn(
                "fp16=True requested, but device is CPU. Forcing fp16=False."
            )
            self.fp16 = False

        self.denoiser = None
        if self.enable_denoise:
            self.get_logger().info("Loading DeepFilterNet")
            self.denoiser = DeepFilterDenoiser(atten_lim_db=self.atten_lim_db)

        self.get_logger().info(
            "Loading Whisper model '%s' on %s" % (self.model_name, device)
        )

        self.model = whisper.load_model(self.model_name, device=device)

        self.worker = threading.Thread(target=self.worker_loop, daemon=True)
        self.worker.start()

        self.get_logger().info("Listening on %s" % self.audio_topic)
        self.get_logger().info("Publishing text on %s" % self.text_topic)
        self.get_logger().info(
            (
                "chunk_seconds=%.2f overlap_seconds=%.2f min_rms=%d "
                "pending_chunks=%d fp16=%s denoise=%s"
            )
            % (
                self.chunk_seconds,
                self.overlap_seconds,
                self.min_rms,
                self.pending_chunks,
                str(self.fp16),
                str(self.enable_denoise),
            )
        )

    def audio_callback(self, msg: AudioData):
        try:
            pcm48_stereo = self.decoder.decode(
                bytes(msg.data),
                self.opus_frame_size,
                decode_fec=False,
            )
        except Exception as exc:
            self.get_logger().warn("Opus decode failed: %s" % exc)
            return

        # Stereo 48 kHz int16 -> mono 48 kHz int16. Keep at 48 kHz so the
        # denoiser sees the full band; downsampling happens after enhancement.
        pcm48_mono = audioop.tomono(pcm48_stereo, 2, 0.5, 0.5)

        with self.lock:
            self.buffer.extend(pcm48_mono)

            if len(self.buffer) >= self.chunk_bytes:
                chunk = bytes(self.buffer[-self.chunk_bytes:])
                self.pending.append(chunk)

                if self.overlap_bytes > 0:
                    self.buffer = self.buffer[-self.overlap_bytes:]
                else:
                    self.buffer.clear()

    def resample_to_16k(self, mono_i16):
        # 48 kHz int16 -> 16 kHz int16. Each chunk is resampled independently.
        out, _ = audioop.ratecv(
            mono_i16.tobytes(),
            2,
            1,
            self.opus_rate,
            self.whisper_rate,
            None,
        )
        return np.frombuffer(out, dtype=np.int16)

    def worker_loop(self):
        while self.running:
            chunk = None

            with self.lock:
                if self.pending:
                    chunk = self.pending.popleft()

            if chunk is None:
                time.sleep(0.05)
                continue

            raw_rms = audioop.rms(chunk, 2)

            try:
                audio48_i16 = np.frombuffer(chunk, dtype=np.int16)

                if self.denoiser is not None:
                    cleaned48_f32 = self.denoiser.enhance(audio48_i16)
                    if self.output_gain != 1.0:
                        cleaned48_f32 = cleaned48_f32 * self.output_gain
                    cleaned48_i16 = np.clip(
                        cleaned48_f32 * 32768.0, -32768, 32767
                    ).astype(np.int16)
                else:
                    cleaned48_i16 = audio48_i16

                audio16_i16 = self.resample_to_16k(cleaned48_i16)
                audio_f32 = audio16_i16.astype(np.float32) / 32768.0

                clean_rms = audioop.rms(audio16_i16.tobytes(), 2)

                if raw_rms < self.min_rms and clean_rms < self.min_rms:
                    continue

                result = self.model.transcribe(
                    audio_f32,
                    language=self.language,
                    fp16=self.fp16,
                    verbose=False,
                    condition_on_previous_text=False,
                    temperature=0.0,
                    no_speech_threshold=0.75,
                    logprob_threshold=-0.8,
                    compression_ratio_threshold=2.4,
                    initial_prompt=self.initial_prompt,
                )

                text = result.get("text", "").strip()

                if len(text) < self.min_text_length:
                    continue

                # Avoid immediate duplicate chunks caused by overlap.
                if text == self.last_text:
                    continue

                self.last_text = text

                msg = String()
                msg.data = text
                self.pub.publish(msg)

                self.get_logger().info("Heard: %s" % text)

            except Exception as exc:
                self.get_logger().error("Whisper failed: %s" % exc)

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
