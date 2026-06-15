#!/usr/bin/env python3

"""
Same pipeline as the original go2_whisper_node, but transcribing with
faster-whisper (CTranslate2) instead of openai-whisper. Used to test
whether the openai-whisper NaN ("!!!!") problem is engine-specific.

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
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from unitree_go.msg import AudioData


class Go2FasterWhisperTestNode(Node):
    def __init__(self):
        super().__init__("go2_faster_whisper_test_node")

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

        # faster-whisper engine settings.
        self.declare_parameter("device", "")        # "" = auto, or "cpu" / "cuda"
        self.declare_parameter("compute_type", "")   # "" = auto
        self.declare_parameter("beam_size", 5)

        # Simple fan/noise reduction.
        self.declare_parameter("enable_denoise", True)
        self.declare_parameter("noise_learn_chunks", 3)
        self.declare_parameter("noise_reduce_strength", 1.2)
        self.declare_parameter("noise_floor", 0.08)
        self.declare_parameter("highpass_hz", 120.0)
        self.declare_parameter("lowpass_hz", 4200.0)

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

        self.device_param = self.get_parameter("device").value
        self.compute_type = self.get_parameter("compute_type").value
        self.beam_size = int(self.get_parameter("beam_size").value)

        self.enable_denoise = bool(self.get_parameter("enable_denoise").value)
        self.noise_learn_chunks = int(self.get_parameter("noise_learn_chunks").value)
        self.noise_reduce_strength = float(
            self.get_parameter("noise_reduce_strength").value
        )
        self.noise_floor = float(self.get_parameter("noise_floor").value)
        self.highpass_hz = float(self.get_parameter("highpass_hz").value)
        self.lowpass_hz = float(self.get_parameter("lowpass_hz").value)

        self.initial_prompt = self.get_parameter("initial_prompt").value

        # Verified Go2 stream format.
        self.opus_rate = 48000
        self.opus_channels = 2
        self.opus_frame_size = 960  # 20 ms at 48 kHz
        self.whisper_rate = 16000

        self.decoder = opuslib.Decoder(self.opus_rate, self.opus_channels)

        # audioop.ratecv needs this state preserved between packets.
        self.ratecv_state = None

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

        self.noise_mag = None
        self.noise_chunks_seen = 0

        self.chunk_bytes = int(self.chunk_seconds * self.whisper_rate * 2)
        self.overlap_bytes = int(self.overlap_seconds * self.whisper_rate * 2)

        if self.overlap_bytes >= self.chunk_bytes:
            self.get_logger().warn(
                "overlap_seconds must be smaller than chunk_seconds; reducing overlap."
            )
            self.overlap_bytes = int(0.25 * self.chunk_bytes)

        import ctranslate2
        from faster_whisper import WhisperModel

        cuda_ok = ctranslate2.get_cuda_device_count() > 0
        device = self.device_param or ("cuda" if cuda_ok else "cpu")
        compute_type = self.compute_type or (
            "float16" if device == "cuda" else "int8"
        )

        self.get_logger().info(
            "Loading faster-whisper '%s' on %s (%s)"
            % (self.model_name, device, compute_type)
        )
        self.model = WhisperModel(
            self.model_name, device=device, compute_type=compute_type
        )

        self.worker = threading.Thread(target=self.worker_loop, daemon=True)
        self.worker.start()

        self.get_logger().info("Listening on %s" % self.audio_topic)
        self.get_logger().info("Publishing text on %s" % self.text_topic)
        self.get_logger().info(
            "chunk_seconds=%.2f overlap_seconds=%.2f min_rms=%d denoise=%s"
            % (
                self.chunk_seconds,
                self.overlap_seconds,
                self.min_rms,
                str(self.enable_denoise),
            )
        )

        if self.enable_denoise and self.noise_learn_chunks > 0:
            self.get_logger().info(
                "Stay quiet for the first %d audio chunks so I can learn fan noise."
                % self.noise_learn_chunks
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

        # Stereo 48 kHz int16 -> mono 48 kHz int16
        pcm48_mono = audioop.tomono(
            pcm48_stereo,
            2,
            0.5,
            0.5,
        )

        # Mono 48 kHz int16 -> mono 16 kHz int16.
        # Keep ratecv_state between packets to avoid resampling artifacts.
        pcm16_mono, self.ratecv_state = audioop.ratecv(
            pcm48_mono,
            2,
            1,
            self.opus_rate,
            self.whisper_rate,
            self.ratecv_state,
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

    def bandpass_filter(self, audio_f32):
        if len(audio_f32) == 0:
            return audio_f32

        spectrum = np.fft.rfft(audio_f32)
        freqs = np.fft.rfftfreq(len(audio_f32), d=1.0 / self.whisper_rate)

        mask = (freqs >= self.highpass_hz) & (freqs <= self.lowpass_hz)
        spectrum *= mask

        filtered = np.fft.irfft(spectrum, n=len(audio_f32)).astype(np.float32)
        return np.clip(filtered, -1.0, 1.0)

    def spectral_denoise(self, audio_f32):
        if len(audio_f32) == 0:
            return audio_f32, False

        spectrum = np.fft.rfft(audio_f32)
        mag = np.abs(spectrum)
        phase = np.angle(spectrum)

        # Learn the fan/noise profile from the first few chunks.
        if self.noise_chunks_seen < self.noise_learn_chunks:
            if self.noise_mag is None:
                self.noise_mag = mag.copy()
            else:
                self.noise_mag = 0.8 * self.noise_mag + 0.2 * mag

            self.noise_chunks_seen += 1
            self.get_logger().info(
                "Learning noise profile chunk %d/%d"
                % (self.noise_chunks_seen, self.noise_learn_chunks)
            )
            return audio_f32, True

        if self.noise_mag is None:
            return audio_f32, False

        reduced_mag = mag - self.noise_reduce_strength * self.noise_mag
        reduced_mag = np.maximum(reduced_mag, self.noise_floor * mag)

        cleaned_spectrum = reduced_mag * np.exp(1j * phase)
        cleaned = np.fft.irfft(cleaned_spectrum, n=len(audio_f32)).astype(np.float32)

        peak = np.max(np.abs(cleaned))
        if peak > 1.0:
            cleaned = cleaned / peak

        return np.clip(cleaned, -1.0, 1.0), False

    def preprocess_audio(self, audio_f32):
        if not self.enable_denoise:
            return audio_f32, False

        audio_f32 = self.bandpass_filter(audio_f32)
        audio_f32, learning_noise = self.spectral_denoise(audio_f32)
        return audio_f32, learning_noise

    def transcribe(self, audio_f32):
        segments, _ = self.model.transcribe(
            audio_f32,
            language=self.language,
            beam_size=self.beam_size,
            condition_on_previous_text=False,
            temperature=0.0,
            no_speech_threshold=0.75,
            log_prob_threshold=-0.8,
            compression_ratio_threshold=2.4,
            initial_prompt=self.initial_prompt,
        )
        return " ".join(s.text for s in segments).strip()

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
                audio_i16 = np.frombuffer(chunk, dtype=np.int16)
                audio_f32 = audio_i16.astype(np.float32) / 32768.0

                audio_f32, learning_noise = self.preprocess_audio(audio_f32)

                # Do not transcribe while learning the fan profile.
                if learning_noise:
                    continue

                # Re-check RMS after filtering/denoising.
                denoised_i16 = np.clip(audio_f32 * 32768.0, -32768, 32767).astype(
                    np.int16
                )
                clean_rms = audioop.rms(denoised_i16.tobytes(), 2)

                if raw_rms < self.min_rms and clean_rms < self.min_rms:
                    continue

                text = self.transcribe(audio_f32)

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
    node = Go2FasterWhisperTestNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
