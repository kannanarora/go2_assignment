#!/usr/bin/env python3

import wave

import numpy as np
import opuslib
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from unitree_go.msg import AudioData

from go2_behaviours.audio_utils import rms_level, stereo_to_mono
from go2_behaviours.denoise import NoiseReducer


class Go2AudioDumpNode(Node):
    def __init__(self):
        super().__init__("go2_audio_dump_node")

        self.declare_parameter("audio_topic", "/audiosender")
        self.declare_parameter("output_path", "/tmp/go2_audio_raw.wav")
        self.declare_parameter("record_seconds", 10.0)
        self.declare_parameter("channel_mode", "stereo")  # stereo, left, right, mono
        self.declare_parameter("noise_reduce", False)

        self.audio_topic    = self.get_parameter("audio_topic").value
        self.output_path    = self.get_parameter("output_path").value
        self.record_seconds = float(self.get_parameter("record_seconds").value)
        self.channel_mode   = self.get_parameter("channel_mode").value
        self.noise_reduce   = self.get_parameter("noise_reduce").value

        self.opus_rate       = 48000
        self.opus_channels   = 2
        self.opus_frame_size = 960

        self.decoder = opuslib.Decoder(self.opus_rate, self.opus_channels)

        self.frames_written = 0
        self.max_frames     = int(self.record_seconds * self.opus_rate)
        self.frame_count    = 0

        self.denoiser = NoiseReducer() if self.noise_reduce else None

        channels = 2 if self.channel_mode == "stereo" else 1

        self.wav = wave.open(self.output_path, "wb")
        self.wav.setnchannels(channels)
        self.wav.setsampwidth(2)
        self.wav.setframerate(self.opus_rate)

        self.sub = self.create_subscription(
            AudioData,
            self.audio_topic,
            self.audio_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            "Recording %s to %s for %.1f seconds"
            % (self.audio_topic, self.output_path, self.record_seconds)
        )
        self.get_logger().info(
            "channel_mode=%s noise_reduce=%s" % (self.channel_mode, self.noise_reduce)
        )

    def audio_callback(self, msg):
        try:
            pcm_bytes = self.decoder.decode(
                bytes(msg.data),
                self.opus_frame_size,
                decode_fec=False,
            )
        except Exception as exc:
            self.get_logger().warn("Opus decode failed: %s" % exc)
            return

        self.frame_count += 1

        stereo = np.frombuffer(pcm_bytes, dtype=np.int16)

        if self.channel_mode == "stereo":
            out    = pcm_bytes
            frames = len(stereo) // 2
        elif self.channel_mode == "left":
            out    = stereo[0::2].tobytes()
            frames = len(stereo) // 2
        elif self.channel_mode == "right":
            out    = stereo[1::2].tobytes()
            frames = len(stereo) // 2
        else:
            mono   = stereo_to_mono(stereo)
            out    = mono.tobytes()
            frames = len(mono)

        if self.frame_count == 1:
            mono_for_rms = stereo_to_mono(stereo) if self.channel_mode == "stereo" else np.frombuffer(out, dtype=np.int16)
            self.get_logger().info("First frame RMS: %d" % rms_level(mono_for_rms))

        if self.denoiser is not None and self.channel_mode != "stereo":
            mono_np = np.frombuffer(out, dtype=np.int16)
            out = self.denoiser.process(mono_np).tobytes()
            if self.denoiser.learning:
                self.get_logger().info("Learning noise profile...", throttle_duration_sec=1.0)
                return

        self.wav.writeframes(out)
        self.frames_written += frames

        if self.frames_written >= self.max_frames:
            self.get_logger().info("Done recording: %s" % self.output_path)
            self.wav.close()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = Go2AudioDumpNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.wav.close()
        except Exception:
            pass
        node.destroy_node()


if __name__ == "__main__":
    main()
