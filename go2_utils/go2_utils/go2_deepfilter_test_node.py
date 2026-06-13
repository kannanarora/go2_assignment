#!/usr/bin/env python3

"""
Record a brief clip from the Go2 mic, denoise the whole clip with
DeepFilterNet, and save raw and cleaned WAVs side by side for comparison.

Expected input:
  /audiosender  unitree_go/msg/AudioData  (Opus, 48 kHz, stereo, 960/packet)
"""

import audioop
import wave

import numpy as np
import opuslib
import rclpy
import torch
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from unitree_go.msg import AudioData


class Go2DeepFilterTestNode(Node):
    def __init__(self):
        super().__init__("go2_deepfilter_test_node")

        self.declare_parameter("audio_topic", "/audiosender")
        self.declare_parameter("output_path", "/tmp/go2_deepfilter_clean.wav")
        self.declare_parameter("raw_output_path", "/tmp/go2_deepfilter_raw.wav")
        self.declare_parameter("record_seconds", 6.0)

        self.audio_topic = self.get_parameter("audio_topic").value
        self.output_path = self.get_parameter("output_path").value
        self.raw_output_path = self.get_parameter("raw_output_path").value
        self.record_seconds = float(self.get_parameter("record_seconds").value)

        self.opus_rate = 48000
        self.opus_channels = 2
        self.opus_frame_size = 960

        self.decoder = opuslib.Decoder(self.opus_rate, self.opus_channels)

        self.frames = []
        self.samples_collected = 0
        self.max_samples = int(self.record_seconds * self.opus_rate)
        self.done = False

        self.get_logger().info("Loading DeepFilterNet")
        from df.enhance import enhance, init_df

        self._enhance = enhance
        self.model, self.df_state, _ = init_df()

        self.sub = self.create_subscription(
            AudioData,
            self.audio_topic,
            self.audio_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            "Recording %s for %.1f seconds, then denoising"
            % (self.audio_topic, self.record_seconds)
        )

    def audio_callback(self, msg):
        if self.done:
            return

        try:
            pcm48_stereo = self.decoder.decode(
                bytes(msg.data),
                self.opus_frame_size,
                decode_fec=False,
            )
        except Exception as exc:
            self.get_logger().warn("Opus decode failed: %s" % exc)
            return

        pcm48_mono = audioop.tomono(pcm48_stereo, 2, 0.5, 0.5)
        mono = np.frombuffer(pcm48_mono, dtype=np.int16)

        self.frames.append(mono)
        self.samples_collected += len(mono)

        if self.samples_collected >= self.max_samples:
            self.done = True
            self.finish()

    def finish(self):
        raw = np.concatenate(self.frames)
        self.save_wav(self.raw_output_path, raw)
        self.get_logger().info("Saved raw recording: %s" % self.raw_output_path)

        self.get_logger().info("Running DeepFilterNet on the whole clip")
        audio = torch.from_numpy(raw.astype(np.float32) / 32768.0).unsqueeze(0)
        cleaned = self._enhance(self.model, self.df_state, audio)
        cleaned = cleaned.squeeze(0).cpu().numpy()
        cleaned_i16 = np.clip(cleaned * 32768.0, -32768, 32767).astype(np.int16)

        self.save_wav(self.output_path, cleaned_i16)
        self.get_logger().info("Saved denoised recording: %s" % self.output_path)

        rclpy.shutdown()

    def save_wav(self, path, samples_i16):
        with wave.open(path, "wb") as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(self.opus_rate)
            wf.writeframes(samples_i16.tobytes())


def main(args=None):
    rclpy.init(args=args)
    node = Go2DeepFilterTestNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
