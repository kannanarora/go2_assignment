"""
publishes a WAV file as Opus-encoded frames to /audioreceiver

An experimental approach that was confirmed to play audio through
the Go2 speaker on the first attempt. The node reads a 48 kHz mono WAV file,
encodes each 20 ms frame with Opus, and publishes to /audioreceiver.

makesure you have /home/unitree/bark.wav
"""

import wave
import time

import opuslib
import rclpy
from rclpy.node import Node
from unitree_go.msg import AudioData

TOPIC      = '/audioreceiver'
RATE       = 48000
CHANNELS   = 1
FRAME_SIZE = 960   # 20 ms at 48 kHz


class AudioPlayerNode(Node):
    def __init__(self):
        super().__init__('audio_player_node')

        self.declare_parameter('wav_file', '/home/unitree/bark.wav')
        self._wav_file = self.get_parameter('wav_file').value

        self._pub     = self.create_publisher(AudioData, TOPIC, 10)
        self._encoder = opuslib.Encoder(RATE, CHANNELS, opuslib.APPLICATION_AUDIO)

        # small delay so the publisher has time to connect to the subscriber
        self.create_timer(1.0, self._play_once)
        self._played = False

        self.get_logger().info(
            f'AudioPlayerNode ready — will play: {self._wav_file}'
        )

    def _play_once(self):
        if self._played:
            return
        self._played = True

        wav_file = self._wav_file
        self.get_logger().info(f'Opening {wav_file}')

        try:
            with wave.open(wav_file, 'rb') as wf:
                if wf.getframerate() != RATE:
                    self.get_logger().error(
                        f'WAV must be {RATE} Hz, got {wf.getframerate()} Hz. '
                        f'Convert with: ffmpeg -i {wav_file} -ar {RATE} -ac 1 out.wav'
                    )
                    return
                if wf.getnchannels() != CHANNELS:
                    self.get_logger().error(
                        f'WAV must be mono, got {wf.getnchannels()} channels.'
                    )
                    return

                self.get_logger().info(f'Playing {wav_file}...')
                frame_num = 0

                while True:
                    pcm = wf.readframes(FRAME_SIZE)
                    if len(pcm) < FRAME_SIZE * 2:   # 2 bytes per int16 sample
                        break

                    msg = AudioData()
                    msg.time_frame = frame_num
                    msg.data       = list(self._encoder.encode(pcm, FRAME_SIZE))
                    self._pub.publish(msg)
                    frame_num += 1
                    time.sleep(0.02)   # pace at 20 ms per frame

        except FileNotFoundError:
            self.get_logger().error(f'File not found: {wav_file}')
            return

        self.get_logger().info('Playback complete.')


def main(args=None):
    rclpy.init(args=args)
    node = AudioPlayerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
