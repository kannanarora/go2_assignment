"""
publishes a WAV file as Opus-encoded frames to /audioreceiver

An experimental approach that was confirmed to play audio through
AudioHub megaphone mode before streaming so the speaker accepts
frames reliably. The node reads a 48 kHz mono WAV file,
encodes each 20 ms frame with Opus, and publishes to /audioreceiver.
"""

import json
import os
import time
import wave

import opuslib
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from unitree_api.msg import Request
from unitree_go.msg import AudioData

TOPIC = '/audioreceiver'
RATE = 48000
CHANNELS = 1
FRAME_SIZE = 960   # 20 ms at 48 kHz

ENTER_MEGAPHONE = 4001
EXIT_MEGAPHONE  = 4002

class AudioPlayerNode(Node):
    def __init__(self):
        super().__init__('audio_player_node')

        default_wav = os.path.join(
            get_package_share_directory('go2_behaviours'), 'sounds', 'go2_bark.wav'
        )
        self.declare_parameter('wav_file', default_wav)
        self._wav_file = self.get_parameter('wav_file').value

        self._audio_pub = self.create_publisher(AudioData, TOPIC, 10)
        self._hub_pub   = self.create_publisher(Request, '/api/audiohub/request', 10)
        self._encoder   = opuslib.Encoder(RATE, CHANNELS, opuslib.APPLICATION_AUDIO)

        # delay so publishers have time to connect before sending
        self.create_timer(1.0, self._play_once)
        self._played = False

        self.get_logger().info(f'AudioPlayerNode ready, will play: {self._wav_file}')

    def _send_hub(self, api_id):
        req = Request()
        req.header.identity.api_id = api_id
        req.parameter = json.dumps({})
        self._hub_pub.publish(req)

    def _play_once(self):
        if self._played:
            return
        self._played = True

        wav_file = self._wav_file

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

                self.get_logger().info('Entering megaphone mode...')
                self._send_hub(ENTER_MEGAPHONE)
                time.sleep(0.5)

                self.get_logger().info(f'Playing {wav_file}...')
                frame_num = 0

                while True:
                    pcm = wf.readframes(FRAME_SIZE)
                    if len(pcm) < FRAME_SIZE * 2:   # 2 bytes per int16 sample
                        break
                    msg = AudioData()
                    msg.time_frame = frame_num
                    msg.data       = list(self._encoder.encode(pcm, FRAME_SIZE))
                    self._audio_pub.publish(msg)
                    frame_num += 1
                    time.sleep(0.02)   # pace at 20 ms per frame

        except FileNotFoundError:
            self.get_logger().error(f'File not found: {wav_file}')
            return

        self.get_logger().info('Playback complete. Exiting megaphone mode...')
        self._send_hub(EXIT_MEGAPHONE)


def main(args=None):
    rclpy.init(args=args)
    node = AudioPlayerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
