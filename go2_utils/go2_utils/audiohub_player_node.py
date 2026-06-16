"""
Uploads a WAV file into AudioHub then plays it once.

A thin provisioning/smoke-test wrapper around AudioHubClient: it makes
sure a clip exists in the robot's AudioHub storage (uploading if needed)
and plays it once to confirm. Runtime playback lives in sound_player_node.
"""

import os
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from go2_utils.audiohub_client import AudioHubClient


class AudioHubPlayerNode(Node):
    def __init__(self):
        super().__init__('audiohub_player_node')

        default_wav = os.path.join(
            get_package_share_directory('go2_utils'), 'sounds', 'bark.wav'
        )
        self._wav_file = self.declare_parameter('wav_file', default_wav).value
        self._file_name = self.declare_parameter('file_name', 'go2_bark').value

        self._client = AudioHubClient(self)

    def run(self):
        time.sleep(1.0)

        file_uuid = self._client.upload(self._wav_file, self._file_name)
        if file_uuid:
            self._client.play(file_uuid)
            self.get_logger().info('Play command sent.')
        else:
            self.get_logger().error('Playback aborted, no UUID found.')


def main(args=None):
    rclpy.init(args=args)
    node = AudioHubPlayerNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
