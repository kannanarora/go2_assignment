"""
Uploads a WAV file via the AudioHub API then plays it.

Unlike the /audioreceiver approach, this uploads the file into the robot's
AudioHub storage, retrieves its UUID, and triggers playback via the API.

    1. Read WAV file, base64-encode, split into 4 KB chunks
    2. Upload all chunks via api_id 2001 (UPLOAD_AUDIO_FILE)
    3. Fetch audio list via api_id 1001 (GET_AUDIO_LIST) to get the UUID
    4. Play by UUID via api_id 1002 (SELECT_START_PLAY)
"""

import base64
import hashlib
import json
import os
import time
import wave

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from unitree_api.msg import Request, Response

UPLOAD_AUDIO_FILE = 2001
GET_AUDIO_LIST    = 1001
SELECT_START_PLAY = 1002

CHUNK_SIZE = 4096  # bytes of base64 per upload chunk


class AudioHubPlayerNode(Node):
    def __init__(self):
        super().__init__('audiohub_player_node')

        default_wav = os.path.join(
            get_package_share_directory('go2_utils'), 'sounds', 'go2_bark.wav'
        )
        self.declare_parameter('wav_file',  default_wav)
        self.declare_parameter('file_name', 'go2_bark')

        self._wav_file  = self.get_parameter('wav_file').value
        self._file_name = self.get_parameter('file_name').value

        self._pub = self.create_publisher(Request,  '/api/audiohub/request',  10)
        self._sub = self.create_subscription(
            Response, '/api/audiohub/response', self._on_response, 10
        )

        self.response = None
        self.last_api = None

    def _on_response(self, msg):
        self.response = msg
        self.last_api = msg.header.identity.api_id

    def _spin_until(self, api_id, timeout=5.0):
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.last_api == api_id:
                return True
        return False

    def _publish(self, api_id, params):
        req = Request()
        req.header.identity.api_id = api_id
        req.parameter = json.dumps(params)
        self._pub.publish(req)

    def run(self):
        time.sleep(1.0)

        # Read WAV and log format info for debugging
        try:
            with wave.open(self._wav_file, 'rb') as wf:
                rate     = wf.getframerate()
                channels = wf.getnchannels()
                width    = wf.getsampwidth()
            self.get_logger().info(
                f'WAV info: rate={rate} Hz, channels={channels}, '
                f'sampwidth={width} bytes ({"16-bit" if width == 2 else str(width*8)+"-bit"})'
            )
            if width != 2:
                self.get_logger().warn('WAV is not 16-bit PCM, AudioHub may reject it')
        except FileNotFoundError:
            self.get_logger().error(f'WAV file not found: {self._wav_file}')
            return

        # Check if file already exists in AudioHub
        file_uuid = self._get_existing_uuid()

        if file_uuid:
            self.get_logger().info(f'Found existing UUID: {file_uuid}, skipping upload.')
        else:
            self.get_logger().info('File not in AudioHub, uploading...')
            file_uuid = self._upload_and_get_uuid()

        # Play by UUID
        if file_uuid:
            self._publish(SELECT_START_PLAY, {'unique_id': file_uuid})
            self.get_logger().info('Play command sent.')
        else:
            self.get_logger().error('Playback aborted, no UUID found.')

    def _get_existing_uuid(self):
        self.response = None
        self.last_api = None
        self._publish(GET_AUDIO_LIST, {})

        if not self._spin_until(GET_AUDIO_LIST, timeout=5.0):
            self.get_logger().error('No response to GET_AUDIO_LIST, is AudioHub running?')
            return None

        try:
            payload    = json.loads(self.response.data) if self.response.data else {}
            audio_list = payload.get('audio_list', [])
            match      = next(
                (a for a in audio_list if a.get('CUSTOM_NAME') == self._file_name), None
            )
            return match.get('UNIQUE_ID') if match else None
        except Exception as e:
            self.get_logger().error(f'Parse error: {e} | raw: {self.response.data}')
            return None

    def _upload_and_get_uuid(self):
        with open(self._wav_file, 'rb') as f:
            audio_data = f.read()

        file_md5 = hashlib.md5(audio_data).hexdigest()
        b64_data = base64.b64encode(audio_data).decode('utf-8')
        chunks   = [b64_data[i:i + CHUNK_SIZE] for i in range(0, len(b64_data), CHUNK_SIZE)]
        total    = len(chunks)

        self.get_logger().info(f'Uploading {len(audio_data)} bytes ({total} chunks)...')

        for i, chunk in enumerate(chunks, 1):
            self.response = None
            self.last_api = None

            self._publish(UPLOAD_AUDIO_FILE, {
                'file_name':           self._file_name,
                'file_type':           'wav',
                'file_size':           len(audio_data),
                'current_block_index': i,
                'total_block_number':  total,
                'block_content':       chunk,
                'current_block_size':  len(chunk),
                'file_md5':            file_md5,
                'create_time':         int(time.time() * 1000),
            })
            # wait for per-chunk ACK, falls back to 0.5s if no ACK
            self._spin_until(UPLOAD_AUDIO_FILE, timeout=0.5)

            if i % 10 == 0 or i == total:
                self.get_logger().info(f'  chunk {i}/{total}')

        self.get_logger().info('Upload complete. Fetching audio list...')
        time.sleep(0.5)

        return self._get_existing_uuid()


def main(args=None):
    rclpy.init(args=args)
    node = AudioHubPlayerNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
