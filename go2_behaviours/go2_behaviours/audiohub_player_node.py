"""
Uploads a WAV file via the AudioHub API then plays it

Unlike publishing to /audioreceiver approach, this uploads the file into the robot's
AudioHub storage, retrieves its UUID, and triggers playback via the API.

    1. Read WAV file, base64-encode, split into 4 KB chunks
    2. Upload all chunks via api_id 2001 (UPLOAD_AUDIO_FILE)
    3. Fetch audio list via api_id 1001 (GET_AUDIO_LIST) to get the UUID
    4. Play by UUID via api_id 1002 (SELECT_START_PLAY)
"""

import base64
import hashlib
import json
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from unitree_api.msg import Request, Response

UPLOAD_AUDIO_FILE = 2001
GET_AUDIO_LIST = 1001
SELECT_START_PLAY = 1002

CHUNK_SIZE  = 4096   # bytes of base64 per upload chunk
CHUNK_INTERVAL = 0.05   # seconds between chunk publishes (~20 Hz)


class AudioHubPlayerNode(Node):
    def __init__(self):
        super().__init__('audiohub_player_node')

        self.declare_parameter('wav_file',  '/home/unitree/bark.wav')
        self.declare_parameter('file_name', 'bark')

        self._wav_file  = self.get_parameter('wav_file').value
        self._file_name = self.get_parameter('file_name').value

        self._pub = self.create_publisher(Request,  '/api/audiohub/request',  10)
        self._sub = self.create_subscription(
            Response, '/api/audiohub/response', self._on_response, 10
        )

        self._response = None        # type: Optional[Response]
        self._last_api_id = None        # type: Optional[int]
        self._response_lock = threading.Lock()

        self.get_logger().info(
            f'AudioHubPlayerNode ready — wav: {self._wav_file}, name: {self._file_name}'
        )

        # Run the multi-step upload to play flow in a background thread
        # so rclpy.spin() stays free to process incoming responses
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    # Response callback

    def _on_response(self, msg):
        with self._response_lock:
            self._response    = msg
            self._last_api_id = msg.header.identity.api_id

    # Helpers

    def _publish(self, api_id, params):
        # publish to /api/audiohub/request
        req = Request()
        req.header.identity.api_id = api_id
        req.parameter = json.dumps(params)
        self._pub.publish(req)

    def _clear_response(self):
        with self._response_lock:
            self._response    = None
            self._last_api_id = None

    def _wait_for_response(self, api_id, timeout=5.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._response_lock:
                if self._last_api_id == api_id:
                    return self._response
            time.sleep(0.05)
        return None

    # Main flow (runs in background thread)
    def _run(self):
        # time to connect
        time.sleep(1.0)

        # Read and encode the WAV file
        try:
            with open(self._wav_file, 'rb') as f:
                audio_data = f.read()
        except FileNotFoundError:
            self.get_logger().error(f'WAV file not found: {self._wav_file}')
            return

        file_md5 = hashlib.md5(audio_data).hexdigest()
        b64_data = base64.b64encode(audio_data).decode('utf-8')
        chunks   = [b64_data[i:i + CHUNK_SIZE] for i in range(0, len(b64_data), CHUNK_SIZE)]
        total    = len(chunks)

        self.get_logger().info(
            f'Uploading {len(audio_data)} bytes ({total} chunks)...'
        )

        # Upload all chunks 
        for i, chunk in enumerate(chunks, 1):
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
            time.sleep(CHUNK_INTERVAL)
            if i % 10 == 0:
                self.get_logger().info(f'  chunk {i}/{total}')

        self.get_logger().info('Upload complete. Fetching audio list...')
        time.sleep(0.5)

        # Get audio list ie. find UUID
        self._clear_response()
        self._publish(GET_AUDIO_LIST, {})

        resp = self._wait_for_response(GET_AUDIO_LIST, timeout=5.0)

        file_uuid = None
        if resp and resp.data:
            try:
                audio_list = json.loads(resp.data).get('audio_list', [])
                match = next(
                    (a for a in audio_list if a.get('CUSTOM_NAME') == self._file_name),
                    None
                )
                if match:
                    file_uuid = match['UNIQUE_ID']
                    self.get_logger().info(f'Found UUID: {file_uuid}')
                else:
                    self.get_logger().error(
                        f'"{self._file_name}" not found in audio list. '
                        f'Available: {[a.get("CUSTOM_NAME") for a in audio_list]}'
                    )
            except Exception as e:
                self.get_logger().error(f'Parse error: {e} | raw: {resp.data}')
        else:
            self.get_logger().error(
                'No response to GET_AUDIO_LIST within timeout. '
                'Is the AudioHub service running?'
            )

        # Play by UUID
        if file_uuid:
            self._publish(SELECT_START_PLAY, {'unique_id': file_uuid})
            self.get_logger().info('Play command sent.')
        else:
            self.get_logger().error('Playback aborted — no UUID found.')


def main(args=None):
    rclpy.init(args=args)
    node = AudioHubPlayerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
