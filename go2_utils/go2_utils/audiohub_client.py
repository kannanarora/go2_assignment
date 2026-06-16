"""
Thin client for the Go2 AudioHub API

Wraps the request/response plumbing shared by the audio nodes (upload,
list, play) so the protocol lives in one place instead of being copied
into every node that touches sound

it creates the pub/sub on that node and drives replies via rclpy.spin_once on that node

    client = AudioHubClient(self)
    uuid = client.upload("/path/to/sound.wav", "my_sound")   # idempotent
    client.play(uuid)
    uuid = client.resolve_uuid("my_sound")    # name -> uuid
"""

import base64
import hashlib
import json
import time
import wave

import rclpy
from unitree_api.msg import Request, Response

UPLOAD_AUDIO_FILE = 2001
GET_AUDIO_LIST = 1001
SELECT_START_PLAY = 1002

CHUNK_SIZE = 4096  # bytes of base64 per upload chunk


class AudioHubClient:
    def __init__(self, node):
        self._node = node
        self._log = node.get_logger()

        self._pub = node.create_publisher(Request, "/api/audiohub/request", 10)
        self._sub = node.create_subscription(
            Response, "/api/audiohub/response", self._on_response, 10
        )

        self._response = None
        self._last_api = None

    def _on_response(self, msg):
        self._response = msg
        self._last_api = msg.header.identity.api_id

    def _spin_until(self, api_id, timeout=5.0):
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self._node, timeout_sec=0.1)
            if self._last_api == api_id:
                return True
        return False

    def _publish(self, api_id, params):
        req = Request()
        req.header.identity.api_id = api_id
        req.parameter = json.dumps(params)
        self._pub.publish(req)

    def resolve_uuid(self, file_name):
        # Return the AudioHub UUID for file_name, or None if not present
        self._response = None
        self._last_api = None
        self._publish(GET_AUDIO_LIST, {})

        if not self._spin_until(GET_AUDIO_LIST, timeout=5.0):
            self._log.error("No response to GET_AUDIO_LIST, is AudioHub running?")
            return None

        try:
            payload = json.loads(self._response.data) if self._response.data else {}
            audio_list = payload.get("audio_list", [])
            match = next(
                (a for a in audio_list if a.get("CUSTOM_NAME") == file_name), None
            )
            return match.get("UNIQUE_ID") if match else None
        except Exception as exc:
            self._log.error("Parse error: %s" % exc)
            return None

    def play(self, uuid):
        # Start playback of a clip by UUID
        self._publish(SELECT_START_PLAY, {"unique_id": uuid})

    def upload(self, wav_path, file_name):
        """Upload a WAV into AudioHub and return its UUID (None on failure)

        Idempotent: if file_name already exists in AudioHub the existing
        UUID is returned without re-uploading.
        """
        existing = self.resolve_uuid(file_name)
        if existing:
            self._log.info(
                "'%s' already in AudioHub (uuid=%s), skipping upload."
                % (file_name, existing)
            )
            return existing

        try:
            with wave.open(wav_path, "rb") as wf:
                rate = wf.getframerate()
                channels = wf.getnchannels()
                width = wf.getsampwidth()
            self._log.info(
                "WAV info: rate=%d Hz, channels=%d, sampwidth=%d bytes (%s)"
                % (rate, channels, width,
                   "16-bit" if width == 2 else "%d-bit" % (width * 8))
            )
            if width != 2:
                self._log.warn("WAV is not 16-bit PCM, AudioHub may reject it")
        except FileNotFoundError:
            self._log.error("WAV file not found: %s" % wav_path)
            return None

        with open(wav_path, "rb") as f:
            audio_data = f.read()

        file_md5 = hashlib.md5(audio_data).hexdigest()
        b64_data = base64.b64encode(audio_data).decode("utf-8")
        chunks = [
            b64_data[i:i + CHUNK_SIZE] for i in range(0, len(b64_data), CHUNK_SIZE)
        ]
        total = len(chunks)

        self._log.info(
            "Uploading %d bytes (%d chunks)..." % (len(audio_data), total)
        )
        for i, chunk in enumerate(chunks, 1):
            self._response = None
            self._last_api = None
            self._publish(UPLOAD_AUDIO_FILE, {
                "file_name": file_name,
                "file_type": "wav",
                "file_size": len(audio_data),
                "current_block_index": i,
                "total_block_number": total,
                "block_content": chunk,
                "current_block_size": len(chunk),
                "file_md5": file_md5,
                "create_time": int(time.time() * 1000),
            })
            # wait for per-chunk ACK, falls back to 0.5s if no ACK
            self._spin_until(UPLOAD_AUDIO_FILE, timeout=0.5)
            if i % 10 == 0 or i == total:
                self._log.info("  chunk %d/%d" % (i, total))

        self._log.info("Upload complete. Fetching audio list...")
        time.sleep(0.5)
        return self.resolve_uuid(file_name)
