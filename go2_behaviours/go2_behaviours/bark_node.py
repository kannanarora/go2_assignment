#!/usr/bin/env python3

"""
Play the dog bark on command

Listens on /trigger_behaviour for "bark" / "speak" and plays the go2_bark
sound from the robot's AudioHub (uploaded by audiohub_player_node)
"""

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from unitree_api.msg import Request, Response

GET_AUDIO_LIST = 1001
SELECT_START_PLAY = 1002

BARK_TOKENS = ("bark", "speak")


class BarkNode(Node):
    def __init__(self):
        super().__init__("bark_node")

        self.declare_parameter("file_name", "go2_bark")
        self.declare_parameter("trigger_topic", "/trigger_behaviour")
        self.file_name = self.get_parameter("file_name").value
        trigger_topic = self.get_parameter("trigger_topic").value

        self._pub = self.create_publisher(Request, "/api/audiohub/request", 10)
        self._sub = self.create_subscription(
            Response, "/api/audiohub/response", self._on_response, 10
        )
        self.response = None
        self.last_api = None

        # Resolve the bark's UUID once at startup.
        self.bark_uuid = self._resolve_uuid()
        if self.bark_uuid:
            self.get_logger().info("Bark ready (uuid=%s)" % self.bark_uuid)
        else:
            self.get_logger().warn(
                "'%s' not in AudioHub - upload it first with audiohub_player_node"
                % self.file_name
            )

        self.create_subscription(String, trigger_topic, self._on_trigger, 10)
        self.get_logger().info("Listening for bark/speak on %s" % trigger_topic)

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

    def _resolve_uuid(self):
        self.response = None
        self.last_api = None
        self._publish(GET_AUDIO_LIST, {})
        if not self._spin_until(GET_AUDIO_LIST, timeout=5.0):
            self.get_logger().error("No response to GET_AUDIO_LIST")
            return None
        try:
            payload = json.loads(self.response.data) if self.response.data else {}
            audio_list = payload.get("audio_list", [])
            match = next(
                (a for a in audio_list if a.get("CUSTOM_NAME") == self.file_name), None
            )
            return match.get("UNIQUE_ID") if match else None
        except Exception as exc:
            self.get_logger().error("Parse error: %s" % exc)
            return None

    def _on_trigger(self, msg: String):
        if msg.data.strip().lower() not in BARK_TOKENS:
            return
        if self.bark_uuid is None:
            self.get_logger().warn("Bark requested but no UUID; is it uploaded?")
            return
        self._publish(SELECT_START_PLAY, {"unique_id": self.bark_uuid})
        self.get_logger().info("Bark!")


def main(args=None):
    rclpy.init(args=args)
    node = BarkNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
