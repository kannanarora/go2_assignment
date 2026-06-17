#!/usr/bin/env python3

"""
Play the dog bark on command

Listens on /trigger_behaviour for "bark" / "speak" and plays the bark
sound from the robot's AudioHub (uploaded by audiohub_player_node)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from go2_utils.audiohub_client import AudioHubClient

BARK_TOKENS = ("bark", "speak")


class BarkNode(Node):
    def __init__(self):
        super().__init__("bark_node")

        self.file_name = self.declare_parameter("file_name", "bark").value
        trigger_topic = self.declare_parameter(
            "trigger_topic", "/trigger_behaviour"
        ).value

        self._client = AudioHubClient(self)

        # Resolve the bark's UUID once at startup.
        self.bark_uuid = self._client.resolve_uuid(self.file_name)
        if self.bark_uuid:
            self.get_logger().info("Bark ready (uuid=%s)" % self.bark_uuid)
        else:
            self.get_logger().warn(
                "'%s' not in AudioHub - upload it first with audiohub_player_node"
                % self.file_name
            )

        self.create_subscription(String, trigger_topic, self._on_trigger, 10)
        self.get_logger().info("Listening for bark/speak on %s" % trigger_topic)

    def _on_trigger(self, msg: String):
        if msg.data.strip().lower() not in BARK_TOKENS:
            return
        if self.bark_uuid is None:
            self.get_logger().warn("Bark requested but no UUID; is it uploaded?")
            return
        self._client.play(self.bark_uuid)
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
