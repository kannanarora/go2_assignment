#!/usr/bin/env python3

"""
Play AudioHub sounds by trigger token

A generic, reusable replacement for bark_node: instead of one hardcoded
sound it takes a token -> file_name map and plays the matching clip when
that token arrives on /trigger_behaviour.

    sound_map: ["bark:go2_bark", "speak:go2_bark", "meow:go2_meow"]

Each file_name must already be in AudioHub (upload it with
audiohub_player_node). UUIDs are resolved once at startup.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from go2_utils.audiohub_client import AudioHubClient


class SoundPlayerNode(Node):
    def __init__(self):
        super().__init__("sound_player_node")

        trigger_topic = self.declare_parameter(
            "trigger_topic", "/trigger_behaviour"
        ).value
        sound_map = self.declare_parameter(
            "sound_map", ["bark:go2_bark", "speak:go2_bark"]
        ).value

        self._client = AudioHubClient(self)

        # token -> file_name, then resolve each unique file_name to a UUID once.
        self._token_to_file = self._parse_sound_map(sound_map)
        self._token_to_uuid = self._resolve_uuids(self._token_to_file)

        self.create_subscription(String, trigger_topic, self._on_trigger, 10)
        self.get_logger().info(
            "Listening for %s on %s"
            % (sorted(self._token_to_uuid), trigger_topic)
        )

    def _parse_sound_map(self, entries):
        mapping = {}
        for entry in entries:
            token, sep, file_name = entry.partition(":")
            token = token.strip().lower()
            file_name = file_name.strip()
            if not sep or not token or not file_name:
                self.get_logger().warn("Ignoring malformed sound_map entry: '%s'" % entry)
                continue
            mapping[token] = file_name
        return mapping

    def _resolve_uuids(self, token_to_file):
        # Resolve each distinct file_name once, then fan out to tokens.
        file_to_uuid = {}
        for file_name in set(token_to_file.values()):
            uuid = self._client.resolve_uuid(file_name)
            if uuid:
                file_to_uuid[file_name] = uuid
                self.get_logger().info("Ready: '%s' (uuid=%s)" % (file_name, uuid))
            else:
                self.get_logger().warn(
                    "'%s' not in AudioHub - upload it first with audiohub_player_node"
                    % file_name
                )

        return {
            token: file_to_uuid[file_name]
            for token, file_name in token_to_file.items()
            if file_name in file_to_uuid
        }

    def _on_trigger(self, msg: String):
        token = msg.data.strip().lower()
        uuid = self._token_to_uuid.get(token)
        if uuid is None:
            return
        self._client.play(uuid)
        self.get_logger().info("Played '%s'" % token)


def main(args=None):
    rclpy.init(args=args)
    node = SoundPlayerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
