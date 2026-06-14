#!/usr/bin/env python3

"""
Map Whisper transcriptions to robot behaviour

Bridges the speech-to-text output to the behaviour commands

  /go2/whisper/text  (std_msgs/String, free text e.g. "sit down")
    -> keyword match -> token e.g. "sit"
  /trigger_behaviour (std_msgs/String, token consumed by sport_client_wrapper_node)

Keyword matching only (no fuzzy matching yet).
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# handled by sport_client_wrapper_node.
COMMAND_RULES = [
    (("turn left",), "turn_left"),
    (("turn right",), "turn_right"),
    (("lie down", "lay down", "lie", "lay"), "lie_down"),
    (("sit",), "sit"),
    (("stand", "get up"), "stand"),
    (("stop", "halt", "freeze"), "stop"),
    (("hello", "hey", "wave"), "hello"),
    (("come", "walk", "forward"), "walk"),
]


def match_command(text):
    # Return the behaviour token for a transcription, or None
    normalized = "".join(c if c.isalnum() or c.isspace() else " " for c in text)
    normalized = " ".join(normalized.lower().split())
    for keywords, token in COMMAND_RULES:
        for kw in keywords:
            if kw in normalized:
                return token
    return None


class VoiceCommandMapperNode(Node):
    def __init__(self):
        super().__init__("voice_command_mapper_node")

        self.declare_parameter("text_topic", "/go2/whisper/text")
        self.declare_parameter("trigger_topic", "/trigger_behaviour")
        self.declare_parameter("cooldown_sec", 2.0)

        self.text_topic = self.get_parameter("text_topic").value
        self.trigger_topic = self.get_parameter("trigger_topic").value
        self.cooldown_sec = float(self.get_parameter("cooldown_sec").value)

        self.last_fire = 0.0

        self.pub = self.create_publisher(String, self.trigger_topic, 10)
        self.sub = self.create_subscription(
            String,
            self.text_topic,
            self.text_callback,
            10,
        )

        self.get_logger().info(
            "Mapping %s -> %s (cooldown=%.1fs)"
            % (self.text_topic, self.trigger_topic, self.cooldown_sec)
        )

    def text_callback(self, msg: String):
        text = msg.data.strip()
        token = match_command(text)

        if token is None:
            self.get_logger().info("No command in '%s' (ignored)" % text)
            return

        now = time.monotonic()
        if now - self.last_fire < self.cooldown_sec:
            self.get_logger().info("Cooldown, dropping '%s' -> %s" % (text, token))
            return
        self.last_fire = now

        out = String()
        out.data = token
        self.pub.publish(out)
        self.get_logger().info("'%s' -> %s" % (text, token))


def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandMapperNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
