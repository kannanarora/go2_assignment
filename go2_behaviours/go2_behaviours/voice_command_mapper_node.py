#!/usr/bin/env python3

"""
Map Whisper transcriptions to robot behaviour ie. bridges the speech-to-text output to the behaviour commands

  /go2/whisper/text  (std_msgs/String, free text ex. "sit down")-> keyword match (then fuzzy fallback) -> token ex. "sit"
      /requested_behaviour (std_msgs/String, token consumed by behaviour_executor_node)

Matching is keyword-first, if no keyword is found, a conservative fuzzy
fallback rescues near-misses on short words (e.g. "six"/"sid" -> "sit").
"""

import time
from difflib import SequenceMatcher

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


COMMAND_RULES = [
    (("turn left",), "turn_left"),
    (("turn right",), "turn_right"),
    (("lie down", "lay down", "lie", "lay"), "lie_down"),
    (("sit", "six", "sid", "sick", "shit"), "sit"),
    (("stand", "get up"), "stand"),
    (("stop", "halt", "freeze"), "stop"),
    (("hello", "hey", "wave"), "hello"),
    (("dance", "dancing"), "dance"),
    (("come", "walk", "forward"), "walk"),
]

# Single word keywords used for the fuzzy fallback (phrases are skipped - they only ever match exactly)
FUZZY_KEYWORDS = [
    (kw, token)
    for keywords, token in COMMAND_RULES
    for kw in keywords
    if " " not in kw
]


def _normalize(text):
    cleaned = "".join(c if c.isalnum() or c.isspace() else " " for c in text)
    return " ".join(cleaned.lower().split())


def match_command(text, fuzzy_threshold=0.8, fuzzy_margin=0.1):
    """Match a transcription to a behaviour token, with reasoning

    Keyword (substring) match first. If nothing matches, fall back to the
    closest single-word keyword by character similarity, but only accept it
    when it clears fuzzy_threshold AND beats the runner-up token by
    fuzzy_margin (so ambiguous matches are rejected rather than guessed)
    """
    normalized = _normalize(text)
    if not normalized:
        return None, "empty"

    for keywords, token in COMMAND_RULES:
        for kw in keywords:
            if kw in normalized:
                return token, "keyword '%s'" % kw

    best_score = {}
    best_pair = {}
    for word in normalized.split():
        # Short words (e.g. "it", "go") are too close to short commands and
        # cause false positives, so don't fuzzy-match them.
        if len(word) < 3:
            continue
        for kw, token in FUZZY_KEYWORDS:
            score = SequenceMatcher(None, word, kw).ratio()
            if score > best_score.get(token, 0.0):
                best_score[token] = score
                best_pair[token] = (word, kw)

    if not best_score:
        return None, "no words"

    ranked = sorted(best_score.items(), key=lambda kv: kv[1], reverse=True)
    top_token, top_score = ranked[0]
    runner_up = ranked[1][1] if len(ranked) > 1 else 0.0
    word, kw = best_pair[top_token]

    if top_score >= fuzzy_threshold and top_score - runner_up >= fuzzy_margin:
        return top_token, "fuzzy '%s'~'%s' %.2f" % (word, kw, top_score)
    if top_score < fuzzy_threshold:
        return None, "best '%s'~%s %.2f < %.2f" % (
            word, top_token, top_score, fuzzy_threshold
        )
    return None, "ambiguous '%s' %s %.2f vs %.2f" % (
        word, top_token, top_score, runner_up
    )


class VoiceCommandMapperNode(Node):
    def __init__(self):
        super().__init__("voice_command_mapper_node")

        self.declare_parameter("text_topic", "/go2/whisper/text")
        self.declare_parameter("requested_behaviour_topic", "/requested_behaviour")
        self.declare_parameter("cooldown_sec", 2.0)
        self.declare_parameter("fuzzy_threshold", 0.8)

        self.text_topic = self.get_parameter("text_topic").value
        self.requested_topic = self.get_parameter(
            "requested_behaviour_topic").value
        self.cooldown_sec = float(self.get_parameter("cooldown_sec").value)
        self.fuzzy_threshold = float(self.get_parameter("fuzzy_threshold").value)

        self.last_fire = 0.0

        self.pub = self.create_publisher(String, self.requested_topic, 10)
        self.sub = self.create_subscription(
            String,
            self.text_topic,
            self.text_callback,
            10,
        )

        self.get_logger().info(
            "Mapping %s -> %s (cooldown=%.1fs)"
            % (self.text_topic, self.requested_topic, self.cooldown_sec)
        )

    def text_callback(self, msg: String):
        text = msg.data.strip()
        token, detail = match_command(text, fuzzy_threshold=self.fuzzy_threshold)

        if token is None:
            self.get_logger().info("ignored '%s' (%s)" % (text, detail))
            return

        now = time.monotonic()
        if now - self.last_fire < self.cooldown_sec:
            self.get_logger().info(
                "cooldown, dropping '%s' -> %s [%s]" % (text, token, detail)
            )
            return
        self.last_fire = now

        out = String()
        out.data = token
        self.pub.publish(out)
        self.get_logger().info("'%s' -> %s [%s]" % (text, token, detail))


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
