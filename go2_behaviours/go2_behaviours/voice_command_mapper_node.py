#!/usr/bin/env python3

"""
Map Whisper transcriptions to subsumption voice commands.

  /go2/whisper/text  (std_msgs/String, free text e.g. "sit down")
      -> keyword match (then fuzzy fallback) -> token e.g. "sit"
      -> Go2Command on /trick_cmd or String on /approach_trigger

Trick tokens are forwarded by the MUX to /trigger_behaviour (sport client,
sound_player). Approach tokens activate approach_person_node, which then
publishes /approach_cmd. Voice commands do not publish movement commands.
"""

import time
from difflib import SequenceMatcher

import rclpy
from go2_interfaces.msg import Go2Command
from rclpy.node import Node
from std_msgs.msg import String


COMMAND_RULES = [
    (("come here", "come", "byte", "bite"), "approach_person"),
    (("lie down", "lay down", "lie", "lay"), "lie_down"),
    (("sit", "six", "sid", "sick", "shit", "down", "fit"), "sit"),
    (("stand", "get up", "stand up", "up"), "stand"),
    (("stop", "halt", "freeze"), "stop"),
    (("hello", "hey", "wave"), "hello"),
    (("dance", "dancing"), "dance"),
    (("bark", "bye", "mark", "arc"), "bark"),
]

TRICK_TOKENS = {"sit", "stand", "lie_down", "stop", "hello", "bark", "dance"}
APPROACH_TOKENS = {"approach_person"}

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


def _contains_keyword(normalized, keyword):
    words = normalized.split()
    keyword_words = keyword.split()
    if not keyword_words:
        return False

    n = len(keyword_words)
    return any(words[i : i + n] == keyword_words for i in range(len(words) - n + 1))


def match_command(text, fuzzy_threshold=0.8, fuzzy_margin=0.1):
    """Match a transcription to a behaviour token, with reasoning."""
    normalized = _normalize(text)
    if not normalized:
        return None, "empty"

    for keywords, token in COMMAND_RULES:
        for kw in keywords:
            if _contains_keyword(normalized, kw):
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


def build_go2_command(token):
    cmd = Go2Command()
    if token in TRICK_TOKENS:
        cmd.command_type = Go2Command.TRICK
        cmd.trick_name = token
        return cmd

    return None


class VoiceCommandMapperNode(Node):
    def __init__(self):
        super().__init__("voice_command_mapper_node")

        self.declare_parameter("text_topic", "/go2/whisper/text")
        self.declare_parameter("command_topic", "/trick_cmd")
        self.declare_parameter("approach_trigger_topic", "/approach_trigger")
        self.declare_parameter("cooldown_sec", 2.0)
        self.declare_parameter("fuzzy_threshold", 0.8)
        self.declare_parameter("trick_hold_sec", 0.6)
        self.declare_parameter("command_rate_hz", 10.0)

        self.text_topic = self.get_parameter("text_topic").value
        self.command_topic = self.get_parameter("command_topic").value
        self.approach_trigger_topic = self.get_parameter("approach_trigger_topic").value
        self.cooldown_sec = float(self.get_parameter("cooldown_sec").value)
        self.fuzzy_threshold = float(self.get_parameter("fuzzy_threshold").value)
        self.trick_hold_sec = float(self.get_parameter("trick_hold_sec").value)
        command_rate_hz = float(self.get_parameter("command_rate_hz").value)

        self.last_fire = 0.0
        self.active_cmd = None
        self.active_until = 0.0

        self.pub = self.create_publisher(Go2Command, self.command_topic, 10)
        self.approach_pub = self.create_publisher(
            String,
            self.approach_trigger_topic,
            10,
        )
        self.sub = self.create_subscription(
            String,
            self.text_topic,
            self.text_callback,
            10,
        )
        self.timer = self.create_timer(1.0 / max(command_rate_hz, 1.0), self.republish_active)

        self.get_logger().info(
            "Mapping %s -> %s and %s (cooldown=%.1fs)"
            % (
                self.text_topic,
                self.command_topic,
                self.approach_trigger_topic,
                self.cooldown_sec,
            )
        )

    def hold_duration_for(self, token):
        if token in TRICK_TOKENS:
            return self.trick_hold_sec
        return self.trick_hold_sec

    def republish_active(self):
        if self.active_cmd is None:
            return
        if time.monotonic() >= self.active_until:
            self.active_cmd = None
            return
        self.pub.publish(self.active_cmd)

    def activate_command(self, cmd, hold_sec):
        self.active_cmd = cmd
        self.active_until = time.monotonic() + hold_sec
        self.pub.publish(cmd)

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

        if token in APPROACH_TOKENS:
            self.active_cmd = None
            trigger = String()
            trigger.data = token
            self.approach_pub.publish(trigger)
            self.get_logger().info("'%s' -> %s [%s]" % (text, token, detail))
            return

        cmd = build_go2_command(token)
        if cmd is None:
            self.get_logger().warn("no Go2Command mapping for '%s'" % token)
            return

        self.activate_command(cmd, self.hold_duration_for(token))
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
