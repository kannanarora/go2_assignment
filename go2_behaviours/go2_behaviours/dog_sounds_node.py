#!/usr/bin/env python3

"""
Ambient dog noises - a decoupled observer in the subsumption stack.

Sound is a separate actuator from the body, so this node does NOT go
through mux_node / Go2Command. It just watches the streams the mux already
produces and reacts with audio, staying silent otherwise:

    /cmd_vel            (mux MOVE output)       -> moving  -> pant occasionally
    /trigger_behaviour  (mux TRICK output)      -> stretch/... -> one-shot clip
    /dog_sound_trigger  (sound-only events)     -> bark/... -> one-shot clip

AudioHub plays one clip at a time (no true mixing), so the two tiers share
the speaker by priority: while moving, panting plays occasionally; an EVENT
sound interrupts it and holds the speaker for a
short busy window before panting resumes.

Requires AudioHubClient and the referenced clips already in AudioHub
(upload them with audiohub_player_node).
"""

import random

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import String

from go2_utils.audiohub_client import AudioHubClient


class DogSoundsNode(Node):
    def __init__(self):
        super().__init__("dog_sounds_node")

        # --- topics ---
        cmd_vel_topic = self.declare_parameter("cmd_vel_topic", "/cmd_vel").value
        trigger_topic = self.declare_parameter(
            "trigger_topic", "/trigger_behaviour"
        ).value
        behaviour_trigger_topic = self.declare_parameter(
            "behaviour_trigger_topic", trigger_topic
        ).value
        sound_trigger_topic = self.declare_parameter(
            "sound_trigger_topic", "/dog_sound_trigger"
        ).value

        # --- EVENT tier: token -> file_name played one-shot ---
        event_map = self.declare_parameter(
            "event_sound_map",
            ["dance:bark2", "sit:bark", "stretch:stretch1",
             "bark:bark", "speak:bark"],
        ).value

        # --- AMBIENT tier ---
        self.panting_clips = self.declare_parameter(
            "panting_clips", ["panting1", "panting2"]
        ).value
        # empty -> idle breathing disabled (no breathing clip in the set)
        self.breathing_clip = self.declare_parameter(
            "breathing_clip", ""
        ).value

        # how "moving" is decided from /cmd_vel
        self.move_speed_threshold = float(
            self.declare_parameter("move_speed_threshold", 0.05).value
        )
        self.cmd_timeout_s = float(
            self.declare_parameter("cmd_timeout_s", 0.5).value
        )

        # panting cadence while moving: each pant waits a RANDOM gap in
        # [min, max] before the next one, and a random clip is chosen, so it
        # sounds natural rather than a fixed metronome
        self.pant_min_gap_s = float(
            self.declare_parameter("pant_min_gap_s", 2.0).value
        )
        self.pant_max_gap_s = float(
            self.declare_parameter("pant_max_gap_s", 6.0).value
        )
        if self.pant_max_gap_s < self.pant_min_gap_s:
            self.pant_max_gap_s = self.pant_min_gap_s

        # idle breathing period (while standing still)
        self.idle_breathe_gap_s = float(
            self.declare_parameter("idle_breathe_gap_s", 8.0).value
        )

        # after an EVENT clip, suppress ambient for this long so they don't cut
        # each other off (AudioHub plays one clip at a time)
        self.event_busy_s = float(
            self.declare_parameter("event_busy_s", 2.0).value
        )
        ambient_rate_hz = float(
            self.declare_parameter("ambient_rate_hz", 5.0).value
        )

        self._client = AudioHubClient(self)

        # resolve every referenced clip to a UUID once at startup
        self._event_uuids = self._resolve_event_map(event_map)
        self._pant_uuids = [
            u for u in (self._resolve(c) for c in self.panting_clips) if u
        ]
        self._breathe_uuid = (
            self._resolve(self.breathing_clip) if self.breathing_clip else None
        )

        # runtime state
        self._last_speed = 0.0
        self._last_cmd_time = None
        self._busy_until = self.get_clock().now()
        self._last_pant_time = self.get_clock().now()
        self._pant_gap_s = random.uniform(self.pant_min_gap_s, self.pant_max_gap_s)
        self._last_breathe_time = self.get_clock().now()

        self.create_subscription(Twist, cmd_vel_topic, self._on_cmd_vel, 10)
        self.create_subscription(
            String,
            behaviour_trigger_topic,
            self._on_behaviour_trigger,
            10,
        )
        if sound_trigger_topic != behaviour_trigger_topic:
            self.create_subscription(
                String,
                sound_trigger_topic,
                self._on_sound_trigger,
                10,
            )
        self.create_timer(1.0 / max(ambient_rate_hz, 0.1), self._ambient_tick)

        self.get_logger().info(
            "DogSoundsNode observing %s + %s + %s (events=%s, pants=%d)"
            % (cmd_vel_topic, behaviour_trigger_topic, sound_trigger_topic,
               sorted(self._event_uuids), len(self._pant_uuids))
        )

    # ---- startup resolution ----

    def _resolve(self, file_name):
        uuid = self._client.resolve_uuid(file_name)
        if uuid is None:
            self.get_logger().warn(
                "'%s' not in AudioHub - upload it first with audiohub_player_node"
                % file_name
            )
        return uuid

    def _resolve_event_map(self, entries):
        out = {}
        for entry in entries:
            token, sep, file_name = entry.partition(":")
            token = token.strip().lower()
            file_name = file_name.strip()
            if not sep or not token or not file_name:
                self.get_logger().warn("Ignoring malformed event entry: '%s'" % entry)
                continue
            uuid = self._resolve(file_name)
            if uuid:
                out[token] = uuid
        return out

    # ---- shared speaker helpers ----

    def _busy(self):
        return self.get_clock().now() < self._busy_until

    def _play_event(self, uuid):
        # events reserve the speaker so ambient won't cut them off
        self._client.play(uuid)
        self._busy_until = self.get_clock().now() + Duration(seconds=self.event_busy_s)

    def _play_ambient(self, uuid):
        # ambient (pant/breathe) does NOT reserve the speaker, so panting can
        # loop continuously; events still interrupt it via _busy()
        self._client.play(uuid)

    def _seconds_since(self, stamp):
        return (self.get_clock().now() - stamp).nanoseconds / 1e9

    # ---- EVENT tier (high priority) ----

    def _play_trigger_token(self, token, source):
        token = token.strip().lower()
        uuid = self._event_uuids.get(token)
        if uuid is None:
            return
        self._play_event(uuid)
        self.get_logger().info("Event sound for '%s' from %s" % (token, source))

    def _on_behaviour_trigger(self, msg: String):
        self._play_trigger_token(msg.data, "behaviour")

    def _on_sound_trigger(self, msg: String):
        self._play_trigger_token(msg.data, "sound")

    def _on_trigger(self, msg: String):
        # Backward-compatible callback name for old launch files/tests.
        self._on_behaviour_trigger(msg)

    # ---- motion observation ----

    def _on_cmd_vel(self, msg: Twist):
        self._last_speed = max(
            abs(msg.linear.x), abs(msg.linear.y), abs(msg.angular.z)
        )
        self._last_cmd_time = self.get_clock().now()

    def _is_moving(self):
        if self._last_cmd_time is None:
            return False
        if self._seconds_since(self._last_cmd_time) > self.cmd_timeout_s:
            return False
        return self._last_speed > self.move_speed_threshold

    # ---- AMBIENT tier (low priority) ----

    def _ambient_tick(self):
        if self._busy():
            return  # an event (or recent ambient) owns the speaker

        if self._is_moving():
            # random clip after a random gap -> natural, irregular panting
            if (self._pant_uuids
                    and self._seconds_since(self._last_pant_time)
                    >= self._pant_gap_s):
                self._play_ambient(random.choice(self._pant_uuids))
                self._last_pant_time = self.get_clock().now()
                self._pant_gap_s = random.uniform(
                    self.pant_min_gap_s, self.pant_max_gap_s
                )
                self.get_logger().info("pant (next in %.1fs)" % self._pant_gap_s)
        else:
            if (self._breathe_uuid
                    and self._seconds_since(self._last_breathe_time)
                    >= self.idle_breathe_gap_s):
                self._play_ambient(self._breathe_uuid)
                self._last_breathe_time = self.get_clock().now()
                self.get_logger().info("breathe")


def main(args=None):
    rclpy.init(args=args)
    node = DogSoundsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
