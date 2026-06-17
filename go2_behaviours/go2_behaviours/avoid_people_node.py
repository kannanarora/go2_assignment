#!/usr/bin/env python3

"""
Top-tier person safety behaviour.

Subscribes to PersonTrack and publishes a high-priority sit command while a
person is close to the robot. It stays silent when inactive so lower mux tiers
can run normally.
"""

import time

import rclpy
from go2_interfaces.msg import Go2Command, PersonTrack
from rclpy.node import Node


class AvoidPeopleNode(Node):
    def __init__(self):
        super().__init__("avoid_people_node")

        self.person_track_topic = self.declare_parameter(
            "person_track_topic", "/person_track"
        ).value
        self.command_topic = self.declare_parameter(
            "command_topic", "/avoid_people_cmd"
        ).value

        self.sit_distance_m = float(
            self.declare_parameter("sit_distance_m", 1.0).value
        )
        self.clear_distance_m = float(
            self.declare_parameter("clear_distance_m", 1.15).value
        )
        self.min_person_confidence = float(
            self.declare_parameter("min_person_confidence", 0.30).value
        )
        self.confirm_near_s = float(self.declare_parameter("confirm_near_s", 0.15).value)
        self.lost_person_grace_s = float(
            self.declare_parameter("lost_person_grace_s", 0.8).value
        )
        self.command_rate_hz = float(
            self.declare_parameter("command_rate_hz", 5.0).value
        )
        self.log_rate_hz = float(self.declare_parameter("log_rate_hz", 1.0).value)
        self.sit_command = self.declare_parameter("sit_command", "sit").value
        self.use_tracker_nearby_fallback = bool(
            self.declare_parameter("use_tracker_nearby_fallback", True).value
        )

        if self.clear_distance_m <= self.sit_distance_m:
            self.clear_distance_m = self.sit_distance_m + 0.15
            self.get_logger().warn(
                "clear_distance_m must be > sit_distance_m; adjusted to %.2f"
                % self.clear_distance_m
            )

        self.latest_person = None
        self.latest_person_time = 0.0
        self.near_candidate_since = 0.0
        self.sitting_for_person = False
        self._last_log_time = 0.0

        self.command_pub = self.create_publisher(Go2Command, self.command_topic, 10)
        self.person_sub = self.create_subscription(
            PersonTrack,
            self.person_track_topic,
            self.person_callback,
            10,
        )

        timer_period = 1.0 / max(self.command_rate_hz, 1.0)
        self.timer = self.create_timer(timer_period, self.tick)

        self.get_logger().info(
            "AvoidPeopleNode listening to %s, publishing %s "
            "(sit<=%.2fm, clear>=%.2fm)"
            % (
                self.person_track_topic,
                self.command_topic,
                self.sit_distance_m,
                self.clear_distance_m,
            )
        )

    def person_callback(self, msg: PersonTrack):
        now = time.monotonic()

        if not msg.visible:
            self.latest_person = None
            self.latest_person_time = now
            return

        if float(msg.confidence) < self.min_person_confidence:
            return

        self.latest_person = msg
        self.latest_person_time = now

    def tick(self):
        now = time.monotonic()
        close_person = self.close_person_detected(now)

        if close_person:
            if self.near_candidate_since == 0.0:
                self.near_candidate_since = now
        else:
            self.near_candidate_since = 0.0

        confirmed_close = (
            close_person
            and now - self.near_candidate_since >= self.confirm_near_s
        )

        if confirmed_close:
            self.sitting_for_person = True

        if self.sitting_for_person and self.person_is_clear(now):
            self.sitting_for_person = False

        if self.sitting_for_person:
            self.publish_sit()
            self.maybe_log("sitting")
        else:
            self.maybe_log("clear")

    def close_person_detected(self, now: float) -> bool:
        person = self.latest_person
        if person is None:
            return False

        if now - self.latest_person_time >= self.lost_person_grace_s:
            return False

        if person.distance_valid:
            return float(person.distance_m) <= self.sit_distance_m

        return self.use_tracker_nearby_fallback and bool(person.nearby)

    def person_is_clear(self, now: float) -> bool:
        person = self.latest_person
        if now - self.latest_person_time >= self.lost_person_grace_s:
            return True

        if person is None:
            return True

        if person.distance_valid:
            return float(person.distance_m) >= self.clear_distance_m

        if self.use_tracker_nearby_fallback and bool(person.nearby):
            return False

        return True

    def publish_sit(self):
        msg = Go2Command()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.command_type = Go2Command.TRICK
        msg.trick_name = self.sit_command
        self.command_pub.publish(msg)

    def maybe_log(self, status: str):
        if self.log_rate_hz <= 0.0:
            return

        now = time.monotonic()
        if now - self._last_log_time < 1.0 / self.log_rate_hz:
            return

        self._last_log_time = now
        person = self.latest_person
        if person is None:
            self.get_logger().info("status=%s no_person" % status)
            return

        distance = (
            "%.2fm" % person.distance_m if person.distance_valid else "invalid"
        )
        self.get_logger().info(
            "status=%s confidence=%.2f distance=%s nearby=%s"
            % (status, person.confidence, distance, person.nearby)
        )


def main(args=None):
    rclpy.init(args=args)
    node = AvoidPeopleNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
