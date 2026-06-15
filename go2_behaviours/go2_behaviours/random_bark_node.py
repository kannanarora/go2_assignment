#!/usr/bin/env python3

"""
Bark at random intervals

Publishes a bark token to /trigger_behaviour every so often so the dog
barks on its own. bark_node listens on the same topic and plays the sound
"""

import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RandomBarkNode(Node):
    def __init__(self):
        super().__init__("random_bark_node")

        self.trigger_topic = self.declare_parameter(
            "trigger_topic", "/trigger_behaviour"
        ).value
        self.bark_token = self.declare_parameter("bark_token", "bark").value
        self.min_interval_s = float(
            self.declare_parameter("min_interval_s", 8.0).value
        )
        self.max_interval_s = float(
            self.declare_parameter("max_interval_s", 30.0).value
        )

        if self.max_interval_s < self.min_interval_s:
            self.max_interval_s = self.min_interval_s
            self.get_logger().warn(
                "max_interval_s must be >= min_interval_s; adjusted to %.1f"
                % self.max_interval_s
            )

        self._pub = self.create_publisher(String, self.trigger_topic, 10)
        self.timer = None
        self._schedule_next()

        self.get_logger().info(
            "RandomBarkNode barking '%s' on %s every %.1f-%.1fs"
            % (
                self.bark_token,
                self.trigger_topic,
                self.min_interval_s,
                self.max_interval_s,
            )
        )

    def _schedule_next(self):
        if self.timer is not None:
            self.timer.cancel()
        delay = random.uniform(self.min_interval_s, self.max_interval_s)
        self.timer = self.create_timer(delay, self._on_timer)

    def _on_timer(self):
        msg = String()
        msg.data = self.bark_token
        self._pub.publish(msg)
        self.get_logger().info("Random bark!")
        self._schedule_next()


def main(args=None):
    rclpy.init(args=args)
    node = RandomBarkNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
