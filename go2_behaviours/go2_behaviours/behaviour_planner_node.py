"""
Tier 3 - Deliberative Layer: Behaviour Planner Node
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String


class BehaviourPlannerNode(Node):

    VOICE_BEHAVIOURS = (
        ('hello', ('hello', 'hi', 'hey', 'greetings')),
        ('sit', ('sit', 'sit down', 'take a rest', 'rest')),
        ('rise_sit', ('stand', 'stand up', 'get up', 'rise')),
        ('stop', ('stop', 'freeze', 'halt')),
    )

    def __init__(self):
        super().__init__('behaviour_planner_node')

        self.declare_parameter('scan_topic', '/front_scan')
        self.declare_parameter('greeting_min_distance', 1.8)
        self.declare_parameter('greeting_distance', 2.3)
        self.declare_parameter('safety_clear_distance', 1.2)
        self.declare_parameter('front_half_angle_deg', 15.0)
        self.declare_parameter('tick_rate_s', 3.0)
        self.declare_parameter('hello_confirm_ticks', 2)
        self.declare_parameter('hello_cooldown_s', 12.0)
        self.declare_parameter('voice_topic', '/go2/whisper/text')

        self.scan_topic = self.get_parameter('scan_topic').value
        self.greeting_min = self.get_parameter('greeting_min_distance').value
        self.greeting_dist = self.get_parameter('greeting_distance').value
        self.safety_clear_dist = self.get_parameter('safety_clear_distance').value
        self.front_half_angle_deg = self.get_parameter('front_half_angle_deg').value
        tick_rate = self.get_parameter('tick_rate_s').value
        self.hello_confirm_ticks = int(
            self.get_parameter('hello_confirm_ticks').value
        )
        self.hello_cooldown_s = float(
            self.get_parameter('hello_cooldown_s').value
        )
        voice_topic = self.get_parameter('voice_topic').value

        self.safety_active = False
        self.front_range = float('inf')
        self._hello_streak = 0
        self._hello_cooldown_until = 0.0

        scan_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(Bool, '/safety_override', self._on_safety, 10)
        self.create_subscription(
            LaserScan, self.scan_topic, self._on_scan, scan_qos)
        self.create_subscription(String, voice_topic, self._on_voice, 10)

        self.behaviour_pub = self.create_publisher(String, '/requested_behaviour', 10)
        self.timer = self.create_timer(tick_rate, self._decide)

        self.get_logger().info(
            'BehaviourPlannerNode ready — voice + optional LiDAR hello'
        )

    def _on_safety(self, msg: Bool):
        self.safety_active = msg.data
        if self.safety_active:
            self._hello_streak = 0

    def _on_scan(self, msg: LaserScan):
        self.front_range = self._front_min_range(msg)

    def _on_voice(self, msg: String):
        if self.safety_active:
            return

        text = msg.data.strip().lower()
        if not text:
            return

        for behaviour, keywords in self.VOICE_BEHAVIOURS:
            if any(keyword in text for keyword in keywords):
                self._request_behaviour(behaviour, f'voice="{text[:40]}"')
                return

    def _decide(self):
        if self.safety_active:
            return

        now = time.monotonic()
        if now < self._hello_cooldown_until:
            return

        if (
            math.isfinite(self.front_range)
            and self.front_range <= self.safety_clear_dist
        ):
            self._hello_streak = 0
            return

        in_greeting_band = (
            math.isfinite(self.front_range)
            and self.greeting_min <= self.front_range < self.greeting_dist
        )

        if in_greeting_band:
            self._hello_streak += 1
        else:
            self._hello_streak = 0

        if self._hello_streak < self.hello_confirm_ticks:
            return

        range_text = f'{self.front_range:.2f}m'
        self._request_behaviour('hello', range_text)
        self._hello_streak = 0
        self._hello_cooldown_until = now + self.hello_cooldown_s

    def _request_behaviour(self, behaviour: str, reason: str):
        self.get_logger().info(
            f'Requesting behaviour: {behaviour} ({reason})'
        )
        msg = String()
        msg.data = behaviour
        self.behaviour_pub.publish(msg)

    def _angle_to_index(self, scan: LaserScan, angle_rad: float) -> int:
        if scan.angle_increment == 0.0 or len(scan.ranges) == 0:
            return 0
        idx = int(round((angle_rad - scan.angle_min) / scan.angle_increment))
        return max(0, min(idx, len(scan.ranges) - 1))

    def _front_min_range(self, scan: LaserScan) -> float:
        if not scan.ranges:
            return float('inf')

        half_angle_rad = math.radians(self.front_half_angle_deg)
        i0 = self._angle_to_index(scan, -half_angle_rad)
        i1 = self._angle_to_index(scan, half_angle_rad)
        if i0 > i1:
            i0, i1 = i1, i0

        valid_ranges = []
        for value in scan.ranges[i0:i1 + 1]:
            if not math.isfinite(value):
                continue
            value = float(value)
            if value <= 0.0 or value < scan.range_min or value > scan.range_max:
                continue
            valid_ranges.append(value)

        return min(valid_ranges) if valid_ranges else float('inf')


def main(args=None):
    rclpy.init(args=args)
    node = BehaviourPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
