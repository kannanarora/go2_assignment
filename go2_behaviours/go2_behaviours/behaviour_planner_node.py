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

    def __init__(self):
        super().__init__('behaviour_planner_node')

        self.declare_parameter('scan_topic', '/front_scan')
        self.declare_parameter('greeting_min_distance', 1.5)
        self.declare_parameter('greeting_distance', 2.5)
        self.declare_parameter('safety_clear_distance', 1.2)
        self.declare_parameter('front_half_angle_deg', 15.0)
        self.declare_parameter('idle_timeout_s', 10.0)
        self.declare_parameter('tick_rate_s', 3.0)
        self.declare_parameter('voice_topic', '/go2/whisper/text')
        self.declare_parameter(
            'voice_greeting_keywords',
            ['hello', 'hi', 'hey', 'greetings'],
        )

        self.scan_topic = self.get_parameter('scan_topic').value
        self.greeting_min = self.get_parameter('greeting_min_distance').value
        self.greeting_dist = self.get_parameter('greeting_distance').value
        self.safety_clear_dist = self.get_parameter('safety_clear_distance').value
        self.front_half_angle_deg = self.get_parameter('front_half_angle_deg').value
        self.idle_timeout = self.get_parameter('idle_timeout_s').value
        tick_rate = self.get_parameter('tick_rate_s').value
        voice_topic = self.get_parameter('voice_topic').value
        self.voice_keywords = list(
            self.get_parameter('voice_greeting_keywords').value
        )

        self.safety_active = False
        self.front_range = float('inf')
        self.last_action_time = time.monotonic()
        self.current_behaviour = 'idle'

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
            f'BehaviourPlannerNode ready — hello={self.greeting_min}-{self.greeting_dist}m'
        )

    def _on_safety(self, msg: Bool):
        self.safety_active = msg.data
        if self.safety_active:
            self.get_logger().info('Safety active — planner paused')

    def _on_scan(self, msg: LaserScan):
        self.front_range = self._front_min_range(msg)

    def _on_voice(self, msg: String):
        if self.safety_active:
            return

        text = msg.data.strip().lower()
        if not text:
            return

        if any(keyword in text for keyword in self.voice_keywords):
            self._request_behaviour('hello', f'voice="{text[:40]}"')

    def _decide(self):
        if self.safety_active:
            return

        now = time.monotonic()
        idle_duration = now - self.last_action_time
        range_text = (
            f'{self.front_range:.2f}m'
            if math.isfinite(self.front_range) else 'inf'
        )

        # Let Tier 1 safety handle close approach and stand-up.
        if (
            math.isfinite(self.front_range)
            and self.front_range <= self.safety_clear_dist
        ):
            return

        if (
            math.isfinite(self.front_range)
            and self.greeting_min <= self.front_range < self.greeting_dist
        ):
            self._request_behaviour('hello', range_text)
            return

        # idle sit only with a confirmed clear reading
        if (
            idle_duration > self.idle_timeout
            and math.isfinite(self.front_range)
            and self.front_range >= self.greeting_dist
        ):
            self._request_behaviour('sit', range_text)
            return

        self._request_behaviour('wander', range_text)

    def _request_behaviour(self, behaviour: str, range_text: str):
        if behaviour == self.current_behaviour:
            return
        self.get_logger().info(
            f'Requesting behaviour: {behaviour} (front={range_text})'
        )
        msg = String()
        msg.data = behaviour
        self.behaviour_pub.publish(msg)
        self.current_behaviour = behaviour
        self.last_action_time = time.monotonic()

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
