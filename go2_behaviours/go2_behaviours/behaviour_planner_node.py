"""
Tier 3 - Deliberative Layer: Behaviour Planner Node
"""

import math
import time

import rclpy
from rclpy.node import Node as RosNode
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

from go2_behaviours.behaviour_tree import (
    Action,
    Blackboard,
    Condition,
    Node as BTNode,
    Selector,
    Sequence,
    Status,
)


def _safety_inactive(bb: Blackboard) -> bool:
    return not bb.safety_active


def _not_in_hello_cooldown(bb: Blackboard) -> bool:
    return bb.now >= bb.hello_cooldown_until


def _not_too_close_for_hello(bb: Blackboard) -> bool:
    if math.isfinite(bb.front_range) and bb.front_range <= bb.safety_clear_dist:
        bb.hello_streak = 0
        return False
    return True


def _update_greeting_streak(bb: Blackboard) -> bool:
    in_band = (
        math.isfinite(bb.front_range)
        and bb.greeting_min <= bb.front_range < bb.greeting_dist
    )
    if in_band:
        bb.hello_streak += 1
    else:
        bb.hello_streak = 0
    return in_band


def _hello_confirmed(bb: Blackboard) -> bool:
    return bb.hello_streak >= bb.hello_confirm_ticks


def _request_hello(bb: Blackboard) -> Status:
    range_text = f'{bb.front_range:.2f}m'
    bb.chosen_behaviour = 'hello'
    bb.chosen_reason = range_text
    bb.hello_streak = 0
    bb.hello_cooldown_until = bb.now + bb.hello_cooldown_s
    return Status.SUCCESS


def _idle(bb: Blackboard) -> Status:
    return Status.FAILURE


def build_planner_tree() -> BTNode:
    return Selector(
        'planner_root',
        [
            Sequence(
                'lidar_hello',
                [
                    Condition('safety_inactive', _safety_inactive),
                    Condition('not_in_cooldown', _not_in_hello_cooldown),
                    Condition('not_too_close', _not_too_close_for_hello),
                    Condition('in_greeting_band', _update_greeting_streak),
                    Condition('hello_confirmed', _hello_confirmed),
                    Action('request_hello', _request_hello),
                ],
            ),
            Action('idle', _idle),
        ],
    )


class BehaviourPlannerNode(RosNode):

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

        self.scan_topic = self.get_parameter('scan_topic').value
        tick_rate = self.get_parameter('tick_rate_s').value
        self.front_half_angle_deg = self.get_parameter('front_half_angle_deg').value

        self._blackboard = Blackboard()
        self._blackboard.greeting_min = self.get_parameter(
            'greeting_min_distance').value
        self._blackboard.greeting_dist = self.get_parameter(
            'greeting_distance').value
        self._blackboard.safety_clear_dist = self.get_parameter(
            'safety_clear_distance').value
        self._blackboard.hello_confirm_ticks = int(
            self.get_parameter('hello_confirm_ticks').value)
        self._blackboard.hello_cooldown_s = float(
            self.get_parameter('hello_cooldown_s').value)

        self._tree = build_planner_tree()

        scan_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(Bool, '/safety_override', self._on_safety, 10)
        self.create_subscription(
            LaserScan, self.scan_topic, self._on_scan, scan_qos)

        self.behaviour_pub = self.create_publisher(String, '/requested_behaviour', 10)
        self.timer = self.create_timer(tick_rate, self._tick_tree)

        self.get_logger().info(
            'BehaviourPlannerNode ready — behaviour tree (LiDAR hello)'
        )

    def _on_safety(self, msg: Bool):
        self._blackboard.safety_active = msg.data
        if self._blackboard.safety_active:
            self._blackboard.hello_streak = 0

    def _on_scan(self, msg: LaserScan):
        self._blackboard.front_range = self._front_min_range(msg)

    def _tick_tree(self):
        bb = self._blackboard
        bb.now = time.monotonic()
        bb.chosen_behaviour = None
        bb.chosen_reason = None

        self._tree.tick(bb)

        if bb.chosen_behaviour:
            self._publish_behaviour(bb.chosen_behaviour, bb.chosen_reason)

    def _publish_behaviour(self, behaviour: str, reason: str):
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
