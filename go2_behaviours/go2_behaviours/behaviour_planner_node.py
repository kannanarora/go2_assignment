"""
Tier 3 - Deliberative Layer: Behaviour Planner Node
"""

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from unitree_go.msg import SportModeState


class BehaviourPlannerNode(Node):

    def __init__(self):
        super().__init__('behaviour_planner_node')

        # Parameters
        self.declare_parameter('greeting_distance', 2.0)
        self.declare_parameter('idle_timeout_s', 10.0)
        self.declare_parameter('tick_rate_s', 3.0)

        self.greeting_dist = self.get_parameter('greeting_distance').value
        self.idle_timeout = self.get_parameter('idle_timeout_s').value
        tick_rate = self.get_parameter('tick_rate_s').value

        # State
        self.safety_active = False
        self.nearest_obstacle = 999.0
        self.last_action_time = time.monotonic()
        self.current_behaviour = 'idle'

        # Subscribers
        self.create_subscription(
            Bool, '/safety_override', self._on_safety, 10)
        self.create_subscription(
            SportModeState, '/sportmodestate', self._on_sportmode, 10)

        # Publisher
        self.behaviour_pub = self.create_publisher(
            String, '/requested_behaviour', 10)

        # Decision timer re-evaluates context every tick_rate seconds
        self.timer = self.create_timer(tick_rate, self._decide)

        self.get_logger().info(
            f'BehaviourPlannerNode ready — '
            f'greeting_dist={self.greeting_dist}m, '
            f'idle_timeout={self.idle_timeout}s'
        )

    def _on_safety(self, msg: Bool):
        self.safety_active = msg.data
        if self.safety_active:
            self.get_logger().info('Safety active — planner paused')

    def _on_sportmode(self, msg: SportModeState):
        valid = [d for d in msg.range_obstacle if 0.0 < d < 10.0]
        if valid:
            self.nearest_obstacle = min(valid)

    def _decide(self):
        # Tier 1 has control
        if self.safety_active:
            return

        now = time.monotonic()
        idle_duration = now - self.last_action_time

        # Priority 1, greet if something in greeting range
        if self.nearest_obstacle < self.greeting_dist:
            self._request_behaviour('hello')
            return

        # Priority 2, sit if idle too long
        if idle_duration > self.idle_timeout:
            self._request_behaviour('sit')
            return

        # default, wander
        self._request_behaviour('wander')

    def _request_behaviour(self, behaviour: str):
        # only publish if behaviour changed, avoid spamming
        if behaviour == self.current_behaviour:
            return

        self.get_logger().info(f'Requesting behaviour: {behaviour}')
        msg = String()
        msg.data = behaviour
        self.behaviour_pub.publish(msg)
        self.current_behaviour = behaviour
        self.last_action_time = time.monotonic()


def main(args=None):
    rclpy.init(args=args)
    node = BehaviourPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
