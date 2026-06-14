"""
Tier 2 - Sequencing Layer: Behaviour Executor Node
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class BehaviourExecutorNode(Node):

    PAUSE_WANDER = frozenset({'sit', 'stop'})
    RESUME_WANDER = frozenset({'rise_sit', 'stand', 'balance_stand'})

    def __init__(self):
        super().__init__('behaviour_executor_node')

        self.declare_parameter('requested_behaviour_topic', '/requested_behaviour')
        self.declare_parameter('trigger_behaviour_topic', '/trigger_behaviour')
        self.declare_parameter('wander_pause_topic', '/wander_pause')

        requested_topic = self.get_parameter('requested_behaviour_topic').value
        trigger_topic = self.get_parameter('trigger_behaviour_topic').value
        wander_pause_topic = self.get_parameter('wander_pause_topic').value

        self.safety_active = False

        self.create_subscription(String, requested_topic, self._on_request, 10)
        self.create_subscription(Bool, '/safety_override', self._on_safety, 10)

        self.trigger_pub = self.create_publisher(String, trigger_topic, 10)
        self.wander_pause_pub = self.create_publisher(Bool, wander_pause_topic, 10)

        self.get_logger().info('BehaviourExecutorNode ready')

    def _on_safety(self, msg: Bool):
        self.safety_active = msg.data

    def _set_wander_pause(self, paused: bool):
        msg = Bool()
        msg.data = paused
        self.wander_pause_pub.publish(msg)

    def _on_request(self, msg: String):
        behaviour = msg.data.strip()
        if not behaviour:
            return

        if self.safety_active and behaviour not in self.RESUME_WANDER:
            self.get_logger().warn(
                f'Safety active — blocking behaviour: {behaviour}')
            return

        if behaviour in self.PAUSE_WANDER:
            self._set_wander_pause(True)
        elif behaviour in self.RESUME_WANDER:
            self._set_wander_pause(False)

        self.get_logger().info(f'Executing behaviour: {behaviour}')
        self.trigger_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BehaviourExecutorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
