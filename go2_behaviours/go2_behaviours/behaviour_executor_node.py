"""
Tier 2 - Sequencing Layer: Behaviour Executor Node

Routes planner behaviours to infrastructure:
  hello, sit  -> /trigger_behaviour (sport client)
  wander      -> /active_behaviour only (wander_node + cmd_vel_bridge)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class BehaviourExecutorNode(Node):

    def __init__(self):
        super().__init__('behaviour_executor_node')

        self.declare_parameter('requested_behaviour_topic', '/requested_behaviour')
        self.declare_parameter('trigger_behaviour_topic', '/trigger_behaviour')
        self.declare_parameter('active_behaviour_topic', '/active_behaviour')

        requested_topic = self.get_parameter('requested_behaviour_topic').value
        trigger_topic = self.get_parameter('trigger_behaviour_topic').value
        active_topic = self.get_parameter('active_behaviour_topic').value

        self.safety_active = False
        self.current_behaviour = 'idle'

        self.create_subscription(String, requested_topic, self._on_request, 10)
        self.create_subscription(Bool, '/safety_override', self._on_safety, 10)

        self.trigger_pub = self.create_publisher(String, trigger_topic, 10)
        self.active_pub = self.create_publisher(String, active_topic, 10)

        self.get_logger().info('BehaviourExecutorNode ready')

    def _on_safety(self, msg: Bool):
        self.safety_active = msg.data

    def _on_request(self, msg: String):
        behaviour = msg.data.strip()
        if not behaviour:
            return

        self.current_behaviour = behaviour
        self.active_pub.publish(msg)

        if self.safety_active:
            self.get_logger().warn(
                f'Safety active — blocking behaviour: {behaviour}')
            return

        if behaviour == 'wander':
            self.get_logger().info(
                'Wander active — delegated to wander_node + cmd_vel_bridge')
            return

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
