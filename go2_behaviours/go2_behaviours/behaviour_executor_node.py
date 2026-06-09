"""
Tier 2 - Sequencing Layer: Behaviour Executor Node
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class BehaviourExecutorNode(Node):

    def __init__(self):
        super().__init__('behaviour_executor_node')

        self.safety_active = False

        # listen to Tier 3 planner
        self.create_subscription(
            String, '/requested_behaviour', self._on_request, 10)

        # listen to Tier 1 safety
        self.create_subscription(
            Bool, '/safety_override', self._on_safety, 10)

        # forward to infrastructure
        self.trigger_pub = self.create_publisher(
            String, '/trigger_behaviour', 10)

        self.get_logger().info('BehaviourExecutorNode ready')

    def _on_safety(self, msg: Bool):
        self.safety_active = msg.data

    def _on_request(self, msg: String):
        if self.safety_active:
            self.get_logger().warn(
                f'Safety active — blocking behaviour: {msg.data}')
            return

        self.get_logger().info(f'Executing behaviour: {msg.data}')
        self.trigger_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BehaviourExecutorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
