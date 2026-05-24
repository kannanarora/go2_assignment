"""
Publishes a lie-down/stand loop to the sport client wrapper.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SitStandLoopNode(Node):
    def __init__(self):
        super().__init__('sit_stand_loop_node')

        self.trigger_topic = '/trigger_behaviour'
        self.command_pub = self.create_publisher(String, self.trigger_topic, 10)

        self.period_s = float(self.declare_parameter('period_s', 3.0).value)

        self._next_is_stand = False

        self.timer = self.create_timer(self.period_s, self.timer_callback)
        self.get_logger().info(
            'SitStandLoopNode ready. Publishing every %.2f s to %s.'
            % (self.period_s, self.trigger_topic)
        )

    def timer_callback(self):
        if self._next_is_stand:
            command = 'stand'
        else:
            command = 'lie_down'

        self.publish_command(command)
        self.get_logger().info('Sent command: %s' % command)

        self._next_is_stand = not self._next_is_stand

    def publish_command(self, command: str):
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SitStandLoopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
