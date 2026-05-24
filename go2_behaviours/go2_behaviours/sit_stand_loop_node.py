"""
Publishes a stand/sit loop directly to Unitree's /api/sport/request topic.
"""

import rclpy
from rclpy.node import Node
from unitree_api.msg import Request


class SitStandLoopNode(Node):
    def __init__(self):
        super().__init__('sit_stand_loop_node')

        self.request_pub = self.create_publisher(Request, '/api/sport/request', 10)

        self.stand_api_id = self.declare_parameter('stand_api_id', 1004).value
        self.sit_api_id = self.declare_parameter('sit_api_id', 1009).value
        self.period_s = float(self.declare_parameter('period_s', 4.0).value)
        self.start_with_stand = bool(self.declare_parameter('start_with_stand', True).value)

        self._next_is_stand = self.start_with_stand
        self._request_counter = 0

        self.timer = self.create_timer(self.period_s, self.timer_callback)
        self.get_logger().info(
            'SitStandLoopNode ready. Publishing every %.2f s (stand_api_id=%d, sit_api_id=%d).'
            % (self.period_s, self.stand_api_id, self.sit_api_id)
        )

    def timer_callback(self):
        if self._next_is_stand:
            api_id = int(self.stand_api_id)
            label = 'stand'
        else:
            api_id = int(self.sit_api_id)
            label = 'sit'

        self.publish_request(api_id)
        self.get_logger().info('Sent %s request (api_id=%d).' % (label, api_id))

        self._next_is_stand = not self._next_is_stand

    def publish_request(self, api_id: int):
        req = Request()
        req.header.identity.id = self.next_request_id()
        req.header.identity.api_id = api_id
        req.header.lease.id = 0
        req.header.policy.priority = 0
        req.header.policy.noreply = False
        req.parameter = ''
        req.binary = []

        self.request_pub.publish(req)

    def next_request_id(self) -> int:
        self._request_counter += 1
        return self._request_counter


def main(args=None):
    rclpy.init(args=args)
    node = SitStandLoopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
