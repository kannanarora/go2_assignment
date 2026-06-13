"""
Steps:
  1. Subscribes to /trigger_behaviour
  2. Translates the behaviour name into a Unitree API request message
  3. Publishes that request to /api/sport/request

TEST (from terminal, robot must be connected):
  ros2 topic pub /trigger_behaviour std_msgs/msg/String "data: 'stand'"
  ros2 topic pub /trigger_behaviour std_msgs/msg/String "data: 'lie_down'"
"""

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from unitree_api.msg import Request


# these numbers come from the Unitree SDK. Each one maps to a robot action.
SPORT_API_ID_STAND_UP      = 1004
SPORT_API_ID_STAND_DOWN    = 1005   # lie down / safe position
SPORT_API_ID_STOP_MOVE     = 1003
SPORT_API_ID_SIT            = 1009
SPORT_API_ID_RISESIT        = 1010
SPORT_API_ID_HELLO          = 1016  # wave hello
SPORT_API_ID_BALANCE_STAND  = 1002
SPORT_API_ID_SWITCH_JOYSTICK = 1027
SPORT_API_ID_FREEWALK = 2045


class SportClientWrapperNode(Node):
    def __init__(self):
        super().__init__('sport_client_wrapper_node')

        self.trigger_topic = self.declare_parameter(
            'trigger_topic', '/trigger_behaviour'
        ).value
        self.request_topic = self.declare_parameter(
            'request_topic', '/api/sport/request'
        ).value

        self.behaviour_sub = self.create_subscription(
            String,
            self.trigger_topic,
            self.behaviour_callback,
            10
        )

        self.request_pub = self.create_publisher(
            Request,
            self.request_topic,
            10
        )

        self.command_handlers = {
            'stand': lambda: self.send_request(SPORT_API_ID_STAND_UP),
            'balance_stand': lambda: self.send_request(SPORT_API_ID_BALANCE_STAND),
            'free_walk': lambda: self.send_request(SPORT_API_ID_FREEWALK),
            'lie_down': lambda: self.send_request(SPORT_API_ID_STAND_DOWN),
            'stop': lambda: self.send_request(SPORT_API_ID_STOP_MOVE),
            'sit': lambda: self.send_request(SPORT_API_ID_SIT),
            'rise_sit': lambda: self.send_request(SPORT_API_ID_RISESIT),
            'hello': lambda: self.send_request(SPORT_API_ID_HELLO),
            'joystick_on': lambda: self.send_request(
                SPORT_API_ID_SWITCH_JOYSTICK, {'data': True}
            ),
            'joystick_off': lambda: self.send_request(
                SPORT_API_ID_SWITCH_JOYSTICK, {'data': False}
            ),
        }

        self.get_logger().info('SportClientWrapperNode is ready.')
        self.get_logger().info('Listening on %s ...' % self.trigger_topic)

    def behaviour_callback(self, msg: String):
        raw_command = msg.data.strip()
        command = raw_command.lower()

        self.get_logger().info(f'Received command: "{raw_command}"')

        handler = self.command_handlers.get(command)
        if handler is None:
            self.get_logger().warn('Unknown command: "%s" — ignoring.' % command)
            return

        handler()

    def send_request(self, api_id: int, params: dict = None):
        req = self.build_request(api_id, params)
        self.request_pub.publish(req)
        self.get_logger().info(
            'Sent request: api_id=%d, params=%s' % (api_id, params)
        )

    def build_request(
        self,
        api_id: int,
        params: dict = None,
        noreply: bool = False,
    ) -> Request:
        req = Request()
        req.header.identity.id = self.get_next_id()
        req.header.identity.api_id = int(api_id)
        req.header.lease.id = 0
        req.header.policy.priority = 0
        req.header.policy.noreply = bool(noreply)
        req.parameter = json.dumps(params) if params else ''
        req.binary = []
        return req

    _request_counter = 0

    def get_next_id(self) -> int:
        SportClientWrapperNode._request_counter += 1
        return SportClientWrapperNode._request_counter


def main(args=None):
    rclpy.init(args=args)
    node = SportClientWrapperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
