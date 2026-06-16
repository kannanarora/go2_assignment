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
SPORT_API_ID_STRETCH = 1017
SPORT_API_ID_FREEWALK = 2045
SPORT_API_ID_FREEWALK_AVOID = 2048
SPORT_API_ID_DANCE1 = 1022          # Dance1 routine (Dance2 is 1023)


class SportClientWrapperNode(Node):
    def __init__(self):
        super().__init__('sport_client_wrapper_node')

        self.trigger_topic = self.declare_parameter(
            'trigger_topic', '/trigger_behaviour'
        ).value
        self.request_topic = self.declare_parameter(
            'request_topic', '/api/sport/request'
        ).value

        # SUBSCRIBER: listens for behaviour commands from other nodes
        # Any node in the system can send a string here to make the robot move
        self.behaviour_sub = self.create_subscription(
            String,
            self.trigger_topic,
            self.behaviour_callback,
            10
        )

        # PUBLISHER: sends the actual command to the robot
        # The Go2 listens on /api/sport/request for all movement commands
        self.request_pub = self.create_publisher(
            Request,
            self.request_topic,
            10
        )

        self._request_counter = 0

        self.command_handlers = {
            'stand': lambda: self.send_request(SPORT_API_ID_STAND_UP),
            'balance_stand': lambda: self.send_request(SPORT_API_ID_BALANCE_STAND),
            'stretch': lambda: self.send_request(SPORT_API_ID_STRETCH),
            'free_walk': lambda: self.send_request(SPORT_API_ID_FREEWALK),
            'free_avoid': lambda: self.send_request(SPORT_API_ID_FREEWALK_AVOID),
            'lie_down': lambda: self.send_request(SPORT_API_ID_STAND_DOWN),
            'stop': lambda: self.send_request(SPORT_API_ID_STOP_MOVE),
            'sit': lambda: self.send_request(SPORT_API_ID_SIT),
            'rise_sit': lambda: self.send_request(SPORT_API_ID_RISESIT),
            'hello': lambda: self.send_request(SPORT_API_ID_HELLO),
            'dance': lambda: self.send_request(SPORT_API_ID_DANCE1),
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
        """
        Called every time a behaviour command arrives on /trigger_behaviour.
        Translates the string command into a Unitree Request and sends it.
        """
        raw_command = msg.data.strip()
        command = raw_command.lower()

        self.get_logger().info(f'Received command: "{raw_command}"')

        handler = self.command_handlers.get(command)
        if handler is None:
            self.get_logger().warn('Unknown command: "%s" — ignoring.' % command)
            return

        handler()

    def send_request(self, api_id: int, params: dict = None):
        """
        Builds a Unitree Request message and publishes it to the robot.
        api_id: the Unitree sport API ID (e.g. 1004 for StandUp)
        params: optional dict of parameters (used for Move commands)
        """
        req = self.build_request(api_id, params)
        self.request_pub.publish(req)
        self.get_logger().info(
            'SEND COMMAND REQUEST TO SPORT API api_id=%d, params=%s' % (api_id, params)
        )

    def build_request(
        self,
        api_id: int,
        params: dict = None,
        noreply: bool = False,
    ) -> Request:
        req = Request()

        # header fields aligned with the official ROS2 client and CLI examples
        req.header.identity.id = self.get_next_id()
        req.header.identity.api_id = int(api_id)
        req.header.lease.id = 0
        req.header.policy.priority = 0
        req.header.policy.noreply = bool(noreply)

        req.parameter = json.dumps(params) if params else ''
        req.binary = []

        return req

    def get_next_id(self) -> int:
        self._request_counter += 1
        return self._request_counter


def main(args=None):
    rclpy.init(args=args)
    node = SportClientWrapperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
