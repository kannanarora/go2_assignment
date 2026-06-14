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
SPORT_API_ID_MOVE          = 1008   # walk: needs vx, vy, vyaw params
SPORT_API_ID_SIT            = 1009
SPORT_API_ID_RISESIT        = 1010
SPORT_API_ID_HELLO          = 1016  # wave hello
SPORT_API_ID_BALANCE_STAND  = 1002
SPORT_API_ID_SWITCH_JOYSTICK = 1027
SPORT_API_ID_FREEWALK = 2045
SPORT_API_ID_DANCE1 = 1022          # Dance1 routine (Dance2 is 1023)


class SportClientWrapperNode(Node):
    def __init__(self):
        super().__init__('sport_client_wrapper_node')

        self.trigger_topic = '/trigger_behaviour'
        self.request_topic = '/api/sport/request'

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

        self.command_handlers = {
            'stand': lambda: self.send_request(SPORT_API_ID_STAND_UP),
            'balance_stand': lambda: self.send_request(SPORT_API_ID_BALANCE_STAND),
            'free_walk': lambda: self.send_request(SPORT_API_ID_FREEWALK),
            'lie_down': lambda: self.send_request(SPORT_API_ID_STAND_DOWN),
            'stop': lambda: self.send_request(SPORT_API_ID_STOP_MOVE),
            'sit': lambda: self.send_request(SPORT_API_ID_SIT),
            'rise_sit': lambda: self.send_request(SPORT_API_ID_RISESIT),
            'hello': lambda: self.send_request(SPORT_API_ID_HELLO),
            'dance': lambda: self.send_request(SPORT_API_ID_DANCE1),
            'walk': lambda: self.send_move_request(vx=0.5, vy=0.0, vyaw=0.0),
            'turn_left': lambda: self.send_move_request(vx=0.0, vy=0.0, vyaw=1),
            'turn_right': lambda: self.send_move_request(vx=0.0, vy=0.0, vyaw=-1),
            'joystick_on': lambda: self.send_request(
                SPORT_API_ID_SWITCH_JOYSTICK, {'flag': True}
            ),
            'joystick_off': lambda: self.send_request(
                SPORT_API_ID_SWITCH_JOYSTICK, {'flag': False}
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

        if command.startswith('move '):
            parts = raw_command.split()
            if len(parts) == 4:
                try:
                    vx = float(parts[1])
                    vy = float(parts[2])
                    vyaw = float(parts[3])
                    self.send_move_request(vx=vx, vy=vy, vyaw=vyaw)
                    return
                except ValueError:
                    self.get_logger().warn(
                        'Invalid move parameters: "%s"' % raw_command
                    )
                    return
            self.get_logger().warn('Move command must be: move vx vy vyaw')
            return

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
            'Sent request: api_id=%d, params=%s' % (api_id, params)
        )

    def send_move_request(self, vx: float, vy: float, vyaw: float):
        """
        Sends a Move command with velocity parameters.
        vx   = forward/backward speed (m/s),  positive = forward
        vy   = left/right speed (m/s),         positive = left
        vyaw = rotation speed (rad/s),          positive = turn left
        """
        params = {'x': float(vx), 'y': float(vy), 'z': float(vyaw)}
        self.send_request(SPORT_API_ID_MOVE, params)

    def build_request(self, api_id: int, params: dict = None) -> Request:
        req = Request()

        # header fields aligned with the official ROS2 client and CLI examples
        req.header.identity.id = self.get_next_id()
        req.header.identity.api_id = int(api_id)
        req.header.lease.id = 0
        req.header.policy.priority = 0
        req.header.policy.noreply = False

        req.parameter = json.dumps(params) if params else ''
        req.binary = []

        return req

    # Simple counter so each request gets a unique ID
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