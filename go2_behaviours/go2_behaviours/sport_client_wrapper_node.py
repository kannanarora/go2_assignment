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
SPORT_API_ID_HELLO          = 1016  # wave hello


class SportClientWrapperNode(Node):
    def __init__(self):
        super().__init__('sport_client_wrapper_node')

        # SUBSCRIBER: listens for behaviour commands from other nodes
        # Any node in the system can send a string here to make the robot move
        self.behaviour_sub = self.create_subscription(
            String,
            '/trigger_behaviour',       # topic your other nodes publish to
            self.behaviour_callback,    # function called when a message arrives
            10                          # queue size
        )

        # PUBLISHER: sends the actual command to the robot
        # The Go2 listens on /api/sport/request for all movement commands
        self.request_pub = self.create_publisher(
            Request,
            '/api/sport/request',
            10
        )

        self.get_logger().info('SportClientWrapperNode is ready.')
        self.get_logger().info('Listening on /trigger_behaviour ...')

    def behaviour_callback(self, msg: String):
        """
        Called every time a behaviour command arrives on /trigger_behaviour.
        Translates the string command into a Unitree Request and sends it.
        """
        command = msg.data.strip().lower()
        self.get_logger().info(f'Received command: "{command}"')

        # translation table
        if command == 'stand':
            self.send_request(SPORT_API_ID_STAND_UP)

        elif command == 'lie_down':
            # This is what the safety node will call when someone is too close
            self.send_request(SPORT_API_ID_STAND_DOWN)

        elif command == 'stop':
            self.send_request(SPORT_API_ID_STOP_MOVE)

        elif command == 'sit':
            self.send_request(SPORT_API_ID_SIT)

        elif command == 'hello':
            self.send_request(SPORT_API_ID_HELLO)

        elif command == 'walk':
            # Walk forward at 0.3 m/s, no sideways, no rotation
            # vx=forward speed, vy=sideways speed, vyaw=rotation speed
            self.send_move_request(vx=0.3, vy=0.0, vyaw=0.0)

        elif command == 'turn_left':
            self.send_move_request(vx=0.0, vy=0.0, vyaw=0.5)

        elif command == 'turn_right':
            self.send_move_request(vx=0.0, vy=0.0, vyaw=-0.5)

        else:
            self.get_logger().warn(f'Unknown command: "{command}" — ignoring.')

    def send_request(self, api_id: int, params: dict = None):
        """
        Builds a Unitree Request message and publishes it to the robot.
        api_id: the Unitree sport API ID (e.g. 1004 for StandUp)
        params: optional dict of parameters (used for Move commands)
        """
        req = Request()

        # the header tells the robot which action to perform
        req.header.identity.id = self.get_next_id()
        req.header.identity.api_id = api_id

        # some commands need extra parameters encoded as JSON
        if params:
            req.parameter = json.dumps(params)
        else:
            req.parameter = ''

        # send it to the robot
        self.request_pub.publish(req)
        self.get_logger().info(f'Sent request: api_id={api_id}, params={params}')

    def send_move_request(self, vx: float, vy: float, vyaw: float):
        """
        Sends a Move command with velocity parameters.
        vx   = forward/backward speed (m/s),  positive = forward
        vy   = left/right speed (m/s),         positive = left
        vyaw = rotation speed (rad/s),          positive = turn left
        """
        params = {'x': vx, 'y': vy, 'z': vyaw}
        self.send_request(SPORT_API_ID_MOVE, params)

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