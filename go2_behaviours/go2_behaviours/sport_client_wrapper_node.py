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
import math
import time

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


class SportClientWrapperNode(Node):
    def __init__(self):
        super().__init__('sport_client_wrapper_node')

        self.trigger_topic = self.declare_parameter(
            'trigger_topic', '/trigger_behaviour'
        ).value
        self.request_topic = self.declare_parameter(
            'request_topic', '/api/sport/request'
        ).value
        self.move_publish_rate_hz = float(
            self.declare_parameter('move_publish_rate_hz', 50.0).value
        )
        self.max_linear_speed_mps = float(
            self.declare_parameter('max_linear_speed_mps', 0.30).value
        )
        self.max_lateral_speed_mps = float(
            self.declare_parameter('max_lateral_speed_mps', 0.20).value
        )
        self.max_yaw_speed_radps = float(
            self.declare_parameter('max_yaw_speed_radps', 0.55).value
        )
        self.linear_accel_mps2 = float(
            self.declare_parameter('linear_accel_mps2', 0.35).value
        )
        self.yaw_accel_radps2 = float(
            self.declare_parameter('yaw_accel_radps2', 0.70).value
        )
        self.move_command_timeout_s = float(
            self.declare_parameter('move_command_timeout_s', 0.6).value
        )
        self.auto_joystick_control = bool(
            self.declare_parameter('auto_joystick_control', True).value
        )

        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_vyaw = 0.0
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_vyaw = 0.0
        self._last_move_command_time = 0.0
        self._last_move_publish_time = time.monotonic()
        self._sent_zero_after_stop = True
        self._joystick_disabled_for_move = False

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
            'walk': lambda: self.set_move_target(vx=0.20, vy=0.0, vyaw=0.0),
            'turn_left': lambda: self.set_move_target(vx=0.0, vy=0.0, vyaw=0.35),
            'turn_right': lambda: self.set_move_target(vx=0.0, vy=0.0, vyaw=-0.35),
            'joystick_on': lambda: self.send_request(
                SPORT_API_ID_SWITCH_JOYSTICK, {'data': True}
            ),
            'joystick_off': lambda: self.send_request(
                SPORT_API_ID_SWITCH_JOYSTICK, {'data': False}
            ),
        }

        timer_period = 1.0 / max(self.move_publish_rate_hz, 1.0)
        self.move_timer = self.create_timer(timer_period, self.publish_smoothed_move)

        self.get_logger().info('SportClientWrapperNode is ready.')
        self.get_logger().info('Listening on %s ...' % self.trigger_topic)
        self.get_logger().info(
            'Smooth move loop: %.1f Hz, max x=%.2fm/s, max yaw=%.2frad/s'
            % (
                self.move_publish_rate_hz,
                self.max_linear_speed_mps,
                self.max_yaw_speed_radps,
            )
        )

    def behaviour_callback(self, msg: String):
        """
        Called every time a behaviour command arrives on /trigger_behaviour.
        Translates the string command into a Unitree Request and sends it.
        """
        raw_command = msg.data.strip()
        command = raw_command.lower()

        if command.startswith('move '):
            self.get_logger().debug(f'Received command: "{raw_command}"')
            parts = raw_command.split()
            if len(parts) == 4:
                try:
                    vx = float(parts[1])
                    vy = float(parts[2])
                    vyaw = float(parts[3])
                    self.set_move_target(vx=vx, vy=vy, vyaw=vyaw)
                    return
                except ValueError:
                    self.get_logger().warn(
                        'Invalid move parameters: "%s"' % raw_command
                    )
                    return
            self.get_logger().warn('Move command must be: move vx vy vyaw')
            return

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
        if api_id != SPORT_API_ID_SWITCH_JOYSTICK:
            self.stop_move_target()

        req = self.build_request(api_id, params)
        self.request_pub.publish(req)
        self.get_logger().info(
            'Sent request: api_id=%d, params=%s' % (api_id, params)
        )

    def set_move_target(self, vx: float, vy: float, vyaw: float):
        """
        Sets the desired movement. A 50 Hz timer ramps current velocity toward
        this target and publishes the Unitree Move requests smoothly.
        vx   = forward/backward speed (m/s),  positive = forward
        vy   = left/right speed (m/s),         positive = left
        vyaw = rotation speed (rad/s),          positive = turn left
        """
        self.target_vx = self.clamp(
            vx,
            -self.max_linear_speed_mps,
            self.max_linear_speed_mps,
        )
        self.target_vy = self.clamp(
            vy,
            -self.max_lateral_speed_mps,
            self.max_lateral_speed_mps,
        )
        self.target_vyaw = self.clamp(
            vyaw,
            -self.max_yaw_speed_radps,
            self.max_yaw_speed_radps,
        )
        self._last_move_command_time = time.monotonic()
        self._sent_zero_after_stop = False

        if self.auto_joystick_control and self.target_is_nonzero():
            self.disable_joystick_for_move()

    def stop_move_target(self):
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_vyaw = 0.0
        self._sent_zero_after_stop = False

    def target_is_nonzero(self) -> bool:
        return any(
            abs(value) > 0.0
            for value in (self.target_vx, self.target_vy, self.target_vyaw)
        )

    def publish_smoothed_move(self):
        now = time.monotonic()
        dt = max(now - self._last_move_publish_time, 0.0)
        self._last_move_publish_time = now

        if (
            self.move_command_timeout_s > 0.0
            and self._last_move_command_time > 0.0
            and now - self._last_move_command_time > self.move_command_timeout_s
        ):
            self.target_vx = 0.0
            self.target_vy = 0.0
            self.target_vyaw = 0.0

        self.current_vx = self.step_toward(
            self.current_vx,
            self.target_vx,
            self.linear_accel_mps2 * dt,
        )
        self.current_vy = self.step_toward(
            self.current_vy,
            self.target_vy,
            self.linear_accel_mps2 * dt,
        )
        self.current_vyaw = self.step_toward(
            self.current_vyaw,
            self.target_vyaw,
            self.yaw_accel_radps2 * dt,
        )

        self.current_vx = self.zero_small(self.current_vx)
        self.current_vy = self.zero_small(self.current_vy)
        self.current_vyaw = self.zero_small(self.current_vyaw)

        moving = any(
            abs(value) > 0.0
            for value in (
                self.current_vx,
                self.current_vy,
                self.current_vyaw,
                self.target_vx,
                self.target_vy,
                self.target_vyaw,
            )
        )

        if not moving and self._sent_zero_after_stop:
            return

        self.publish_move_request(
            vx=self.current_vx,
            vy=self.current_vy,
            vyaw=self.current_vyaw,
        )

        if not moving:
            self._sent_zero_after_stop = True
            self.enable_joystick_after_move()

    def publish_move_request(self, vx: float, vy: float, vyaw: float):
        params = {'x': float(vx), 'y': float(vy), 'z': float(vyaw)}
        req = self.build_request(SPORT_API_ID_MOVE, params, noreply=True)
        self.request_pub.publish(req)

    def disable_joystick_for_move(self):
        if self._joystick_disabled_for_move:
            return

        self.send_request(SPORT_API_ID_SWITCH_JOYSTICK, {'data': False})
        self._joystick_disabled_for_move = True

    def enable_joystick_after_move(self):
        if not self._joystick_disabled_for_move:
            return

        self.send_request(SPORT_API_ID_SWITCH_JOYSTICK, {'data': True})
        self._joystick_disabled_for_move = False

    def clamp(self, value: float, low: float, high: float) -> float:
        return max(low, min(high, float(value)))

    def step_toward(self, current: float, target: float, max_delta: float) -> float:
        if max_delta <= 0.0:
            return target

        delta = target - current
        if abs(delta) <= max_delta:
            return target

        return current + math.copysign(max_delta, delta)

    def zero_small(self, value: float) -> float:
        if abs(value) < 1e-4:
            return 0.0
        return value

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
