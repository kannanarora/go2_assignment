#!/usr/bin/env python3
# THIS WILL BE USED AS THE SEQUENCING LAYER

import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String
from unitree_api.msg import Request


SPORT_API_ID_MOVE = 1008
SPORT_API_ID_STOP_MOVE = 1003
SPORT_API_ID_SWITCH_JOYSTICK = 1027


class CmdVelBridgeNode(Node):
    def __init__(self):
        super().__init__("cmd_vel_bridge_node")

        self.trick_commands = ["stop", "sit", "lie_down", "stand_down", "rise_sit"]

        self.cmd_vel_topic = self.declare_parameter("cmd_vel_topic", "/cmd_vel").value
        self.request_topic = self.declare_parameter(
            "request_topic", "/api/sport/request"
        ).value
        self.trigger_topic = self.declare_parameter(
            "trigger_topic", "/trigger_behaviour"
        ).value

        self.publish_rate_hz = float(
            self.declare_parameter("publish_rate_hz", 50.0).value
        )
        self.max_linear_speed_mps = float(
            self.declare_parameter("max_linear_speed_mps", 0.70).value
        )
        self.max_lateral_speed_mps = float(
            self.declare_parameter("max_lateral_speed_mps", 0.35).value
        )
        self.max_yaw_speed_radps = float(
            self.declare_parameter("max_yaw_speed_radps", 1.40).value
        )
        self.linear_accel_mps2 = float(
            self.declare_parameter("linear_accel_mps2", 0.90).value
        )
        self.yaw_accel_radps2 = float(
            self.declare_parameter("yaw_accel_radps2", 2.00).value
        )
        self.cmd_vel_timeout_s = float(
            self.declare_parameter("cmd_vel_timeout_s", 0.6).value
        )
        self.auto_joystick_control = bool(
            self.declare_parameter("auto_joystick_control", True).value
        )

        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_vyaw = 0.0
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_vyaw = 0.0
        self._last_cmd_vel_time = 0.0
        self._last_publish_time = time.monotonic()
        self._sent_zero_after_stop = True
        self._joystick_disabled_for_move = False

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            10,
        )
        self.trigger_sub = self.create_subscription(
            String,
            self.trigger_topic,
            self.trigger_callback,
            10,
        )
        self.request_pub = self.create_publisher(Request, self.request_topic, 10)

        timer_period = 1.0 / max(self.publish_rate_hz, 1.0)
        self.timer = self.create_timer(timer_period, self.publish_smoothed_move)

        self.get_logger().info(
            "CmdVelBridgeNode: %s -> %s at %.1f Hz "
            "(max x=%.2fm/s, y=%.2fm/s, yaw=%.2frad/s)"
            % (
                self.cmd_vel_topic,
                self.request_topic,
                self.publish_rate_hz,
                self.max_linear_speed_mps,
                self.max_lateral_speed_mps,
                self.max_yaw_speed_radps,
            )
        )

    def cmd_vel_callback(self, msg: Twist):
        self.target_vx = self.clamp(
            msg.linear.x,
            -self.max_linear_speed_mps,
            self.max_linear_speed_mps,
        )
        self.target_vy = self.clamp(
            msg.linear.y,
            -self.max_lateral_speed_mps,
            self.max_lateral_speed_mps,
        )
        self.target_vyaw = self.clamp(
            msg.angular.z,
            -self.max_yaw_speed_radps,
            self.max_yaw_speed_radps,
        )
        self._last_cmd_vel_time = time.monotonic()
        self._sent_zero_after_stop = False

        if self.auto_joystick_control and self.target_is_nonzero():
            self.disable_joystick_for_move()

    # If another command is detected, immediately stop
    def trigger_callback(self, msg: String):
        command = msg.data.strip().lower()
        if command in self.trick_commands:
            self.stop_target()

    # Timer to continually publish movement command to the sports api
    def publish_smoothed_move(self):
        now = time.monotonic()
        dt = max(now - self._last_publish_time, 0.0)
        self._last_publish_time = now

        # TODO start free avoid with small pause on first publish

        if (
            self.cmd_vel_timeout_s > 0.0
            and self._last_cmd_vel_time > 0.0
            and now - self._last_cmd_vel_time > self.cmd_vel_timeout_s
        ):
            self.stop_target()

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

    def stop_target(self):
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_vyaw = 0.0
        self._sent_zero_after_stop = False

    def target_is_nonzero(self) -> bool:
        return any(
            abs(value) > 0.0
            for value in (self.target_vx, self.target_vy, self.target_vyaw)
        )

    # Send single movement request to sports api
    def publish_move_request(self, vx: float, vy: float, vyaw: float):
        params = {"x": float(vx), "y": float(vy), "z": float(vyaw)}
        req = self.build_request(SPORT_API_ID_MOVE, params, noreply=True)
        self.request_pub.publish(req)

    def disable_joystick_for_move(self):
        if self._joystick_disabled_for_move:
            return

        self.publish_request(SPORT_API_ID_SWITCH_JOYSTICK, {"data": False})
        self._joystick_disabled_for_move = True

    def enable_joystick_after_move(self):
        if not self._joystick_disabled_for_move:
            return

        self.publish_request(SPORT_API_ID_SWITCH_JOYSTICK, {"data": True})
        self._joystick_disabled_for_move = False

    def publish_request(self, api_id: int, params: dict = None, noreply: bool = False):
        req = self.build_request(api_id, params, noreply)
        self.request_pub.publish(req)

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
        req.parameter = json.dumps(params) if params else ""
        req.binary = []
        return req

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

    def destroy_node(self):
        self.stop_target()
        self.publish_request(SPORT_API_ID_STOP_MOVE)
        super().destroy_node()

    _request_counter = 0

    def get_next_id(self) -> int:
        CmdVelBridgeNode._request_counter += 1
        return CmdVelBridgeNode._request_counter


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
