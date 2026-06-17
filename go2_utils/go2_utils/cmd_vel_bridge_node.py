#!/usr/bin/env python3

import json
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from unitree_api.msg import Request

# Unitree Go2 Sport API IDs
SPORT_API_ID_MOVE = 1008
SPORT_API_ID_STOP_MOVE = 1003
SPORT_API_ID_SWITCH_JOYSTICK = 1027

class CmdVelBridgeNode(Node):
    def __init__(self):
        super().__init__("cmd_vel_bridge_node")

        self.cmd_vel_topic = self.declare_parameter("cmd_vel_topic", "/cmd_vel").value
        self.request_topic = self.declare_parameter("request_topic", "/api/sport/request").value

        # Hard speed limits for safety clamping
        self.max_linear_speed_mps = float(self.declare_parameter("max_linear_speed_mps", 0.70).value)
        self.max_lateral_speed_mps = float(self.declare_parameter("max_lateral_speed_mps", 0.35).value)
        self.max_yaw_speed_radps = float(self.declare_parameter("max_yaw_speed_radps", 1.40).value)
        
        self.auto_joystick_control = bool(self.declare_parameter("auto_joystick_control", True).value)

        # State tracking for joystick toggling
        self._joystick_disabled_for_move = False

        # Subscriber
        self.cmd_vel_sub = self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_vel_callback, 10)
        
        # Publisher
        self.request_pub = self.create_publisher(Request, self.request_topic, 10)

    def cmd_vel_callback(self, msg: Twist):
        vx = self.clamp(msg.linear.x, -self.max_linear_speed_mps, self.max_linear_speed_mps)
        vy = self.clamp(msg.linear.y, -self.max_lateral_speed_mps, self.max_lateral_speed_mps)
        vyaw = self.clamp(msg.angular.z, -self.max_yaw_speed_radps, self.max_yaw_speed_radps)

        is_moving = abs(vx) > 0.0 or abs(vy) > 0.0 or abs(vyaw) > 0.0

        if self.auto_joystick_control:
            if is_moving:
                self.disable_joystick_for_move()
            else:
                self.enable_joystick_after_move()

        self.publish_move_request(vx, vy, vyaw)

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

    def build_request(self, api_id: int, params: dict = None, noreply: bool = False) -> Request:
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

    def destroy_node(self):
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