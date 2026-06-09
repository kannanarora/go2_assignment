"""
Tier 1 - Reactive Layer: Safety Monitor Node
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String


class SafetyMonitorNode(Node):

    def __init__(self):
        super().__init__('safety_monitor_node')

        self.scan_topic = self.declare_parameter('scan_topic', '/front_scan').value
        self.override_topic = self.declare_parameter(
            'safety_override_topic', '/safety_override').value
        self.trigger_topic = self.declare_parameter(
            'trigger_behaviour_topic', '/trigger_behaviour').value
        self.trigger_threshold_m = float(
            self.declare_parameter('trigger_threshold_m', 0.15).value)
        self.clear_threshold_m = float(
            self.declare_parameter('clear_threshold_m', 0.20).value)
        self.safety_command = self.declare_parameter('safety_command', 'sit').value
        self.clear_command = self.declare_parameter('clear_command', 'rise_sit').value

        self._safety_active = False

        self.override_pub = self.create_publisher(Bool, self.override_topic, 10)
        self.trigger_pub = self.create_publisher(String, self.trigger_topic, 10)
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self._on_scan,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f'SafetyMonitorNode ready — scan={self.scan_topic}, '
            f'trigger<{self.trigger_threshold_m}m, '
            f'clear>{self.clear_threshold_m}m'
        )

    def _on_scan(self, msg: LaserScan):
        min_range = self._min_range(msg)
        if min_range is None:
            return

        if min_range < self.trigger_threshold_m and not self._safety_active:
            self._set_safety_active(True)
            self._send_command(self.safety_command)
            return

        if min_range > self.clear_threshold_m and self._safety_active:
            self._set_safety_active(False)
            self._send_command(self.clear_command)

    def _set_safety_active(self, active: bool):
        self._safety_active = active
        msg = Bool()
        msg.data = active
        self.override_pub.publish(msg)
        state = 'ACTIVE' if active else 'cleared'
        self.get_logger().info(f'Safety override {state}')

    def _send_command(self, command: str):
        msg = String()
        msg.data = command
        self.trigger_pub.publish(msg)
        self.get_logger().info(f'Sent safety command: {command}')

    def _min_range(self, msg: LaserScan):
        min_range = None
        for value in msg.ranges:
            if math.isinf(value) or math.isnan(value):
                continue
            if value < msg.range_min or value > msg.range_max:
                continue
            if min_range is None or value < min_range:
                min_range = value
        return min_range


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
