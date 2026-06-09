"""
Sits the robot when an obstacle is too close in the front scan.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class FrontSafetySitNode(Node):
    def __init__(self):
        super().__init__('front_safety_sit_node')

        self.scan_topic = self.declare_parameter('scan_topic', '/front_scan').value
        self.trigger_topic = self.declare_parameter('trigger_topic', '/trigger_behaviour').value
        self.sit_threshold_m = float(self.declare_parameter('sit_threshold_m', 0.15).value)
        self.clear_threshold_m = float(
            self.declare_parameter('clear_threshold_m', 0.20).value
        )
        self.sit_command = self.declare_parameter('sit_command', 'sit').value
        self.stand_command = self.declare_parameter('stand_command', 'rise_sit').value

        self._is_sitting = False

        self.cmd_pub = self.create_publisher(String, self.trigger_topic, 10)
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            'FrontSafetySitNode listening on %s (sit<%.2fm, clear>%.2fm).'
            % (self.scan_topic, self.sit_threshold_m, self.clear_threshold_m)
        )

    def scan_callback(self, msg: LaserScan):
        min_range = self.get_min_range(msg)
        if min_range is None:
            return

        if min_range < self.sit_threshold_m and not self._is_sitting:
            self.publish_command(self.sit_command)
            self._is_sitting = True
            return

        if min_range > self.clear_threshold_m and self._is_sitting:
            self.publish_command(self.stand_command)
            self._is_sitting = False

    def get_min_range(self, msg: LaserScan):
        min_range = None
        for value in msg.ranges:
            if math.isinf(value) or math.isnan(value):
                continue
            if value < msg.range_min or value > msg.range_max:
                continue
            if min_range is None or value < min_range:
                min_range = value
        return min_range

    def publish_command(self, command: str):
        msg = String()
        msg.data = command
        self.cmd_pub.publish(msg)
        self.get_logger().info('Sent command: %s' % command)


def main(args=None):
    rclpy.init(args=args)
    node = FrontSafetySitNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
