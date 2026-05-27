"""
Sits the robot when an obstacle is too close in the front range info.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String


class FrontSafetySitNode(Node):
    def __init__(self):
        super().__init__('front_safety_sit_node')

        self.range_topic = self.declare_parameter(
            'range_topic', '/utlidar/range_info'
        ).value
        self.trigger_topic = self.declare_parameter('trigger_topic', '/trigger_behaviour').value
        self.sit_threshold_m = float(self.declare_parameter('sit_threshold_m', 0.8).value)
        self.clear_threshold_m = float(
            self.declare_parameter('clear_threshold_m', 0.8).value
        )
        self.sit_command = self.declare_parameter('sit_command', 'sit').value
        self.stand_command = self.declare_parameter('stand_command', 'rise_sit').value

        if self.clear_threshold_m <= self.sit_threshold_m:
            self.clear_threshold_m = self.sit_threshold_m + 0.1
            self.get_logger().warn(
                'clear_threshold_m must be > sit_threshold_m; adjusted to %.2f'
                % self.clear_threshold_m
            )

        self._is_sitting = False

        self.cmd_pub = self.create_publisher(String, self.trigger_topic, 10)
        self.range_sub = self.create_subscription(
            PointStamped,
            self.range_topic,
            self.range_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            'FrontSafetySitNode listening on %s (sit<%.2fm, clear>%.2fm).'
            % (self.range_topic, self.sit_threshold_m, self.clear_threshold_m)
        )

    def range_callback(self, msg: PointStamped):
        front_range = float(msg.point.x)

        if not math.isfinite(front_range) or front_range <= 0.0:
            return

        if front_range < self.sit_threshold_m and not self._is_sitting:
            self.publish_command(self.sit_command)
            self._is_sitting = True
            return

        if front_range > self.clear_threshold_m and self._is_sitting:
            self.publish_command(self.stand_command)
            self._is_sitting = False

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
