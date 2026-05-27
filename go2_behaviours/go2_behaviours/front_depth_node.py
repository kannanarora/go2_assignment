"""
Publishes front obstacle distance from Unitree's range_info topic.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32


class FrontDepthNode(Node):
    def __init__(self):
        super().__init__('front_depth_node')

        self.range_topic = self.declare_parameter(
            'range_topic', '/utlidar/range_info'
        ).value

        self.depth_pub = self.create_publisher(Float32, 'front_depth', 10)
        self.range_sub = self.create_subscription(
            PointStamped,
            self.range_topic,
            self.range_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            'FrontDepthNode listening on %s.'
            % self.range_topic
        )

    def range_callback(self, msg: PointStamped):
        out = Float32()
        out.data = float(msg.point.x)
        self.depth_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = FrontDepthNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
