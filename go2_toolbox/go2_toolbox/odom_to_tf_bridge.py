#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomToTFBridge(Node):
    def __init__(self):
        super().__init__("odom_to_tf_bridge")
        self.br = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.sub = self.create_subscription(Odometry, "/utlidar/robot_odom", self.cb, 10)
        self.logged = False

    def cb(self, msg):
        if not msg.header.frame_id:
            msg.header.frame_id = "odom"
        if not msg.child_frame_id:
            msg.child_frame_id = "base_link"

        self.odom_pub.publish(msg)

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id
        t.child_frame_id = msg.child_frame_id
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation

        self.br.sendTransform(t)

        if not self.logged:
            self.get_logger().info(f"Publishing TF {t.header.frame_id} -> {t.child_frame_id} from /utlidar/robot_odom and republishing /odom")
            self.logged = True


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTFBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()