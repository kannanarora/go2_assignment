#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomToTFBridge(Node):
    def __init__(self):
        super().__init__("odom_to_tf_bridge")

        self.declare_parameter("input_odom_topic", "/utlidar/robot_odom")
        self.declare_parameter("output_odom_topic", "/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_odom", True)
        self.declare_parameter("publish_tf", True)

        self.input_odom_topic = self.get_parameter("input_odom_topic").value
        self.output_odom_topic = self.get_parameter("output_odom_topic").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.publish_odom = self.get_parameter("publish_odom").value
        self.publish_tf = self.get_parameter("publish_tf").value

        self.br = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, self.output_odom_topic, 10)
        self.sub = self.create_subscription(Odometry, self.input_odom_topic, self.cb, 10)

        self.logged = False

    def cb(self, msg):
        frame_id = msg.header.frame_id or self.odom_frame
        child_frame_id = msg.child_frame_id or self.base_frame

        msg.header.frame_id = frame_id
        msg.child_frame_id = child_frame_id

        if self.publish_odom:
            self.odom_pub.publish(msg)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = msg.header.stamp
            t.header.frame_id = frame_id
            t.child_frame_id = child_frame_id
            t.transform.translation.x = msg.pose.pose.position.x
            t.transform.translation.y = msg.pose.pose.position.y
            t.transform.translation.z = msg.pose.pose.position.z
            t.transform.rotation = msg.pose.pose.orientation
            self.br.sendTransform(t)

        if not self.logged:
            self.get_logger().info(
                f"Listening to {self.input_odom_topic}, publishing {self.output_odom_topic}, "
                f"TF {frame_id} -> {child_frame_id}"
            )
            self.logged = True


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTFBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()