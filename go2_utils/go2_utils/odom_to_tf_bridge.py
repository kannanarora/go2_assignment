#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def quaternion_from_yaw(yaw):
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    return qz, qw


class OdomToTFBridge(Node):
    def __init__(self):
        super().__init__("odom_to_tf_bridge")
        self.br = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.sub = self.create_subscription(Odometry, "/utlidar/robot_odom", self.cb, qos)
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

        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        qz, qw = quaternion_from_yaw(yaw)

        t_flat = TransformStamped()
        t_flat.header.stamp = msg.header.stamp
        t_flat.header.frame_id = "odom"
        t_flat.child_frame_id = "base_footprint"

        t_flat.transform.translation.x = msg.pose.pose.position.x
        t_flat.transform.translation.y = msg.pose.pose.position.y

        # Start with robot base z. If height filtering looks weird, change this to 0.0.
        t_flat.transform.translation.z = 0.0

        t_flat.transform.rotation.x = 0.0
        t_flat.transform.rotation.y = 0.0
        t_flat.transform.rotation.z = qz
        t_flat.transform.rotation.w = qw

        self.br.sendTransform(t_flat)

        if not self.logged:
            self.get_logger().info(
                f"Publishing TF {t.header.frame_id} -> {t.child_frame_id}, odom -> base_footprint, and republishing /odom with incoming Go2 timestamps"
            )
            self.logged = True


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTFBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()