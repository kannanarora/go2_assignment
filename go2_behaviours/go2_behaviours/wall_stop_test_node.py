"""
Walks forward and stops when a wall is within 0.5 m (front LiDAR distance)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String

STOP_DISTANCE = 0.5 #m


class WallStopTestNode(Node):

    def __init__(self):
        super().__init__('wall_stop_test_node')

        self.stopped  = False
        self._walking = False

        self.cmd_pub = self.create_publisher(String, '/trigger_behaviour', 10)

        # /utlidar/range_info gives front obstacle distance as point.x
        # qos_profile_sensor_data is required for LiDAR topics on the Go2
        self.create_subscription(
            PointStamped,
            '/utlidar/range_info',
            self._on_range,
            qos_profile_sensor_data,
        )

        self._start_timer = self.create_timer(2.0, self._start_walking) # wait 2s

        # Send 'walk' at 10 Hz since firmware stops the robot without continuous commands
        self.create_timer(0.1, self._keep_walking)

        self.get_logger().info(f'WallStopTestNode is ready, will stop at {STOP_DISTANCE} m')

    def _start_walking(self):
        self.get_logger().info('Starting walk...')
        self._walking = True
        self._start_timer.cancel()

    def _keep_walking(self):
        if self._walking and not self.stopped:
            self.cmd_pub.publish(String(data='walk'))

    def _on_range(self, msg: PointStamped):
        if self.stopped:
            return

        front_dist = float(msg.point.x)
        if front_dist <= 0.0:
            return

        self.get_logger().info(
            f'Front distance: {front_dist:.2f} m',
            throttle_duration_sec=1.0,
        )

        if front_dist < STOP_DISTANCE:
            self.get_logger().warn(f'Wall at {front_dist:.2f} m and STOPPING!')
            self.cmd_pub.publish(String(data='stop'))
            self.stopped = True


def main(args=None):
    rclpy.init(args=args)
    node = WallStopTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
