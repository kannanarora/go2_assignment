"""MoveNode

Move in a given direction for a given distance.
Uses the SportClientWrapperNode by publishing a repeated string command
on /trigger_behaviour containing the x,y, and z motion vector.
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MoveNode(Node):
    def __init__(self):
        super().__init__('move_node')

        self.direction = self.declare_parameter('direction', 'forward').value
        self.distance_m = float(self.declare_parameter('distance_m', 1.0).value)
        self.speed = float(self.declare_parameter('speed', 0.3).value)
        self.rate_hz = float(self.declare_parameter('rate_hz', 10.0).value)

        self.trigger_pub = self.create_publisher(String, '/trigger_behaviour', 10)

        self.start_time = time.monotonic()
        self.publish_duration = self.distance_m / max(abs(self.speed), 0.001)
        self.deadline = self.start_time + self.publish_duration
        self._done = False

        self.timer = self.create_timer(1.0 / max(self.rate_hz, 1.0), self.timer_callback)
        self.get_logger().info(
            f'MoveNode ready: direction={self.direction} distance={self.distance_m}m '
            f'speed={self.speed}m/s rate={self.rate_hz}Hz'
        )

    def timer_callback(self):
        if self._done:
            return

        now = time.monotonic()
        if now < self.deadline and self.distance_m > 0.0:
            vx, vy, vyaw = self.get_move_vector(self.direction)
            command = f'move {vx} {vy} {vyaw}'
            msg = String()
            msg.data = command
            self.trigger_pub.publish(msg)
            self.get_logger().info(f'Sent trigger command: {command}')
            return

        stop_msg = String()
        stop_msg.data = 'stop'
        self.trigger_pub.publish(stop_msg)
        self.get_logger().info('Target distance reached, sent stop.')
        self._done = True

    def get_move_vector(self, direction: str):
        direction = direction.strip().lower()
        if direction == 'forward':
            return self.speed, 0.0, 0.0
        if direction == 'backward':
            return -self.speed, 0.0, 0.0
        if direction == 'left':
            return 0.0, self.speed, 0.0
        if direction == 'right':
            return 0.0, -self.speed, 0.0
        if direction == 'turn_left':
            return 0.0, 0.0, 1.0
        if direction == 'turn_right':
            return 0.0, 0.0, -1.0

        self.get_logger().warn(
            'Unknown direction "%s", defaulting to forward.' % direction
        )
        return self.speed, 0.0, 0.0


def main(args=None):
    rclpy.init(args=args)
    node = MoveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
