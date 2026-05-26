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

        self.speed = 0.3
        self.rate_hz = 10.0

        self.trigger_pub = self.create_publisher(String, '/trigger_behaviour', 10)
        self.cmd_sub = self.create_subscription(String, '/cmd_vel', self.cmd_vel_callback, 10)

        self._active = False
        self._command = (0.0, 0.0, 0.0)
        self._deadline = 0.0

        self.timer = self.create_timer(1.0 / max(self.rate_hz, 1.0), self.timer_callback)
        self.get_logger().info(
            f'MoveNode ready: listening on /cmd_vel, publishing to /trigger_behaviour '
            f'at {self.rate_hz}Hz'
        )

    def timer_callback(self):
        if not self._active:
            return

        now = time.monotonic()
        if now < self._deadline:
            vx, vy, vyaw = self._command
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
        self._active = False

    def cmd_vel_callback(self, msg: String):
        raw = msg.data.strip()
        parts = raw.split()

        # accept: forward 1.0, backward 1.0, left 1.0, right 1.0, turn_left 1.0, turn_right 1.0
        # accept: move vx vy vyaw distance
        if len(parts) == 2:
            direction = parts[0].lower()
            try:
                distance = float(parts[1])
            except ValueError:
                self.get_logger().warn('Invalid distance in /cmd_vel: %s' % raw)
                return

            vx, vy, vyaw = self.get_move_vector(direction)
            self.start_movement(vx, vy, vyaw, distance)
            return

        if len(parts) == 5 and parts[0].lower() == 'move':
            try:
                vx = float(parts[1])
                vy = float(parts[2])
                vyaw = float(parts[3])
                distance = float(parts[4])
            except ValueError:
                self.get_logger().warn('Invalid move params in /cmd_vel: %s' % raw)
                return

            self.start_movement(vx, vy, vyaw, distance)
            return

        self.get_logger().warn(
            'Unsupported /cmd_vel format: "%s"' % raw
        )

    def start_movement(self, vx: float, vy: float, vyaw: float, distance: float):
        if distance <= 0:
            self.get_logger().warn('Distance must be positive: %s' % distance)
            return

        duration = distance / max(abs(self.speed), 0.001)
        self._command = (vx, vy, vyaw)
        self._deadline = time.monotonic() + duration
        self._active = True
        self.get_logger().info(
            'Starting movement vx=%s vy=%s vyaw=%s for %.2f m (%.2f s)'
            % (vx, vy, vyaw, distance, duration)
        )

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
