"""
Publishes a sit -> stand -> unlock gait -> rotate -> sit loop.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time


class SitStandLoopNode(Node):
    def __init__(self):
        super().__init__('sit_stand_loop_node')

        self.trigger_topic = '/trigger_behaviour'
        self.command_pub = self.create_publisher(String, self.trigger_topic, 10)

        self.sit_wait_s = float(self.declare_parameter('sit_wait_s', 3.0).value)
        self.stand_wait_s = float(self.declare_parameter('stand_wait_s', 3.0).value)
        self.rotate_command = self.declare_parameter('rotate_command', 'turn_left').value
        self.rotate_duration_s = float(
            self.declare_parameter('rotate_duration_s', 2.0).value
        )
        self.rotate_pulses = int(self.declare_parameter('rotate_pulses', 5).value)
        self.rotate_rate_hz = float(
            self.declare_parameter('rotate_rate_hz', 2.0).value
        )
        self.sit_after_wait_s = float(self.declare_parameter('sit_after_wait_s', 3.0).value)
        self.stop_wait_s = float(self.declare_parameter('stop_wait_s', 1.0).value)
        self.use_balance_stand = bool(
            self.declare_parameter('use_balance_stand', True).value
        )
        self.balance_wait_s = float(self.declare_parameter('balance_wait_s', 0.5).value)

        self._state = 'sit'
        self._state_deadline = 0.0
        self._next_rotate_publish = 0.0
        self._rotate_sent = 0

        tick_s = max(0.05, 1.0 / max(self.rotate_rate_hz, 1.0))
        self.timer = self.create_timer(tick_s, self.timer_callback)
        self.get_logger().info(
            'SitStandLoopNode ready. Publishing to %s.'
            % self.trigger_topic
        )

    def timer_callback(self):
        now = time.monotonic()

        if self._state == 'sit':
            self.publish_command('sit')
            self.get_logger().info('Sent command: sit')
            self._state = 'sit_wait'
            self._state_deadline = now + self.sit_wait_s
            return

        if self._state == 'sit_wait':
            if now >= self._state_deadline:
                self.publish_command('stand')
                self.get_logger().info('Sent command: stand')
                self._state = 'stand_wait'
                self._state_deadline = now + self.stand_wait_s
            return

        if self._state == 'stand_wait':
            if now >= self._state_deadline:
                if self.use_balance_stand:
                    self.publish_command('balance_stand')
                    self.get_logger().info('Sent command: balance_stand')
                    self._state = 'balance_wait'
                    self._state_deadline = now + self.balance_wait_s
                else:
                    self._state = 'rotate'
                    self._state_deadline = now + self.get_rotate_duration()
                    self._next_rotate_publish = 0.0
                    self._rotate_sent = 0
            return

        if self._state == 'balance_wait':
            if now >= self._state_deadline:
                self._state = 'rotate'
                self._state_deadline = now + self.get_rotate_duration()
                self._next_rotate_publish = 0.0
                self._rotate_sent = 0
            return

        if self._state == 'rotate':
            if self._rotate_sent >= max(self.rotate_pulses, 1):
                self.publish_command('stop')
                self.get_logger().info('Sent command: stop')
                self._state = 'stop_wait'
                self._state_deadline = now + self.stop_wait_s
                return

            if now >= self._state_deadline:
                self.publish_command('stop')
                self.get_logger().info('Sent command: stop')
                self._state = 'stop_wait'
                self._state_deadline = now + self.stop_wait_s
                return

            if now >= self._next_rotate_publish:
                command = str(self.rotate_command)
                self.publish_command(command)
                self.get_logger().info('Sent command: %s' % command)
                self._next_rotate_publish = now + (1.0 / max(self.rotate_rate_hz, 1.0))
                self._rotate_sent += 1
            return

        if self._state == 'stop_wait':
            if now >= self._state_deadline:
                self.publish_command('sit')
                self.get_logger().info('Sent command: sit')
                self._state = 'sit_after_wait'
                self._state_deadline = now + self.sit_after_wait_s
            return

        if self._state == 'sit_after_wait':
            if now >= self._state_deadline:
                self._state = 'sit'
            return

    def publish_command(self, command: str):
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)

    def get_rotate_duration(self) -> float:
        min_duration = self.rotate_pulses / max(self.rotate_rate_hz, 1.0)
        return max(self.rotate_duration_s, min_duration)


def main(args=None):
    rclpy.init(args=args)
    node = SitStandLoopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
