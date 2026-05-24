"""
Publishes a stand -> rotate -> lie-down loop to the sport client wrapper.
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

        self.stand_wait_s = float(self.declare_parameter('stand_wait_s', 3.0).value)
        self.rotate_command = self.declare_parameter('rotate_command', 'turn_left').value
        self.rotate_duration_s = float(
            self.declare_parameter('rotate_duration_s', 2.0).value
        )
        self.rotate_rate_hz = float(
            self.declare_parameter('rotate_rate_hz', 5.0).value
        )
        self.lie_down_wait_s = float(self.declare_parameter('lie_down_wait_s', 3.0).value)
        self.stop_wait_s = float(self.declare_parameter('stop_wait_s', 1.0).value)
        self.use_balance_stand = bool(
            self.declare_parameter('use_balance_stand', False).value
        )
        self.enable_joystick_on = bool(
            self.declare_parameter('enable_joystick_on', True).value
        )

        self._state = 'stand'
        self._state_deadline = 0.0
        self._next_rotate_publish = 0.0

        tick_s = max(0.05, 1.0 / max(self.rotate_rate_hz, 1.0))
        self.timer = self.create_timer(tick_s, self.timer_callback)
        self.get_logger().info(
            'SitStandLoopNode ready. Publishing to %s.'
            % self.trigger_topic
        )

    def timer_callback(self):
        now = time.monotonic()

        if self._state == 'stand':
            self.publish_command('stand')
            self.get_logger().info('Sent command: stand')
            if self.use_balance_stand:
                self.publish_command('balance_stand')
                self.get_logger().info('Sent command: balance_stand')
            self._state = 'stand_wait'
            self._state_deadline = now + self.stand_wait_s
            return

        if self._state == 'stand_wait':
            if now >= self._state_deadline:
                self._state = 'rotate'
                self._state_deadline = now + self.rotate_duration_s
                self._next_rotate_publish = 0.0
            return

        if self._state == 'rotate':
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
            return

        if self._state == 'stop_wait':
            if now >= self._state_deadline:
                self.publish_command('lie_down')
                self.get_logger().info('Sent command: lie_down')
                self._state = 'lie_down_wait'
                self._state_deadline = now + self.lie_down_wait_s
            return

        if self._state == 'lie_down_wait':
            if now >= self._state_deadline:
                if self.enable_joystick_on:
                    self.publish_command('joystick_on')
                    self.get_logger().info('Sent command: joystick_on')
                self._state = 'stand'
            return

    def publish_command(self, command: str):
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SitStandLoopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
