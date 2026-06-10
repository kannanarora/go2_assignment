"""WanderNode

Publishes random walk behaviour using a simple Markov decision process.
The node will randomly choose to `sit`, `stand`, `move` (forward/backward),
or `turn` (left/right) according to configurable transition probabilities.
"""

import random
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')

        self.trigger_topic = '/trigger_behaviour'
        self.command_pub = self.create_publisher(String, self.trigger_topic, 10)
        self.cmd_vel_topic = '/cmd_vel'
        self.cmd_vel_pub = self.create_publisher(String, self.cmd_vel_topic, 10)
        self.command_list = [
            'forward 1.0',
            'balance_stand',
            'sit',
            'rise_sit',
            'backward 1.0',
            'balance_stand',
            'sit',
            'forward 1.0',
            'backward 1.0',
        ]
        self.command_index = 0

        # how often to check if we should change action (seconds)
        tick_s = 8.0
        self.timer = self.create_timer(tick_s, self.timer_callback)
        self.get_logger().info('TestNode ready. Publishing to %s.' % self.trigger_topic)

    def timer_callback(self):
        self.get_logger().info('New command incoming')
        command = self.command_list[self.command_index]
        self.command_index = (self.command_index + 1) % len(self.command_list)
        self.publish_command(command)

    def publish_command(self, command: str):
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)

    def publish_cmd_vel(self, command: str):
        msg = String()
        msg.data = command
        self.cmd_vel_pub.publish(msg)

    def build_cmd_vel(self, direction: str) -> str:
        if direction == 'forward':
            return f'move {self.move_speed} 0.0 0.0 {self.max_move_distance}'
        if direction == 'backward':
            return f'move {-self.move_speed} 0.0 0.0 {self.max_move_distance}'
        if direction == 'turn_left':
            return f'move 0.0 0.0 {self.turn_speed} {self.max_move_distance}'
        if direction == 'turn_right':
            return f'move 0.0 0.0 {-self.turn_speed} {self.max_move_distance}'
        return f'move {self.move_speed} 0.0 0.0 {self.max_move_distance}'

def main(args=None):
    rclpy.init(args=args)
    node = TestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
