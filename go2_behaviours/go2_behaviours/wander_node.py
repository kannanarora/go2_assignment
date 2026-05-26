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

class WanderNode(Node):
    def __init__(self):
        super().__init__('wander_node')

        self.trigger_topic = '/trigger_behaviour'
        self.command_pub = self.create_publisher(String, self.trigger_topic, 10)
        self.cmd_vel_topic = '/cmd_vel'
        self.cmd_vel_pub = self.create_publisher(String, self.cmd_vel_topic, 10)

        # core timing params (hardcoded defaults)
        self.sit_wait_s = 3.0
        self.stand_wait_s = 1.0
        self.max_move_distance = 1.0
        self.move_speed = 0.3
        self.turn_speed = 1.0
        self.action_rate_hz = 1.0

        # commands
        self.move_commands = ['forward'] # TODO add more
        self.turn_commands = ['turn_left', 'turn_right']

        # compact Markov table
        self.transitions = {
            'sit': [('stand', 0.3), ('move', 0.4), ('sit', 0.3)],
            'stand': [('move', 0.5), ('sit', 0.2), ('stand', 0.3)],
            'move': [('move', 0.4), ('turn', 0.4), ('sit', 0.2)],
            'turn': [('move', 0.5), ('sit', 0.2), ('turn', 0.3)],
        }

        # state
        self._state = 'sit'
        self._deadline = 0.0

        tick_s = max(0.05, 1.0 / max(self.action_rate_hz, 1.0))
        self.timer = self.create_timer(tick_s, self.timer_callback)
        self.get_logger().info('WanderNode ready. Publishing to %s.' % self.trigger_topic)

    def timer_callback(self):
        now = time.monotonic()

        # Wait after stop -> then sit
        if self._state == 'stop_wait':
            if now >= self._deadline:
                self.publish_command('stand')
                self.get_logger().info('Sent command after moving: stand')
                self._state = 'stand'
                self._deadline = now + self.sit_wait_s
            return
        
        # If in sit, always stand up. If in stand, choose next action.
        if self._state in ('sit'):
            self.get_logger().info('Rise from sit')
            self.publish_command('rise_sit')
            self._state = 'stand'
            return

        # If in stand, decide next action using Markov transitions
        if self._state in ('stand'):
            next_state = self.choose_next_state(self._state)

            if next_state == 'sit':
                self.get_logger().info('Sent command: sit')
                self._state = 'sit'
                self._deadline = now + self.sit_wait_s
                return

            if next_state == 'stand':
                self.publish_command('stand')
                self.get_logger().info('Sent command: stand')
                self._state = 'stand'
                self._deadline = now + self.stand_wait_s
                return

            # move/turn: send a single /cmd_vel request and let MoveNode handle distance
            if next_state == 'move':
                direction = random.choice(self.move_commands)
                vel_cmd = self.build_cmd_vel(direction)
            else:
                direction = random.choice(self.turn_commands)
                vel_cmd = self.build_cmd_vel(direction)

            self.publish_cmd_vel(vel_cmd)
            self.get_logger().info('Sent cmd_vel: %s' % vel_cmd)
            self._state = 'stop_wait'
            self._deadline = now + self.stand_wait_s
            return

    def choose_next_state(self, current: str) -> str:
        # weighted random choice from transitions[current]
        options = self.transitions.get(current, [(current, 1.0)])
        states, weights = zip(*options)
        total = sum(weights)
        if total <= 0:
            return states[0]
        # normalize and choose
        probs = [w / total for w in weights]
        return random.choices(states, probs, k=1)[0]

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
    node = WanderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
