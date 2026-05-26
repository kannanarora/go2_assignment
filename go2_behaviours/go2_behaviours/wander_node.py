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

        # core timing params (hardcoded defaults)
        self.sit_wait_s = 3.0
        self.stand_wait_s = 1.0
        self.action_duration_s = 2.0
        self.action_pulses = 4
        self.action_rate_hz = 2.0

        # commands
        self.move_commands = ['walk'] # TODO add more
        self.turn_commands = ['turn_left', 'turn_right']

        # compact Markov table (same structure, easier to read)
        self.transitions = {
            'sit': [('stand', 0.3), ('move', 0.4), ('sit', 0.3)],
            'stand': [('move', 0.5), ('sit', 0.2), ('stand', 0.3)],
            'move': [('move', 0.4), ('turn', 0.4), ('sit', 0.2)],
            'turn': [('move', 0.5), ('sit', 0.2), ('turn', 0.3)],
        }

        # state
        self._state = 'sit'
        self._deadline = 0.0
        self._next_pub = 0.0
        self._sent = 0
        self._target = 0
        self._cmd = ''

        tick_s = max(0.05, 1.0 / max(self.action_rate_hz, 1.0))
        self.timer = self.create_timer(tick_s, self.timer_callback)
        self.get_logger().info('WanderNode ready. Publishing to %s.' % self.trigger_topic)

    def timer_callback(self):
        now = time.monotonic()

        # Active action states that publish repeated commands: 'move' and 'turn'
        if self._state in ('move', 'turn'):
            if self._sent >= max(self._target, 1) or now >= self._deadline:
                self.publish_command('stop')
                self.get_logger().info('Sent command: stop')
                self._state = 'stop_wait'
                self._deadline = now + self.stand_wait_s
                return

            if now >= self._next_pub:
                self.publish_command(self._cmd)
                self.get_logger().info('Sent command: %s' % self._cmd)
                self._next_pub = now + (1.0 / max(self.action_rate_hz, 1.0))
                self._sent += 1
            return

        # Wait after stop -> then sit
        if self._state == 'stop_wait':
            if now >= self._deadline:
                self.publish_command('sit')
                self.get_logger().info('Sent command: sit')
                self._state = 'sit'
                self._deadline = now + self.sit_wait_s
            return

        # If in sit or stand, decide next action using Markov transitions
        if self._state in ('sit', 'stand'):
            next_state = self.choose_next_state(self._state)
            if next_state == 'sit':
                self.publish_command('sit')
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

            # move/turn: choose specific command and set up repeated publishes
            if next_state == 'move':
                cmd = random.choice(self.move_commands)
            else:
                cmd = random.choice(self.turn_commands)

            self._cmd = cmd
            self._sent = 0
            self._target = max(self.action_pulses, 1)
            self._next_pub = now
            self._deadline = now + self.action_duration_s
            self._state = next_state
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

def main(args=None):
    rclpy.init(args=args)
    node = WanderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
