#!/usr/bin/env python3

import math
import random
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


class WanderNode(Node):
    def __init__(self):
        super().__init__("wander_node")

        # All parameters
        self.trigger_topic = self.declare_parameter(
            "trigger_topic", "/trigger_behaviour"
        ).value
        self.cmd_vel_topic = self.declare_parameter("cmd_vel_topic", "/cmd_vel").value
        self.bark_topic = self.declare_parameter("bark_topic", "/bark").value

        self.forward_speed_mps = float(
            self.declare_parameter("forward_speed_mps", 0.3).value
        )
        self.turn_speed_radps = float(
            self.declare_parameter("turn_speed_radps", 1.26).value
        )
        self.min_turn_deg = float(self.declare_parameter("min_turn_deg", 35.0).value)
        self.max_turn_deg = float(self.declare_parameter("max_turn_deg", 160.0).value)
        self.min_walk_distance_m = float(
            self.declare_parameter("min_walk_distance_m", 0.1).value
        )
        self.max_walk_distance_m = float(
            self.declare_parameter("max_walk_distance_m", 0.25).value
        )
        self.stop_duration_s = float(
            self.declare_parameter("stop_duration_s", 0.4).value
        )
        self.trick_duration = float(
            self.declare_parameter("trick_duration_s", 5).value
        )
        self.command_rate_hz = float(
            self.declare_parameter("command_rate_hz", 10.0).value
        )
        self.log_rate_hz = float(self.declare_parameter("log_rate_hz", 1.0).value)
        self.startup_command = self.declare_parameter(
            "startup_command", "balance_stand"
        ).value
        self.phase = "startup"
        self.phase_end_time = time.monotonic() + 1.0
        self.turn_direction = 1.0
        self._last_command = None
        self._last_log_time = 0.0
        self.cmd_pub = self.create_publisher(String, self.trigger_topic, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.bark_pub = self.create_publisher(String, self.bark_topic, 10)

        # Actions: walk, turn, sit, stretch, bark, rise_sit

        # TODO take bark out of this?

        # Markov table
        self.transitions = {
            'sit': [('rise_sit', 0.75), ('sit', 0.25)],
            'rise_sit': [('walk', 0.25), ('turn', 0.25), ('stretch', 0.25), ('bark', 0.25)],
            'walk': [('walk', 0.2), ('turn', 0.3), ('stretch', 0.1), ('bark', 0.2), ('sit', 0.2)],
            'turn': [('walk', 0.3), ('turn', 0.2), ('stretch', 0.1), ('bark', 0.2), ('sit', 0.2)],
            'stretch': [('walk', 0.4), ('turn', 0.4),  ('bark', 0.2)],
            'bark': [('walk', 0.3), ('turn', 0.2), ('stretch', 0.1), ('bark', 0.2), ('sit', 0.2)],
        }

        # Tick timer
        timer_period = 1.0 / max(self.command_rate_hz, 1.0)
        self.timer = self.create_timer(timer_period, self.tick)

        if self.startup_command:
            self.publish_command(self.startup_command, force=True)

        self.get_logger().info(
            "WanderNode publishing velocity to %s, commands to %s"
            % (
                self.cmd_vel_topic,
                self.trigger_topic
            )
        )

    def tick(self):
        now = time.monotonic()
        if now >= self.phase_end_time:
            self.publish_move(0.0, 0.0)
            self.advance_phase()

        if self.phase == "walk":
            self.publish_move(self.forward_speed_mps, 0.0)
        elif self.phase == "turn":
            self.publish_move(0.0, self.turn_direction * self.current_turn_speed())
        # else:
        #     self.publish_move(0.0, 0.0)

    def advance_phase(self):
        if self.phase == "startup":
            self.start_random_turn()
            return

        # Weighted random choice from transitions[current]
        options = self.transitions.get(self.phase, [(self.phase, 1.0)])
        states, weights = zip(*options)
        total = sum(weights)

        # Check weights exsit
        if total <= 0:
            state = states[0]
        else:
            # Normalize probabilities and choose
            probs = [w / total for w in weights]
            state = random.choices(states, probs, k=1)[0]
            self.get_logger().info(f'NEXT STATE: {state}')

        if state == "turn":
            self.start_random_turn()
        elif state == "walk":
            self.start_random_walk()
        elif state == "sit":
            self.start_sit()
        elif state == "stretch":
            self.start_stretch()
        elif state == "rise_sit":
            self.start_rise_sit()
        elif state == "bark":
            self.start_bark()
        else:
            self.start_random_turn()

    def start_sit(self):
        self.publish_command('sit', force=True)
        duration = max(abs(self.trick_duration), 0.01)
        self.set_phase("sit", duration)
        return

    def start_rise_sit(self):
        self.publish_command('rise_sit', force=True)
        duration = max(abs(self.trick_duration), 0.01)
        self.set_phase("rise_sit", duration)
        return

    def start_stretch(self):
        self.publish_command('stretch', force=True)
        duration = max(abs(self.trick_duration), 0.01)
        self.set_phase("stretch", duration)
        return

    def start_bark(self):
        msg = String()
        msg.data = "bark"
        self.publish_command('bark/speak', force=True)
        
        self.get_logger().info('PUBLISHED BARK')
        duration = max(abs(self.trick_duration), 0.01)
        self.set_phase("bark", duration)
        return

    def start_random_turn(self):
        angle_rad = math.radians(random.uniform(self.min_turn_deg, self.max_turn_deg))
        self.turn_direction = random.choice([-1.0, 1.0])
        duration = angle_rad / max(abs(self.turn_speed_radps), 0.01)
        self.get_logger().info(f'TURNING')
        self.set_phase("turn", duration)

    def start_random_walk(self):
        distance = random.uniform(self.min_walk_distance_m, self.max_walk_distance_m)
        duration = distance / max(abs(self.forward_speed_mps), 0.01)
        self.get_logger().info(f'WALKING {distance}m AT SPEED {self.forward_speed_mps}')
        self.set_phase("walk", duration)

    def set_phase(self, phase: str, duration_s: float):
        self.phase = phase
        self.phase_end_time = time.monotonic() + max(duration_s, 0.0)

    def current_turn_speed(self) -> float:
        return self.turn_speed_radps

    # Publish a twist movement command
    def publish_move(self, vx: float, vyaw: float):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = 0.0
        msg.angular.z = float(vyaw)
        self.cmd_vel_pub.publish(msg)

    # Publish a non-move commands (startup, sit, bark, stretch)
    def publish_command(self, command: str, force: bool = False):
        if not force and command == self._last_command:
            return

        msg = String()
        msg.data = command
        self.cmd_pub.publish(msg)
        self._last_command = command

    def destroy_node(self):
        self.publish_move(0.0, 0.0)
        self.publish_command("stop", force=True)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WanderNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
