#!/usr/bin/env python3

import math
import random
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class WanderNode(Node):
    def __init__(self):
        super().__init__("wander_node")

        # All parameters
        self.scan_topic = self.declare_parameter("scan_topic", "/front_scan").value
        self.trigger_topic = self.declare_parameter(
            "trigger_topic", "/trigger_behaviour"
        ).value
        self.cmd_vel_topic = self.declare_parameter("cmd_vel_topic", "/cmd_vel").value

        self.forward_speed_mps = float(
            self.declare_parameter("forward_speed_mps", 0.66).value
        )
        self.turn_speed_radps = float(
            self.declare_parameter("turn_speed_radps", 1.26).value
        )
        self.min_turn_deg = float(self.declare_parameter("min_turn_deg", 35.0).value)
        self.max_turn_deg = float(self.declare_parameter("max_turn_deg", 160.0).value)
        self.min_walk_distance_m = float(
            self.declare_parameter("min_walk_distance_m", 1.2).value
        )
        self.max_walk_distance_m = float(
            self.declare_parameter("max_walk_distance_m", 3.5).value
        )
        self.clear_threshold_m = float(
            self.declare_parameter("clear_threshold_m", 1.75).value
        )
        self.front_half_angle_deg = float(
            self.declare_parameter("front_half_angle_deg", 18.0).value
        )
        self.side_sector_min_deg = float(
            self.declare_parameter("side_sector_min_deg", 25.0).value
        )
        self.side_sector_max_deg = float(
            self.declare_parameter("side_sector_max_deg", 80.0).value
        )
        self.stop_duration_s = float(
            self.declare_parameter("stop_duration_s", 0.4).value
        )
        self.trick_duration = float(
            self.declare_parameter("trick_duration_s", 2).value
        )
        self.scan_timeout_s = float(
            self.declare_parameter("scan_timeout_s", 1.5).value
        )
        self.command_rate_hz = float(
            self.declare_parameter("command_rate_hz", 10.0).value
        )
        self.log_rate_hz = float(self.declare_parameter("log_rate_hz", 1.0).value)
        self.startup_command = self.declare_parameter(
            "startup_command", "balance_stand"
        ).value
        self.latest_scan = None
        self.latest_scan_time = 0.0
        self.phase = "startup"
        self.phase_end_time = time.monotonic() + 1.0
        self.turn_direction = 1.0
        self._last_command = None
        self._last_log_time = 0.0

        scan_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.cmd_pub = self.create_publisher(String, self.trigger_topic, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            scan_qos,
        )

        # walk, turn, sit, stretch, bark

        # TODO take bark out of this?

        # Markov table
        self.transitions = {
            'sit': [('rise_sit', 0.75), ('sit', 0.25)],
            'rise_sit': [('walk', 0.25), ('turn', 0.25), ('stretch', 0.25), ('bark', 0.25)],
            'walk': [('walk', 0.2), ('turn', 0.2), ('stretch', 0.2), ('bark', 0.2), ('sit', 0.2)],
            'turn': [('walk', 0.2), ('turn', 0.2), ('stretch', 0.2), ('bark', 0.2), ('sit', 0.2)],
            'stretch': [('walk', 0.2), ('turn', 0.2), ('stretch', 0.2), ('bark', 0.2), ('sit', 0.2)],
            'bark': [('walk', 0.2), ('turn', 0.2), ('stretch', 0.2), ('bark', 0.2), ('sit', 0.2)],
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

    def scan_callback(self, scan: LaserScan):
        self.latest_scan = scan
        self.latest_scan_time = time.monotonic()

    def tick(self):
        now = time.monotonic()
        if now >= self.phase_end_time:
            self.advance_phase()

        if self.phase == "walk":
            self.publish_move(self.forward_speed_mps, 0.0)
        elif self.phase in ("turn"):
            self.publish_move(0.0, self.turn_direction * self.current_turn_speed())
        else:
            self.publish_move(0.0, 0.0)

    def advance_phase(self):
        if self.phase == "startup":
            self.start_random_turn()

        # weighted random choice from transitions[current]
        options = self.transitions.get(current, [(current, 1.0)])
        states, weights = zip(*options)
        total = sum(weights)

        if total <= 0:
            return states[0]

        # normalize and choose
        probs = [w / total for w in weights]
        state = random.choices(states, probs, k=1)[0]

        if self.phase == "turn":
            self.start_random_walk()
        elif self.phase == "walk":
            self.start_random_turn()
        elif self.phase == "sit":
            self.start_sit()
        elif self.phase == "stretch":
            self.start_stretch()
        elif self.phase == "rise_sit":
            self.start_rise_sit()
        elif self.phase == "bark":
            self.start_bark()
        else:
            self.start_random_turn()

    def start_sit():
        # TODO
        duration = angle_rad / max(abs(self.trick_duration), 0.01)
        self.set_phase("sit", duration)
        return

    def start_rise_sit():
        # TODO
        duration = angle_rad / max(abs(self.trick_duration), 0.01)
        self.set_phase("rise_sit", duration)
        return

    def start_bark():
        # TODO
        duration = angle_rad / max(abs(self.trick_duration), 0.01)
        self.set_phase("bark", duration)
        return

    def start_random_turn(self):
        angle_rad = math.radians(random.uniform(self.min_turn_deg, self.max_turn_deg))
        self.turn_direction = random.choice([-1.0, 1.0])
        duration = angle_rad / max(abs(self.turn_speed_radps), 0.01)
        self.set_phase("turn", duration)

    def start_random_walk(self):
        distance = random.uniform(self.min_walk_distance_m, self.max_walk_distance_m)
        duration = distance / max(abs(self.forward_speed_mps), 0.01)
        self.set_phase("walk", duration)

    def set_phase(self, phase: str, duration_s: float):
        self.phase = phase
        self.phase_end_time = time.monotonic() + max(duration_s, 0.0)

    def current_turn_speed(self) -> float:
        return self.turn_speed_radps

    def clearer_turn_direction(self) -> float:
        left = self.sector_min(self.side_sector_min_deg, self.side_sector_max_deg)
        right = self.sector_min(-self.side_sector_max_deg, -self.side_sector_min_deg)

        if math.isfinite(left) and math.isfinite(right):
            return 1.0 if left >= right else -1.0
        if math.isfinite(left):
            return 1.0
        if math.isfinite(right):
            return -1.0
        return random.choice([-1.0, 1.0])

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
