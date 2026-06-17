#!/usr/bin/env python3

import math
import random
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from go2_interfaces.msg import Go2Command
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy

class WanderNode(Node):
    def __init__(self):
        super().__init__("wander_node")

        # All parameters
        self.trigger_topic = self.declare_parameter(
            "trigger_topic", "/wander_cmd"
        ).value

        self.latest_scan = None
        self.latest_scan_time = 0.0
        scan_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.side_sector_min_deg = float(
            self.declare_parameter("side_sector_min_deg", 25.0).value
        )
        self.side_sector_max_deg = float(
            self.declare_parameter("side_sector_max_deg", 90.0).value
        )
        self.scan_topic = self.declare_parameter("scan_topic", "/front_scan").value
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            scan_qos,
        )
        
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

        # Stretch is 5 seconds
        # Stand is 5 seconds
        self.trick_duration = float(
            self.declare_parameter("trick_duration_s", 5.0).value
        )
        self.bark_duration = float(
            self.declare_parameter("bark_duration_s", 1).value
        )

        self.command_rate_hz = float(
            self.declare_parameter("command_rate_hz", 1).value
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
        self.cmd_pub = self.create_publisher(Go2Command, self.trigger_topic, 10)
        self.bark_pub = self.create_publisher(String, self.bark_topic, 10)

        # Actions: walk, turn, sit, stretch, bark TODO

        # Markov table
        self.transitions = {
            'sit': [('walk', 0.75), ('sit', 0.25)],
            'walk': [('turn', 0.6), ('stretch', 0.2), ('sit', 0.2)],
            'turn': [('walk', 0.3), ('turn', 0.2), ('stretch', 0.1), ('bark', 0.1), ('sit', 0.2)],
            'stretch': [('walk', 0.6), ('turn', 0.4)],
            'bark': [('walk', 0.3), ('turn', 0.2), ('stretch', 0.1), ('bark', 0.2), ('sit', 0.2)],
        }

        # Tick timer
        timer_period = 1.0 / max(self.command_rate_hz, 1.0)
        self.timer = self.create_timer(timer_period, self.tick)

        if self.startup_command:
            self.publish_command(self.startup_command, force=True)

        self.get_logger().info(
            "WanderNode publishing Unified Go2Commands to %s" % (self.trigger_topic)
        )

    def scan_callback(self, scan: LaserScan):
        self.latest_scan = scan
        self.latest_scan_time = time.monotonic()

    def tick(self):
        now = time.monotonic()
        if now >= self.phase_end_time:
            self.publish_move(0.0, 0.0)
            self.advance_phase()

        if self.phase == "walk":
            self.publish_move(self.forward_speed_mps, 0.0)
        elif self.phase == "turn":
            self.publish_move(0.0, self.turn_direction * self.current_turn_speed())

    def advance_phase(self):
        if self.phase == "startup":
            self.start_turn()
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

        if state == "turn":
            self.start_turn()
        elif state == "walk":
            self.start_walk()
        elif state == "sit":
            self.start_sit()
        elif state == "stretch":
            self.start_stretch()
        elif state == "rise_sit":
            self.start_rise_sit()
        elif state == "bark":
            self.start_bark()
        else:
            self.start_turn()

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
        self.publish_command('bark', force=True)
        
        self.get_logger().info('PUBLISHED BARK')
        duration = max(abs(self.bark_duration), 0.01)
        self.set_phase("bark", duration)
        return

    def start_turn(self):
        # Determine angle to turn
        if self.latest_scan is None:
            self.turn_direction = random.choice([-1.0, 1.0])
        else:
            self.turn_direction = avoid_obvious_obstacles()

        # Determine speed and therefore duration
        angle_rad = math.radians(random.uniform(self.min_turn_deg, self.max_turn_deg))
        duration = angle_rad / max(abs(self.turn_speed_radps), 0.01)

        # Set turn phase on
        self.set_phase("turn", duration)

    def avoid_obvious_obstacles(self):
        scan = self.latest_scan
        if scan is not None and scan.ranges:
            left = self.sector_min(self.side_sector_min_deg, self.side_sector_max_deg)
            right = self.sector_min(-self.side_sector_max_deg, -self.side_sector_min_deg)
            if math.isfinite(left) and left < 1.5 and math.isfinite(right) and right < 1.5:
                self.get_logger().info('AVOIDING FRONT')
                return 1.0 if left >= right else -1.0
            if math.isfinite(left) and left < 1.5:
                self.get_logger().info('AVOIDING LEFT')
                return 1.0
            if math.isfinite(right) and right < 1.5:
                self.get_logger().info('AVOIDING RIGHT')
                return -1.0
        return random.choice([-1.0, 1.0])

    # TODO refactor dupication of this
    def angle_to_index(self, scan: LaserScan, angle_rad: float) -> int:
        # Prevent division by zero
        if scan.angle_increment == 0.0:
            return 0
            
        # Calculate where the angle falls in the array
        index = round((angle_rad - scan.angle_min) / scan.angle_increment)
        
        # Clamp the index to ensure it stays within the array bounds
        return max(0, min(len(scan.ranges) - 1, index))

    def sector_min(self, deg_min: float, deg_max: float) -> float:
        scan = self.latest_scan
        if scan is None or not scan.ranges:
            return float("inf")

        i0 = self.angle_to_index(scan, math.radians(deg_min))
        i1 = self.angle_to_index(scan, math.radians(deg_max))
        if i0 > i1:
            i0, i1 = i1, i0

        valid_ranges = []
        for value in scan.ranges[i0 : i1 + 1]:
            if not math.isfinite(value):
                continue
            value = float(value)
            if value <= 0.0 or value < scan.range_min or value > scan.range_max:
                continue
            valid_ranges.append(value)

        if not valid_ranges:
            return float("inf")
        return min(valid_ranges)

    def start_walk(self):
        distance = random.uniform(self.min_walk_distance_m, self.max_walk_distance_m)
        duration = distance / max(abs(self.forward_speed_mps), 0.01)
        self.get_logger().info(f'WALKING {distance}m AT SPEED {self.forward_speed_mps}')
        self.set_phase("walk", duration)

    def set_phase(self, phase: str, duration_s: float):
        self.phase = phase
        self.phase_end_time = time.monotonic() + max(duration_s, 0.0)

    def current_turn_speed(self) -> float:
        return self.turn_speed_radps

    # Publish a twist movement command using Go2Command
    def publish_move(self, vx: float, vyaw: float):
        msg = Go2Command()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.command_type = Go2Command.MOVE
        
        # Populate the nested Twist message
        msg.twist_command.linear.x = float(vx)
        msg.twist_command.linear.y = 0.0
        msg.twist_command.linear.z = 0.0
        
        msg.twist_command.angular.x = 0.0
        msg.twist_command.angular.y = 0.0
        msg.twist_command.angular.z = float(vyaw)
        
        self.cmd_pub.publish(msg)

    # Publish a non-move commands (startup, sit, bark, stretch) using Go2Command
    def publish_command(self, command: str, force: bool = False):
        if not force and command == self._last_command:
            return

        msg = Go2Command()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.command_type = Go2Command.TRICK
        msg.trick_name = command
        
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