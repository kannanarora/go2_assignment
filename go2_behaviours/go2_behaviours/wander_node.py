#!/usr/bin/env python3

import math
import random
import time

import rclpy
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

        self.scan_topic = self.declare_parameter("scan_topic", "/front_scan").value
        self.trigger_topic = self.declare_parameter(
            "trigger_topic", "/trigger_behaviour"
        ).value

        self.forward_speed_mps = float(
            self.declare_parameter("forward_speed_mps", 0.22).value
        )
        self.turn_speed_radps = float(
            self.declare_parameter("turn_speed_radps", 0.42).value
        )
        self.avoid_turn_speed_radps = float(
            self.declare_parameter("avoid_turn_speed_radps", 0.45).value
        )

        self.min_turn_deg = float(self.declare_parameter("min_turn_deg", 35.0).value)
        self.max_turn_deg = float(self.declare_parameter("max_turn_deg", 160.0).value)
        self.min_walk_distance_m = float(
            self.declare_parameter("min_walk_distance_m", 1.2).value
        )
        self.max_walk_distance_m = float(
            self.declare_parameter("max_walk_distance_m", 3.5).value
        )

        self.avoid_threshold_m = float(
            self.declare_parameter("avoid_threshold_m", 1.45).value
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
        self.avoid_extra_turn_deg = float(
            self.declare_parameter("avoid_extra_turn_deg", 18.0).value
        )
        self.max_avoid_turn_s = float(
            self.declare_parameter("max_avoid_turn_s", 8.0).value
        )

        self.stop_duration_s = float(
            self.declare_parameter("stop_duration_s", 0.4).value
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

        if self.clear_threshold_m <= self.avoid_threshold_m:
            self.clear_threshold_m = self.avoid_threshold_m + 0.2
            self.get_logger().warn(
                "clear_threshold_m must be > avoid_threshold_m; adjusted to %.2f"
                % self.clear_threshold_m
            )

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
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            scan_qos,
        )

        timer_period = 1.0 / max(self.command_rate_hz, 1.0)
        self.timer = self.create_timer(timer_period, self.tick)

        if self.startup_command:
            self.publish_command(self.startup_command, force=True)

        self.get_logger().info(
            "WanderNode publishing to %s and avoiding obstacles from %s "
            "(avoid<%.2fm, clear>%.2fm)"
            % (
                self.trigger_topic,
                self.scan_topic,
                self.avoid_threshold_m,
                self.clear_threshold_m,
            )
        )

    def scan_callback(self, scan: LaserScan):
        self.latest_scan = scan
        self.latest_scan_time = time.monotonic()

    def tick(self):
        now = time.monotonic()
        front = self.sector_min(-self.front_half_angle_deg, self.front_half_angle_deg)
        blocked = math.isfinite(front) and front < self.avoid_threshold_m
        clear = (not math.isfinite(front)) or front > self.clear_threshold_m
        scan_stale = self.scan_is_stale(now)

        if scan_stale:
            self.set_phase("waiting_for_scan", 0.5)
            self.publish_move(0.0, 0.0)
            self.maybe_log(front, "scan_stale")
            return

        if blocked and self.phase not in ("avoid_stop", "avoid_turn"):
            self.start_avoidance()

        if self.phase == "avoid_turn" and clear:
            self.start_avoid_extra_turn()

        if now >= self.phase_end_time:
            self.advance_phase()

        if self.phase == "walk":
            if blocked:
                self.start_avoidance()
                self.publish_move(0.0, 0.0)
            else:
                self.publish_move(self.forward_speed_mps, 0.0)
        elif self.phase in ("turn", "avoid_turn", "avoid_extra_turn"):
            self.publish_move(0.0, self.turn_direction * self.current_turn_speed())
        else:
            self.publish_move(0.0, 0.0)

        self.maybe_log(front, "blocked" if blocked else self.phase)

    def advance_phase(self):
        if self.phase == "startup":
            self.start_random_turn()
        elif self.phase == "turn":
            self.start_random_walk()
        elif self.phase == "walk":
            self.start_random_turn()
        elif self.phase == "avoid_stop":
            self.start_avoid_turn()
        elif self.phase == "avoid_turn":
            self.start_avoid_turn()
        elif self.phase == "avoid_extra_turn":
            self.start_random_walk()
        elif self.phase == "waiting_for_scan":
            self.start_random_turn()
        else:
            self.start_random_turn()

    def start_random_turn(self):
        angle_rad = math.radians(random.uniform(self.min_turn_deg, self.max_turn_deg))
        self.turn_direction = random.choice([-1.0, 1.0])
        duration = angle_rad / max(abs(self.turn_speed_radps), 0.01)
        self.set_phase("turn", duration)

    def start_random_walk(self):
        distance = random.uniform(self.min_walk_distance_m, self.max_walk_distance_m)
        duration = distance / max(abs(self.forward_speed_mps), 0.01)
        self.set_phase("walk", duration)

    def start_avoidance(self):
        self.turn_direction = self.clearer_turn_direction()
        self.set_phase("avoid_stop", self.stop_duration_s)

    def start_avoid_turn(self):
        self.set_phase("avoid_turn", self.max_avoid_turn_s)

    def start_avoid_extra_turn(self):
        duration = math.radians(self.avoid_extra_turn_deg) / max(
            abs(self.avoid_turn_speed_radps), 0.01
        )
        self.set_phase("avoid_extra_turn", duration)

    def set_phase(self, phase: str, duration_s: float):
        self.phase = phase
        self.phase_end_time = time.monotonic() + max(duration_s, 0.0)

    def current_turn_speed(self) -> float:
        if self.phase == "avoid_turn":
            return self.avoid_turn_speed_radps
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

    def scan_is_stale(self, now: float) -> bool:
        if self.latest_scan is None:
            return True
        return now - self.latest_scan_time > self.scan_timeout_s

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

    def angle_to_index(self, scan: LaserScan, angle_rad: float) -> int:
        if scan.angle_increment == 0.0 or len(scan.ranges) == 0:
            return 0
        idx = int(round((angle_rad - scan.angle_min) / scan.angle_increment))
        return max(0, min(idx, len(scan.ranges) - 1))

    def publish_move(self, vx: float, vyaw: float):
        self.publish_command("move %.3f 0.000 %.3f" % (vx, vyaw), force=True)

    def publish_command(self, command: str, force: bool = False):
        if not force and command == self._last_command:
            return

        msg = String()
        msg.data = command
        self.cmd_pub.publish(msg)
        self._last_command = command

    def maybe_log(self, front: float, status: str):
        if self.log_rate_hz <= 0.0:
            return

        now = time.monotonic()
        if now - self._last_log_time < 1.0 / self.log_rate_hz:
            return

        self._last_log_time = now
        front_text = "%.2fm" % front if math.isfinite(front) else "inf"
        self.get_logger().info(
            "phase=%s status=%s front=%s turn_dir=%+.0f"
            % (self.phase, status, front_text, self.turn_direction)
        )

    def destroy_node(self):
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
