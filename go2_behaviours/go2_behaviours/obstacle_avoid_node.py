#!/usr/bin/env python3

"""
Reactive obstacle avoidance - Subscribes to /front_scan and publishes Go2Command (MOVE) on /avoidance_cmd for mux_node. Stays silent when idle so wander can pass through.
"""

import math
import random
import time

import rclpy
from go2_interfaces.msg import Go2Command
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan


class ObstacleAvoidNode(Node):
    def __init__(self):
        super().__init__("obstacle_avoid_node")

        self.scan_topic = self.declare_parameter("scan_topic", "/front_scan").value
        self.command_topic = self.declare_parameter(
            "command_topic", "/avoidance_cmd"
        ).value

        self.turn_speed_radps = float(
            self.declare_parameter("turn_speed_radps", 1.26).value
        )
        self.avoid_turn_speed_radps = float(
            self.declare_parameter("avoid_turn_speed_radps", 1.35).value
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
        self.turn_direction_lock_s = float(
            self.declare_parameter("turn_direction_lock_s", 8.0).value
        )
        self.switch_direction_margin_m = float(
            self.declare_parameter("switch_direction_margin_m", 0.25).value
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

        if self.clear_threshold_m <= self.avoid_threshold_m:
            self.clear_threshold_m = self.avoid_threshold_m + 0.2
            self.get_logger().warn(
                "clear_threshold_m must be > avoid_threshold_m; adjusted to %.2f"
                % self.clear_threshold_m
            )

        self.latest_scan = None
        self.latest_scan_time = 0.0
        self.phase = "idle"
        self.phase_end_time = 0.0
        self.turn_direction = 1.0
        self.turn_direction_lock_until = 0.0
        self._last_log_time = 0.0
        self._active = False
        
        # Keep track of the closest distance recorded during the node's lifespan
        self.closest_distance_recorded = float("inf")

        scan_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.command_pub = self.create_publisher(
            Go2Command, self.command_topic, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            scan_qos,
        )

        timer_period = 1.0 / max(self.command_rate_hz, 1.0)
        self.timer = self.create_timer(timer_period, self.tick)

        self.get_logger().info(
            "ObstacleAvoidNode publishing Go2Command to %s from %s "
            "(avoid<%.2fm, clear>%.2fm)"
            % (
                self.command_topic,
                self.scan_topic,
                self.avoid_threshold_m,
                self.clear_threshold_m,
            )
        )

    def scan_callback(self, scan: LaserScan):
        self.latest_scan = scan
        self.latest_scan_time = time.monotonic()
        
        # Calculate and update the closest distance recorded across the entire scan
        valid_ranges = [
            r for r in scan.ranges 
            if math.isfinite(r) and r > 0.0 and scan.range_min <= r <= scan.range_max
        ]
        
        if valid_ranges:
            current_min = min(valid_ranges)
            if current_min < self.closest_distance_recorded:
                self.closest_distance_recorded = current_min

    def tick(self):
        now = time.monotonic()
        front = self.sector_min(-self.front_half_angle_deg, self.front_half_angle_deg)
        blocked = math.isfinite(front) and front < self.avoid_threshold_m
        clear = (not math.isfinite(front)) or front > self.clear_threshold_m
        scan_stale = self.scan_is_stale(now)

        if scan_stale:
            self.set_phase("waiting_for_scan", 0.5)
            self._active = True
            self.publish_move(0.0, 0.0)
            self.maybe_log(front, "scan_stale")
            return

        if blocked and self.phase not in ("avoid_stop", "avoid_turn"):
            self.start_avoidance()

        if self.phase == "avoid_turn" and clear:
            self.start_avoid_extra_turn()

        if now >= self.phase_end_time and self.phase != "idle":
            self.advance_phase()

        if self.phase in ("avoid_stop", "waiting_for_scan"):
            self._active = True
            self.publish_move(0.0, 0.0)
        elif self.phase in ("avoid_turn", "avoid_extra_turn"):
            self._active = True
            self.maybe_switch_turn_direction()
            self.publish_move(0.0, self.turn_direction * self.current_turn_speed())
        else:
            self._active = False

        self.maybe_log(front, "blocked" if blocked else self.phase)

    def advance_phase(self):
        if self.phase == "avoid_stop":
            self.start_avoid_turn()
        elif self.phase == "avoid_turn":
            self.start_avoid_turn()
        elif self.phase == "avoid_extra_turn":
            self.set_phase("idle", 0.0)
        elif self.phase == "waiting_for_scan":
            self.set_phase("idle", 0.0)

    def start_avoidance(self):
        now = time.monotonic()
        if now >= self.turn_direction_lock_until:
            self.turn_direction = self.clearer_turn_direction()
        self.turn_direction_lock_until = now + max(self.turn_direction_lock_s, 0.0)
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

    def maybe_switch_turn_direction(self):
        current_side = self.side_clearance(self.turn_direction)
        opposite_side = self.side_clearance(-self.turn_direction)

        if not math.isfinite(current_side):
            return
        if current_side >= self.avoid_threshold_m:
            return
        if not math.isfinite(opposite_side):
            self.turn_direction *= -1.0
            self.turn_direction_lock_until = (
                time.monotonic() + max(self.turn_direction_lock_s, 0.0)
            )
            return
        if opposite_side - current_side >= self.switch_direction_margin_m:
            self.turn_direction *= -1.0
            self.turn_direction_lock_until = (
                time.monotonic() + max(self.turn_direction_lock_s, 0.0)
            )

    def side_clearance(self, direction: float) -> float:
        if direction >= 0.0:
            return self.sector_min(self.side_sector_min_deg, self.side_sector_max_deg)
        return self.sector_min(-self.side_sector_max_deg, -self.side_sector_min_deg)

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
        msg = Go2Command()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.command_type = Go2Command.MOVE
        msg.twist_command.linear.x = float(vx)
        msg.twist_command.linear.y = 0.0
        msg.twist_command.linear.z = 0.0
        msg.twist_command.angular.x = 0.0
        msg.twist_command.angular.y = 0.0
        msg.twist_command.angular.z = float(vyaw)
        self.command_pub.publish(msg)

    def maybe_log(self, front: float, status: str):
        if self.log_rate_hz <= 0.0:
            return

        now = time.monotonic()
        if now - self._last_log_time < 1.0 / self.log_rate_hz:
            return

        self._last_log_time = now
        front_text = "%.2fm" % front if math.isfinite(front) else "inf"
        self.get_logger().info(
            "phase=%s status=%s front=%s turn_dir=%+.0f active=%s"
            % (self.phase, status, front_text, self.turn_direction, self._active)
        )

    def destroy_node(self):
        self.publish_move(0.0, 0.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Fetch the tracked variable when the script is killed
        closest = node.closest_distance_recorded
        dist_str = "%.3fm" % closest if math.isfinite(closest) else "inf"
        
        # Log the result directly to the terminal
        node.get_logger().info(
            f"\n--- KeyboardInterrupt caught ---\n"
            f"Closest LIDAR distance recorded during run: {dist_str}"
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()