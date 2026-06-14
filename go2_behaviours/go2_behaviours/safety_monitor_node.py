"""
Tier 1 - Reactive Layer: Safety Monitor Node
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String


class SafetyMonitorNode(Node):

    def __init__(self):
        super().__init__('safety_monitor_node')

        self.scan_topic = self.declare_parameter('scan_topic', '/front_scan').value
        self.override_topic = self.declare_parameter(
            'safety_override_topic', '/safety_override').value
        self.trigger_topic = self.declare_parameter(
            'trigger_behaviour_topic', '/trigger_behaviour').value

        self.sit_threshold_m = float(
            self.declare_parameter('sit_threshold_m', 0.8).value)
        self.clear_threshold_m = float(
            self.declare_parameter('clear_threshold_m', 1.2).value)
        self.front_half_angle_deg = float(
            self.declare_parameter('front_half_angle_deg', 15.0).value)
        self.required_blocked_frames = int(
            self.declare_parameter('required_blocked_frames', 2).value)
        self.required_clear_frames = int(
            self.declare_parameter('required_clear_frames', 3).value)
        self.sit_command = self.declare_parameter('safety_command', 'sit').value
        self.stand_command = self.declare_parameter('clear_command', 'rise_sit').value
        self.enable_stand_command = bool(
            self.declare_parameter('enable_stand_command', True).value)
        self.self_ignore_distance_m = float(
            self.declare_parameter('self_ignore_distance_m', 0.68).value)
        self.min_sit_hold_s = float(
            self.declare_parameter('min_sit_hold_s', 2.5).value)
        self.recovery_cooldown_s = float(
            self.declare_parameter('recovery_cooldown_s', 1.5).value)
        self.log_rate_hz = float(self.declare_parameter('log_rate_hz', 2.0).value)

        if self.clear_threshold_m <= self.sit_threshold_m:
            self.clear_threshold_m = self.sit_threshold_m + 0.2

        self._safety_active = False
        self._blocked_count = 0
        self._clear_count = 0
        self._sit_since_time = 0.0
        self._rise_since_time = 0.0
        self._last_log_time = 0.0

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.override_pub = self.create_publisher(Bool, self.override_topic, 10)
        self.trigger_pub = self.create_publisher(String, self.trigger_topic, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self._on_scan, qos)

        self.get_logger().info(
            f'SafetyMonitorNode ready — sit<{self.sit_threshold_m}m, '
            f'clear>{self.clear_threshold_m}m, front=±{self.front_half_angle_deg}deg'
        )

    def _angle_to_index(self, scan: LaserScan, angle_rad: float) -> int:
        if scan.angle_increment == 0.0 or len(scan.ranges) == 0:
            return 0
        idx = int(round((angle_rad - scan.angle_min) / scan.angle_increment))
        return max(0, min(idx, len(scan.ranges) - 1))

    def _front_min_range(self, scan: LaserScan) -> float:
        if not scan.ranges:
            return float('inf')

        half_angle_rad = math.radians(self.front_half_angle_deg)
        i0 = self._angle_to_index(scan, -half_angle_rad)
        i1 = self._angle_to_index(scan, half_angle_rad)
        if i0 > i1:
            i0, i1 = i1, i0

        valid_ranges = []
        for value in scan.ranges[i0:i1 + 1]:
            if not math.isfinite(value):
                continue
            value = float(value)
            if value <= 0.0 or value < scan.range_min or value > scan.range_max:
                continue
            valid_ranges.append(value)

        return min(valid_ranges) if valid_ranges else float('inf')

    def _on_scan(self, scan: LaserScan):
        front_range = self._front_min_range(scan)
        blocked = math.isfinite(front_range) and front_range < self.sit_threshold_m

        if self._safety_active:
            # Ignore leg/body returns very close to the sensor while sitting.
            if blocked and front_range < self.self_ignore_distance_m:
                blocked = False
            # require a genuinely open path before standing
            clear = (
                not math.isfinite(front_range)
                or front_range > self.clear_threshold_m
            )
        else:
            clear = (
                not math.isfinite(front_range)
                or front_range > self.clear_threshold_m
            )

        if blocked:
            self._blocked_count += 1
            self._clear_count = 0
        elif clear:
            self._clear_count += 1
            self._blocked_count = 0
        elif not self._safety_active:
            self._blocked_count = 0
            self._clear_count = 0
        # While safety is active, hold clear_count in the dead zone (0.8-1.2 m).

        now = time.monotonic()

        if not self._safety_active:
            cooled_down = now - self._rise_since_time >= self.recovery_cooldown_s
            if cooled_down and self._blocked_count >= self.required_blocked_frames:
                self._set_safety_active(True)
                self._send_command(self.sit_command)
                self._blocked_count = 0
                self._clear_count = 0
        elif (
            self.enable_stand_command
            and now - self._sit_since_time >= self.min_sit_hold_s
            and self._clear_count >= self.required_clear_frames
        ):
            self._set_safety_active(False)
            self._send_command(self.stand_command)
            self._blocked_count = 0
            self._clear_count = 0

        self._maybe_log(front_range, blocked, clear)

    def _set_safety_active(self, active: bool):
        now = time.monotonic()
        if active and not self._safety_active:
            self._sit_since_time = now
        elif not active and self._safety_active:
            self._rise_since_time = now
        self._safety_active = active
        msg = Bool()
        msg.data = active
        self.override_pub.publish(msg)
        self.get_logger().info(f'Safety override {"ACTIVE" if active else "cleared"}')

    def _send_command(self, command: str):
        msg = String()
        msg.data = command
        self.trigger_pub.publish(msg)
        self.get_logger().info(f'Sent safety command: {command}')

    def _maybe_log(self, front_range: float, blocked: bool, clear: bool):
        if self.log_rate_hz <= 0.0:
            return
        now = time.monotonic()
        if now - self._last_log_time < 1.0 / self.log_rate_hz:
            return
        self._last_log_time = now
        front_text = f'{front_range:.2f}m' if math.isfinite(front_range) else 'inf'
        self.get_logger().info(
            f'front={front_text} blocked={blocked} clear={clear} '
            f'safety_active={self._safety_active} clear_count={self._clear_count}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
