#!/usr/bin/env python3

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from go2_interfaces.msg import PersonTrack
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class ApproachPersonNode(Node):
    def __init__(self):
        super().__init__("approach_person_node")

        self.person_track_topic = self.declare_parameter(
            "person_track_topic", "/person_track"
        ).value
        self.scan_topic = self.declare_parameter("scan_topic", "/front_scan").value
        self.cmd_vel_topic = self.declare_parameter("cmd_vel_topic", "/cmd_vel").value
        self.trigger_topic = self.declare_parameter(
            "trigger_topic", "/trigger_behaviour"
        ).value

        self.stop_distance_m = float(
            self.declare_parameter("stop_distance_m", 1.0).value
        )
        self.distance_tolerance_m = float(
            self.declare_parameter("distance_tolerance_m", 0.08).value
        )
        self.forward_speed_mps = float(
            self.declare_parameter("forward_speed_mps", 0.25).value
        )
        self.min_forward_speed_mps = float(
            self.declare_parameter("min_forward_speed_mps", 0.08).value
        )
        self.search_yaw_speed_radps = float(
            self.declare_parameter("search_yaw_speed_radps", 0.45).value
        )
        self.max_yaw_speed_radps = float(
            self.declare_parameter("max_yaw_speed_radps", 0.85).value
        )
        self.yaw_kp = float(self.declare_parameter("yaw_kp", 1.6).value)
        self.yaw_sign = float(self.declare_parameter("yaw_sign", -1.0).value)
        self.centered_bearing_rad = float(
            self.declare_parameter("centered_bearing_rad", 0.08).value
        )
        self.turn_in_place_bearing_rad = float(
            self.declare_parameter("turn_in_place_bearing_rad", 0.40).value
        )

        self.obstacle_stop_distance_m = float(
            self.declare_parameter("obstacle_stop_distance_m", 0.75).value
        )
        self.obstacle_clear_distance_m = float(
            self.declare_parameter("obstacle_clear_distance_m", 1.05).value
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
        self.avoid_yaw_speed_radps = float(
            self.declare_parameter("avoid_yaw_speed_radps", 0.65).value
        )
        self.max_avoid_s = float(self.declare_parameter("max_avoid_s", 5.0).value)

        self.person_timeout_s = float(
            self.declare_parameter("person_timeout_s", 0.8).value
        )
        self.scan_timeout_s = float(self.declare_parameter("scan_timeout_s", 1.5).value)
        self.command_rate_hz = float(
            self.declare_parameter("command_rate_hz", 10.0).value
        )
        self.log_rate_hz = float(self.declare_parameter("log_rate_hz", 1.0).value)
        self.startup_command = self.declare_parameter(
            "startup_command", "balance_stand"
        ).value
        self.arrival_command = self.declare_parameter("arrival_command", "sit").value
        self.arrival_command_repeats = int(
            self.declare_parameter("arrival_command_repeats", 5).value
        )
        self.arrival_command_period_s = float(
            self.declare_parameter("arrival_command_period_s", 0.25).value
        )

        if self.obstacle_clear_distance_m <= self.obstacle_stop_distance_m:
            self.obstacle_clear_distance_m = self.obstacle_stop_distance_m + 0.2

        self.latest_person = None
        self.latest_person_time = 0.0
        self.latest_scan = None
        self.latest_scan_time = 0.0
        self.phase = "search"
        self.avoid_direction = 1.0
        self.avoid_end_time = 0.0
        self.finished = False
        self.arrival_repeat_count = 0
        self.next_arrival_command_time = 0.0
        self._last_command = None
        self._last_log_time = 0.0

        scan_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.trigger_pub = self.create_publisher(String, self.trigger_topic, 10)
        self.person_sub = self.create_subscription(
            PersonTrack,
            self.person_track_topic,
            self.person_callback,
            10,
        )
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
            "ApproachPersonNode tracking %s, stopping at %.2fm, avoiding front<%.2fm"
            % (
                self.person_track_topic,
                self.stop_distance_m,
                self.obstacle_stop_distance_m,
            )
        )

    def person_callback(self, msg: PersonTrack):
        if not msg.visible:
            return

        self.latest_person = msg
        self.latest_person_time = time.monotonic()

    def scan_callback(self, scan: LaserScan):
        self.latest_scan = scan
        self.latest_scan_time = time.monotonic()

    def tick(self):
        if self.finished:
            self.publish_move(0.0, 0.0)
            return

        now = time.monotonic()
        if self.phase == "arrived":
            self.publish_move(0.0, 0.0)
            self.publish_arrival_command_if_due(now)
            return

        scan_stale = self.scan_is_stale(now)
        person_visible = self.person_is_fresh(now)
        front = self.sector_min(-self.front_half_angle_deg, self.front_half_angle_deg)
        blocked = (
            not scan_stale
            and math.isfinite(front)
            and front < self.obstacle_stop_distance_m
        )
        clear = scan_stale or (not math.isfinite(front)) or front > self.obstacle_clear_distance_m

        if scan_stale:
            self.phase = "waiting_for_scan"
            self.publish_move(0.0, 0.0)
            self.maybe_log("scan_stale", front)
            return

        if person_visible and self.has_arrived(self.latest_person):
            self.arrive_and_sit()
            return

        if blocked and self.phase != "avoid":
            self.start_avoid(now)

        if self.phase == "avoid":
            if clear or now >= self.avoid_end_time:
                self.phase = "search" if not person_visible else "approach"
            else:
                self.publish_move(0.0, self.avoid_direction * self.avoid_yaw_speed_radps)
                self.maybe_log("avoiding", front)
                return

        if not person_visible:
            self.phase = "search"
            self.publish_move(0.0, self.search_yaw_speed_radps)
            self.maybe_log("searching", front)
            return

        self.phase = "approach"
        vx, vyaw = self.approach_command(self.latest_person, front)
        self.publish_move(vx, vyaw)
        self.maybe_log("approach", front)

    def approach_command(self, person: PersonTrack, front: float):
        bearing = float(person.bearing_rad)
        yaw = self.clamp(
            self.yaw_sign * self.yaw_kp * bearing,
            -self.max_yaw_speed_radps,
            self.max_yaw_speed_radps,
        )

        if abs(bearing) > self.turn_in_place_bearing_rad:
            return 0.0, yaw

        distance_scale = 1.0
        if person.distance_valid:
            remaining = max(float(person.distance_m) - self.stop_distance_m, 0.0)
            distance_scale = self.clamp(remaining / 1.0, 0.0, 1.0)

        bearing_scale = self.clamp(
            1.0 - abs(bearing) / max(self.turn_in_place_bearing_rad, 0.01),
            0.0,
            1.0,
        )
        speed = self.forward_speed_mps * max(distance_scale, 0.2) * bearing_scale

        if abs(bearing) <= self.centered_bearing_rad:
            speed = max(speed, self.min_forward_speed_mps)

        if math.isfinite(front):
            obstacle_margin = front - self.obstacle_stop_distance_m
            if obstacle_margin <= 0.0:
                speed = 0.0
            else:
                speed *= self.clamp(obstacle_margin / 0.5, 0.0, 1.0)

        return speed, yaw

    def has_arrived(self, person: PersonTrack):
        if not person.distance_valid:
            return False
        return float(person.distance_m) <= self.stop_distance_m + self.distance_tolerance_m

    def arrive_and_sit(self):
        self.phase = "arrived"
        self.publish_move(0.0, 0.0)
        self.arrival_repeat_count = 0
        self.next_arrival_command_time = 0.0
        self.get_logger().info("Arrived at person; stopping and sending %s" % self.arrival_command)

    def publish_arrival_command_if_due(self, now: float):
        if self.arrival_repeat_count >= self.arrival_command_repeats:
            self.finished = True
            self.get_logger().info("Approach person behaviour complete")
            return

        if now < self.next_arrival_command_time:
            return

        self.publish_command(self.arrival_command, force=True)
        self.arrival_repeat_count += 1
        self.next_arrival_command_time = now + self.arrival_command_period_s

    def start_avoid(self, now: float):
        self.phase = "avoid"
        self.avoid_direction = self.clearer_turn_direction()
        self.avoid_end_time = now + self.max_avoid_s
        self.publish_move(0.0, 0.0)

    def clearer_turn_direction(self) -> float:
        left = self.sector_min(self.side_sector_min_deg, self.side_sector_max_deg)
        right = self.sector_min(-self.side_sector_max_deg, -self.side_sector_min_deg)

        if math.isfinite(left) and math.isfinite(right):
            return 1.0 if left >= right else -1.0
        if math.isfinite(left):
            return 1.0
        if math.isfinite(right):
            return -1.0
        return 1.0

    def person_is_fresh(self, now: float) -> bool:
        if self.latest_person is None:
            return False
        return now - self.latest_person_time <= self.person_timeout_s

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
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(vyaw)
        self.cmd_vel_pub.publish(msg)

    def publish_command(self, command: str, force: bool = False):
        if not force and command == self._last_command:
            return

        msg = String()
        msg.data = command
        self.trigger_pub.publish(msg)
        self._last_command = command

    def maybe_log(self, status: str, front: float):
        if self.log_rate_hz <= 0.0:
            return

        now = time.monotonic()
        if now - self._last_log_time < 1.0 / self.log_rate_hz:
            return

        self._last_log_time = now
        person = self.latest_person
        front_text = "%.2fm" % front if math.isfinite(front) else "inf"
        if person is not None and self.person_is_fresh(now):
            distance_text = (
                "%.2fm" % person.distance_m if person.distance_valid else "invalid"
            )
            self.get_logger().info(
                "phase=%s status=%s front=%s bearing=%.2f distance=%s"
                % (self.phase, status, front_text, person.bearing_rad, distance_text)
            )
        else:
            self.get_logger().info(
                "phase=%s status=%s front=%s no_person"
                % (self.phase, status, front_text)
            )

    def clamp(self, value: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, value))

    def destroy_node(self):
        self.publish_move(0.0, 0.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ApproachPersonNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
