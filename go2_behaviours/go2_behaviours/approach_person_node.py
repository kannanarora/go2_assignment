#!/usr/bin/env python3

import math
import time

import rclpy
from go2_interfaces.msg import Go2Command, PersonTrack
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan


class ApproachPersonNode(Node):
    def __init__(self):
        super().__init__("approach_person_node")

        self.person_track_topic = self.declare_parameter(
            "person_track_topic", "/person_track"
        ).value
        self.scan_topic = self.declare_parameter("scan_topic", "/front_scan").value
        self.command_topic = self.declare_parameter(
            "command_topic",
            "/approach_cmd",
        ).value

        self.stop_distance_m = float(
            self.declare_parameter("stop_distance_m", 1.10).value
        )
        self.distance_tolerance_m = float(
            self.declare_parameter("distance_tolerance_m", 0.10).value
        )
        self.forward_speed_mps = float(
            self.declare_parameter("forward_speed_mps", 0.42).value
        )
        self.min_forward_speed_mps = float(
            self.declare_parameter("min_forward_speed_mps", 0.26).value
        )
        self.approach_slowdown_distance_m = float(
            self.declare_parameter("approach_slowdown_distance_m", 0.30).value
        )
        self.min_bearing_speed_scale = float(
            self.declare_parameter("min_bearing_speed_scale", 0.50).value
        )
        self.search_yaw_speed_radps = float(
            self.declare_parameter("search_yaw_speed_radps", 0.55).value
        )
        self.max_yaw_speed_radps = float(
            self.declare_parameter("max_yaw_speed_radps", 0.55).value
        )
        self.max_walk_yaw_speed_radps = float(
            self.declare_parameter("max_walk_yaw_speed_radps", 0.18).value
        )
        self.yaw_kp = float(self.declare_parameter("yaw_kp", 0.90).value)
        self.yaw_sign = float(self.declare_parameter("yaw_sign", 1.0).value)
        self.centered_bearing_rad = float(
            self.declare_parameter("centered_bearing_rad", 0.18).value
        )
        self.walk_with_turn_bearing_rad = float(
            self.declare_parameter("walk_with_turn_bearing_rad", 0.35).value
        )
        self.turn_in_place_bearing_rad = float(
            self.declare_parameter("turn_in_place_bearing_rad", 0.60).value
        )
        self.min_person_confidence = float(
            self.declare_parameter("min_person_confidence", 0.40).value
        )
        self.confirm_person_s = float(
            self.declare_parameter("confirm_person_s", 0.35).value
        )
        self.arrival_confirm_s = float(
            self.declare_parameter("arrival_confirm_s", 0.45).value
        )
        self.arrival_centered_bearing_rad = float(
            self.declare_parameter("arrival_centered_bearing_rad", 0.25).value
        )

        self.person_obstacle_bearing_gate_rad = float(
            self.declare_parameter("person_obstacle_bearing_gate_rad", 0.35).value
        )
        self.person_obstacle_distance_tolerance_m = float(
            self.declare_parameter("person_obstacle_distance_tolerance_m", 0.35).value
        )
        self.centered_front_is_person_max_distance_m = float(
            self.declare_parameter(
                "centered_front_is_person_max_distance_m",
                2.5,
            ).value
        )
        self.obstacle_stop_distance_m = float(
            self.declare_parameter("obstacle_stop_distance_m", 0.65).value
        )
        self.obstacle_clear_distance_m = float(
            self.declare_parameter("obstacle_clear_distance_m", 0.90).value
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
            self.declare_parameter("avoid_yaw_speed_radps", 0.35).value
        )
        self.max_avoid_s = float(self.declare_parameter("max_avoid_s", 2.0).value)

        self.person_timeout_s = float(
            self.declare_parameter("person_timeout_s", 0.8).value
        )
        self.distance_memory_s = float(
            self.declare_parameter("distance_memory_s", 0.7).value
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
        self.person_seen_since = 0.0
        self.last_approach_distance = None
        self.last_approach_distance_time = 0.0
        self.arrival_candidate_since = 0.0
        self.latest_scan = None
        self.latest_scan_time = 0.0
        self.phase = "search"
        self.avoid_direction = 1.0
        self.avoid_end_time = 0.0
        self.finished = False
        self.arrival_repeat_count = 0
        self.next_arrival_command_time = 0.0
        self._last_vx = 0.0
        self._last_vyaw = 0.0
        self._last_command = None
        self._last_log_time = 0.0

        scan_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.command_pub = self.create_publisher(Go2Command, self.command_topic, 10)
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

        if float(msg.confidence) < self.min_person_confidence:
            return

        now = time.monotonic()
        if self.latest_person is None or now - self.latest_person_time > self.person_timeout_s:
            self.person_seen_since = now

        self.latest_person = msg
        self.latest_person_time = now

    def scan_callback(self, scan: LaserScan):
        self.latest_scan = scan
        self.latest_scan_time = time.monotonic()

    def tick(self):
        if self.finished:
            return

        now = time.monotonic()
        if self.phase == "arrived":
            self.publish_arrival_command_if_due(now)
            return

        scan_stale = self.scan_is_stale(now)
        person_visible = self.person_is_confirmed(now)
        front = self.sector_min(-self.front_half_angle_deg, self.front_half_angle_deg)
        front_is_person = (
            person_visible and self.front_return_matches_person(self.latest_person, front)
        )
        blocked_by_front = (
            not scan_stale
            and math.isfinite(front)
            and front < self.obstacle_stop_distance_m
        )
        blocked = blocked_by_front and not front_is_person
        clear = (
            scan_stale
            or front_is_person
            or (not math.isfinite(front))
            or front > self.obstacle_clear_distance_m
        )

        if scan_stale:
            self.phase = "waiting_for_scan"
            self.publish_move(0.0, 0.0)
            self.maybe_log("scan_stale", front)
            return

        if person_visible and self.has_arrived(self.latest_person, front):
            if self.arrival_candidate_since == 0.0:
                self.arrival_candidate_since = now
            if now - self.arrival_candidate_since >= self.arrival_confirm_s:
                self.arrive_and_sit()
                return
        else:
            self.arrival_candidate_since = 0.0

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
        yaw_error = self.centered_yaw_error(bearing)
        yaw = self.clamp(
            self.yaw_sign * self.yaw_kp * yaw_error,
            -self.max_yaw_speed_radps,
            self.max_yaw_speed_radps,
        )

        if abs(bearing) > max(self.walk_with_turn_bearing_rad, self.centered_bearing_rad):
            return 0.0, yaw

        yaw = self.clamp(
            yaw,
            -self.max_walk_yaw_speed_radps,
            self.max_walk_yaw_speed_radps,
        )

        approach_distance = self.estimate_person_distance(person, front)
        distance_scale = 1.0
        if approach_distance is not None:
            remaining = max(approach_distance - self.stop_distance_m, 0.0)
            if remaining <= 0.0:
                return 0.0, yaw
            distance_scale = self.clamp(
                remaining / max(self.approach_slowdown_distance_m, 0.05),
                0.0,
                1.0,
            )

        bearing_scale = self.clamp(
            1.0 - abs(bearing) / max(self.turn_in_place_bearing_rad, 0.01),
            self.min_bearing_speed_scale,
            1.0,
        )
        speed = self.forward_speed_mps * distance_scale * bearing_scale

        if distance_scale > 0.0:
            speed = max(speed, self.min_forward_speed_mps)

        if math.isfinite(front) and not self.front_return_matches_person(person, front):
            obstacle_margin = front - self.obstacle_stop_distance_m
            if obstacle_margin <= 0.0:
                speed = 0.0
            else:
                speed *= self.clamp(obstacle_margin / 0.5, 0.0, 1.0)

        return speed, yaw

    def estimate_person_distance(self, person: PersonTrack, front: float):
        distance = None
        if person.distance_valid:
            distance = float(person.distance_m)
        elif self.front_return_matches_person(person, front):
            distance = float(front)

        now = time.monotonic()
        if distance is not None:
            self.last_approach_distance = distance
            self.last_approach_distance_time = now
            return distance

        if (
            self.last_approach_distance is not None
            and now - self.last_approach_distance_time <= self.distance_memory_s
        ):
            return self.last_approach_distance

        return None

    def centered_yaw_error(self, bearing: float) -> float:
        deadband = max(self.centered_bearing_rad, 0.0)
        abs_bearing = abs(bearing)
        if abs_bearing <= deadband:
            return 0.0
        return math.copysign(abs_bearing - deadband, bearing)

    def front_return_matches_person(self, person: PersonTrack, front: float) -> bool:
        if person is None or not math.isfinite(front):
            return False

        if abs(float(person.bearing_rad)) > self.person_obstacle_bearing_gate_rad:
            return False

        if front < self.obstacle_stop_distance_m:
            return False

        if not person.distance_valid:
            return front <= self.centered_front_is_person_max_distance_m

        return (
            abs(float(person.distance_m) - float(front))
            <= self.person_obstacle_distance_tolerance_m
        )

    def has_arrived(self, person: PersonTrack, front: float):
        if abs(float(person.bearing_rad)) > self.arrival_centered_bearing_rad:
            return False

        approach_distance = self.estimate_person_distance(person, front)
        if approach_distance is None:
            return False

        return approach_distance <= self.stop_distance_m + self.distance_tolerance_m

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

    def person_is_confirmed(self, now: float) -> bool:
        if not self.person_is_fresh(now):
            return False
        return now - self.person_seen_since >= self.confirm_person_s

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
        msg.twist_command.angular.z = float(vyaw)
        self.command_pub.publish(msg)
        self._last_vx = float(vx)
        self._last_vyaw = float(vyaw)

    def publish_command(self, command: str, force: bool = False):
        if not force and command == self._last_command:
            return

        msg = Go2Command()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.command_type = Go2Command.TRICK
        msg.trick_name = command
        self.command_pub.publish(msg)
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
                "phase=%s status=%s front=%s bearing=%.2f distance=%s "
                "cmd=(%.2f, %.2f)"
                % (
                    self.phase,
                    status,
                    front_text,
                    person.bearing_rad,
                    distance_text,
                    self._last_vx,
                    self._last_vyaw,
                )
            )
        else:
            self.get_logger().info(
                "phase=%s status=%s front=%s no_person cmd=(%.2f, %.2f)"
                % (self.phase, status, front_text, self._last_vx, self._last_vyaw)
            )

    def clamp(self, value: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, value))

    def destroy_node(self):
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
