"""
Converts a PointCloud2 topic into a LaserScan in a forward-facing sector.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2


class PointCloudToLaserScanNode(Node):
    def __init__(self):
        super().__init__('pointcloud_to_laserscan_node')

        self.cloud_topic = self.declare_parameter(
            'cloud_topic', '/utlidar/cloud_base'
        ).value
        self.angle_min_deg = float(self.declare_parameter('angle_min_deg', -105.0).value)
        self.angle_max_deg = float(self.declare_parameter('angle_max_deg', 105.0).value)
        self.angle_increment_deg = float(
            self.declare_parameter('angle_increment_deg', 1.0).value
        )
        self.range_min = float(self.declare_parameter('range_min', 0.05).value)
        self.range_max = float(self.declare_parameter('range_max', 30.0).value)
        self.use_z_filter = bool(self.declare_parameter('use_z_filter', False).value)
        self.z_min = float(self.declare_parameter('z_min', 0.1).value)
        self.z_max = float(self.declare_parameter('z_max', 1.0).value)
        self.output_frame = self.declare_parameter('output_frame', '').value
        self.expected_frame = self.declare_parameter('expected_frame', 'base_link').value
        self.forward_axis = self.declare_parameter('forward_axis', 'x').value
        self.lateral_axis = self.declare_parameter('lateral_axis', 'y').value
        self.require_positive_forward = bool(
            self.declare_parameter('require_positive_forward', True).value
        )
        self.log_every_n = int(self.declare_parameter('log_every_n', 30).value)
        self.log_bin_deg = float(self.declare_parameter('log_bin_deg', 30.0).value)
        self._scan_count = 0
        self._last_counts = None

        self.scan_pub = self.create_publisher(LaserScan, 'front_scan', 10)
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            self.cloud_topic,
            self.cloud_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            'PointCloudToLaserScanNode listening on %s (%.1f..%.1f deg).'
            % (self.cloud_topic, self.angle_min_deg, self.angle_max_deg)
        )

    def cloud_callback(self, msg: PointCloud2):
        angle_min = math.radians(self.angle_min_deg)
        angle_max = math.radians(self.angle_max_deg)
        angle_inc = math.radians(self.angle_increment_deg)

        if angle_inc <= 0.0 or angle_max <= angle_min:
            self.get_logger().warn('Invalid angle configuration, skipping scan publish.')
            return

        count = int(math.floor((angle_max - angle_min) / angle_inc)) + 1
        ranges = [math.inf] * count

        total = 0
        z_ok = 0
        forward_ok = 0
        angle_ok = 0
        range_ok = 0
        z_min_seen = None
        z_max_seen = None

        for x, y, z in point_cloud2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True
        ):
            total += 1
            if z_min_seen is None or z < z_min_seen:
                z_min_seen = z
            if z_max_seen is None or z > z_max_seen:
                z_max_seen = z

            if self.use_z_filter and (z < self.z_min or z > self.z_max):
                continue
            z_ok += 1

            forward, lateral = self.select_axes(x, y, z)
            if self.require_positive_forward and forward <= 0.0:
                continue
            forward_ok += 1

            angle = math.atan2(lateral, forward)
            if angle < angle_min or angle > angle_max:
                continue
            angle_ok += 1

            dist = math.hypot(forward, lateral)
            if dist < self.range_min or dist > self.range_max:
                continue
            range_ok += 1

            index = int((angle - angle_min) / angle_inc)
            if 0 <= index < count and dist < ranges[index]:
                ranges[index] = dist

        scan = LaserScan()
        scan.header.stamp = msg.header.stamp
        scan.header.frame_id = self.output_frame or msg.header.frame_id
        scan.angle_min = angle_min
        scan.angle_max = angle_max
        scan.angle_increment = angle_inc
        scan.time_increment = 0.0
        scan.scan_time = 0.0
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges

        self.scan_pub.publish(scan)
        self._last_counts = (total, z_ok, forward_ok, angle_ok, range_ok, z_min_seen, z_max_seen)

        if self.expected_frame and msg.header.frame_id != self.expected_frame:
            self.get_logger().warn(
                'Cloud frame is %s, expected %s. Consider switching cloud_topic.'
                % (msg.header.frame_id, self.expected_frame)
            )

        self._scan_count += 1
        if self._scan_count % max(self.log_every_n, 1) == 0:
            self.log_bins(angle_min, angle_max, angle_inc, ranges)

    def select_axes(self, x: float, y: float, z: float):
        axes = {
            'x': x,
            'y': y,
            'z': z,
        }
        forward = axes.get(self.forward_axis, x)
        lateral = axes.get(self.lateral_axis, y)
        return forward, lateral

    def log_bins(self, angle_min: float, angle_max: float, angle_inc: float, ranges):
        step = math.radians(max(self.log_bin_deg, 1.0))
        angle = angle_min
        bins = []

        zero_index = int(round((0.0 - angle_min) / angle_inc))
        if 0 <= zero_index < len(ranges):
            zero_value = ranges[zero_index]
            if math.isfinite(zero_value):
                bins.append('0=%.3f' % zero_value)
            else:
                bins.append('0=inf')

        while angle <= angle_max + 1e-9:
            index = int(round((angle - angle_min) / angle_inc))
            index = max(0, min(index, len(ranges) - 1))
            value = ranges[index]
            if math.isfinite(value):
                bins.append('%.0f=%.3f' % (math.degrees(angle), value))
            else:
                bins.append('%.0f=inf' % math.degrees(angle))
            angle += step

        counts = ''
        if self._last_counts:
            total, z_ok, forward_ok, angle_ok, range_ok, z_min_seen, z_max_seen = self._last_counts
            z_range = ''
            if z_min_seen is not None and z_max_seen is not None:
                z_range = ' zmin=%.2f zmax=%.2f' % (z_min_seen, z_max_seen)
            counts = ' | counts: total=%d z=%d fwd=%d ang=%d rng=%d%s' % (
                total, z_ok, forward_ok, angle_ok, range_ok, z_range
            )
        self.get_logger().info('front_scan bins: %s%s' % (', '.join(bins), counts))


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudToLaserScanNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
