"""
Tier 1 - Reactive Layer: Safety Monitor Node

Reads obstacle distances from /sportmodestate and stops the robot if any
direction is within 0.5 m of an obstacle.

How it works:
  - SportModeState contains range_obstacle[4]: firmware-computed obstacle
    distances in 4 directions. No LiDAR processing needed.
  - If min(range_obstacle) < stop_dist → publish /safety_override + send 'stop'
  - If obstacle clears → clear /safety_override + send 'stand'

Subscriptions:
  /sportmodestate    - velocity + range_obstacle[4] (firmware-computed distances)
  /camera/image_raw  - camera feed (person detection stub, not yet implemented)

Publications:
  /safety_override    std_msgs/Bool    True = too close, False = clear
  /trigger_behaviour  std_msgs/String  'stop' or 'stand', bypasses Tiers 2 & 3

Parameters (config/behaviour_params.yaml):
  obstacle_stop_distance  - metres to stop from obstacle (default 0.5)

TEST:
  ros2 topic echo /safety_override
  ros2 topic echo /trigger_behaviour
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from unitree_go.msg import SportModeState


class SafetyMonitorNode(Node):

    def __init__(self):
        super().__init__('safety_monitor_node')

        # Parameters — change values in config/behaviour_params.yaml, not here
        self.declare_parameter('obstacle_stop_distance', 0.5)
        self.stop_dist = self.get_parameter('obstacle_stop_distance').value

        # State
        self.safety_active = False  # True while an obstacle is within stop_dist
        self.robot_speed   = 0.0   # cached from /sportmodestate (m/s)

        # Subscribers
        self.create_subscription(SportModeState, '/sportmodestate',   self._on_sportmode, 10)
        self.create_subscription(Image,          '/camera/image_raw', self._on_camera,    10)

        # Publishers
        self.safety_pub  = self.create_publisher(Bool,   '/safety_override',   10)
        self.trigger_pub = self.create_publisher(String, '/trigger_behaviour', 10)

        self.get_logger().info(f'SafetyMonitorNode ready — stop distance: {self.stop_dist} m')

    # ---------- callbacks ----------

    def _on_sportmode(self, msg: SportModeState) -> None:
        """
        Fires on every /sportmodestate update (~50 Hz).

        range_obstacle[4] contains firmware-computed obstacle distances in
        4 directions. We take the minimum across all directions so the robot
        stops regardless of which side the wall is on.

        Zeros mean no reading — filtered out. Values above 10 m are open space.
        """
        self.robot_speed = math.hypot(msg.velocity[0], msg.velocity[1])

        # Keep only valid readings (non-zero, within a plausible range)
        valid = [d for d in msg.range_obstacle if 0.0 < d < 10.0]
        if not valid:
            return

        min_dist = min(valid)

        if min_dist < self.stop_dist and not self.safety_active:
            self.get_logger().warn(
                f'Obstacle at {min_dist:.2f} m (threshold {self.stop_dist} m) '
                '— activating safety override'
            )
            self._set_safety(True)

        elif min_dist >= self.stop_dist and self.safety_active:
            self.get_logger().info(
                f'Obstacle cleared ({min_dist:.2f} m) — releasing safety override'
            )
            self._set_safety(False)

    def _on_camera(self, msg: Image) -> None:
        """Stub: person detection via camera (not yet implemented)."""
        pass

    # ---------- helper ----------

    def _set_safety(self, active: bool) -> None:
        """
        Switch safety state, signal Tier 2, and command the robot directly.
          active=True  → 'stop'  (skipped if robot is already stationary)
          active=False → 'stand' (recover from stop)
        """
        self.safety_active = active
        self.safety_pub.publish(Bool(data=active))

        if active:
            if self.robot_speed > 0.05:      # don't spam stop if already still
                self.trigger_pub.publish(String(data='stop'))
        else:
            self.trigger_pub.publish(String(data='stand'))


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
