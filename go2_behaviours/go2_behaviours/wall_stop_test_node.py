"""
Wall Stop Test Node

Simplest possible test:
  1. Waits 2 seconds for the wrapper to be ready
  2. Sends 'walk' to start moving forward
  3. Watches range_obstacle from /sportmodestate
  4. Sends 'stop' the moment any obstacle is within 0.5 m
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from unitree_go.msg import SportModeState

STOP_DISTANCE = 0.5  # metres


class WallStopTestNode(Node):

    def __init__(self):
        super().__init__('wall_stop_test_node')

        self.stopped = False  # once stopped, don't send any more commands

        # Publisher — sends commands to the sport client wrapper
        self.cmd_pub = self.create_publisher(String, '/trigger_behaviour', 10)

        # Subscriber — reads obstacle distances from the firmware
        self.create_subscription(SportModeState, '/sportmodestate', self._on_sportmode, 10)

        # Wait 2 s then start walking (gives the wrapper time to come up)
        self._start_timer = self.create_timer(2.0, self._start_walking)

        # Repeatedly publishes 'walk' at 10 Hz to keep the robot moving since the firmware stops the robot if it doesn't receive continuous commands
        self._walk_timer = self.create_timer(0.1, self._keep_walking)
        self._walking = False  # only start sending after the 2 s delay

        self.get_logger().info(f'WallStopTestNode ready — will stop at {STOP_DISTANCE} m')

    def _start_walking(self):
        """Enable the walk loop after the initial 2 s delay."""
        self.get_logger().info('Starting walk...')
        self._walking = True
        self._start_timer.cancel()  # run once only

    def _keep_walking(self):
        """Publish 'walk' at 10 Hz to keep the robot moving until stopped."""
        if self._walking and not self.stopped:
            self.cmd_pub.publish(String(data='walk'))

    def _on_sportmode(self, msg: SportModeState) -> None:
        """Check obstacle distances on every update. Stop if anything is too close."""
        if self.stopped:
            return

        valid = [d for d in msg.range_obstacle if 0.0 < d < 10.0]
        if not valid:
            return

        min_dist = min(valid)
        self.get_logger().info(f'Nearest obstacle: {min_dist:.2f} m', throttle_duration_sec=1.0)

        if min_dist < STOP_DISTANCE:
            self.get_logger().warn(f'Wall at {min_dist:.2f} m — stopping!')
            self.cmd_pub.publish(String(data='stop'))
            self.stopped = True


def main(args=None):
    rclpy.init(args=args)
    node = WallStopTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
