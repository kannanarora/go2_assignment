# Multiplexer node for subsumption

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String
from unitree_api.msg import Request
from go2_interfaces.msg import Go2Command

class MuxNode(Node):
    def __init__(self):
        super().__init__("mux_node")

        # PUBLISHERS
        self.go2_command_pub = "/go2_command"

        # STATE STORAGE
        # We store the latest message and the time it was received
        self.state = {
            'trick':  {'msg': Go2Command(), 'time': None, 'timeout': 0.5}, # Priority 1 (Highest)
            'avoid':  {'msg': Go2Command(), 'time': None, 'timeout': 0.2}, # Priority 2
            'wander': {'msg': Go2Command(), 'time': None, 'timeout': 0.5}  # Priority 3 (Lowest)
        }
        self.priorites = {wander: 10, trick: 20, obsacle_avoidance: 30}
        
        # NODE SUBSCRIPTIONS
        self.wander_sub = self.create_subscription(
            Request,
            "/wander_cmd",
            self.wander_callback,
            10
        )
        self.trick_sub = self.create_subscription(
            Request,
            "/trick_cmd",
            self.trick_callback,
            10
        )
        self.avoidance_sub = self.create_subscription(
            Request,
            "/avoidance_cmd",
            self.avoid_callback,
            10
        )

        # CONTROL LOOP (runs at 20Hz / every 0.05 seconds)
        self.timer = self.create_timer(0.05, self.publish_highest_priority)
        self.get_logger().info("Simple Mux Started. Waiting for commands...")

    def publish_highest_priority(self):
        now = self.get_clock().now()
        selected_msg = Go2Command() # Defaults to all zeros (stop)
        active_source = "None"

        # Check in order of highest priority to lowest
        if self.is_active('avoid', now):
            selected_msg = self.state['avoid']['msg']
            active_source = "Avoid"
        elif self.is_active('trick', now):
            selected_msg = self.state['trick']['msg']
            active_source = "Trick"
        elif self.is_active('wander', now):
            selected_msg = self.state['wander']['msg']
            active_source = "Wander"

        self.go2_command_pub.publish(selected_msg)

    # Check if message is recent enough to be action
    def is_active(self, source, current_time):
        last_time = self.state[source]['time']
        if last_time is None:
            return False

        # Calculate how long ago the last message arrived
        elapsed_seconds = (current_time - last_time).nanoseconds / 1e9
        return elapsed_seconds < self.state[source]['timeout']
    
    def wander_callback(self, msg: Go2Command):
        self.state['wander']['msg'] = msg
        self.state['wander']['time'] = self.get_clock().now()

    def trick_callback(self, msg: Go2Command):
        self.state['trick']['msg'] = msg
        self.state['trick']['time'] = self.get_clock().now()

    def avoid_callback(self, msg: Go2Command):
        self.state['avoid']['msg'] = msg
        self.state['avoid']['time'] = self.get_clock().now()

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
