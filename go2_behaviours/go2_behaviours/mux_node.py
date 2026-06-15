# Multiplexer node for subsumption
# This runs every 0.1 second and is always giving a command every 0.1 seconds
# If there is no recent command then it send an empty command to make the robot
# do nothing but stand

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String
from go2_interfaces.msg import Go2Command

class MuxNode(Node):
    def __init__(self):
        super().__init__("mux_node")

        # PUBLISHERS
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )
        self.trigger_behaviour_pub = self.create_publisher(
            String,
            "/trigger_behaviour",
            10
        )

        # STATE STORAGE
        # We store the latest message and the time it was received
        # TODO tune timeouts
        self.state = {
            'trick':  {'msg': Go2Command(), 'time': None, 'timeout': 0.5}, # Priority 1 (Highest)
            'avoid':  {'msg': Go2Command(), 'time': None, 'timeout': 0.2}, # Priority 2
            'wander': {'msg': Go2Command(), 'time': None, 'timeout': 0.25}  # Priority 3 (Lowest)
        }
        
        # NODE SUBSCRIPTIONS
        self.wander_sub = self.create_subscription(
            Go2Command,
            "/wander_cmd",
            self.wander_callback,
            10
        )
        self.trick_sub = self.create_subscription(
            Go2Command,
            "/trick_cmd",
            self.trick_callback,
            10
        )
        self.avoidance_sub = self.create_subscription(
            Go2Command,
            "/avoidance_cmd",
            self.avoid_callback,
            10
        )

        # CONTROL LOOP (runs at 10Hz / every 0.1 seconds)
        self.timer = self.create_timer(0.1, self.publish_highest_priority)
        self.get_logger().info("Simple Mux Started. Waiting for commands...")

    def publish_highest_priority(self):
        now = self.get_clock().now()
        selected_msg = Go2Command() # Defaults nil which stops go2
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

        self.publish_command(selected_msg)

    # If twist publish to cmd_vel
    # if trick publish directly to trick
    def publish_command(self, msg):
        if msg.command_type == Go2Command.MOVE:
            self.cmd_vel_pub.publish(msg.twist_command)
        elif msg.command_type == Go2Command.TRICK:
            s = String()
            # If empty command, just make go2 balance stand
            s.data = msg.trick_name
            self.trigger_behaviour_pub.publish(s)
        else:
            # If empty command or STAY, just make go2 balance stand
            s = String()
            s.data = 'balance_stand'
            self.trigger_behaviour_pub.publish(s)

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
    node = MuxNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
