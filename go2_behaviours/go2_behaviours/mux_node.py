# Multiplexer node for subsumption
# This runs every 0.1 second and is always giving a command every 0.1 seconds
# If there is no recent command then it send an empty command to make the robot
# do nothing but stand

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String
from go2_interfaces.msg import Go2Command

# TODO just do the last wonder command as a base
# however, if switching from a different command, it should only perform the command if it's fresh


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
            'trick':  {'msg': Go2Command(), 'time': None, 'timeout': 4}, # Priority 1 (Highest)
            'avoid':  {'msg': Go2Command(), 'time': None, 'timeout': 1}, # Priority 2
            'wander': {'msg': Go2Command(), 'time': None, 'timeout': 5}  # Priority 3 (Lowest)
        }
        # General robot state trick/avoid/wander
        self.active_teir = 'none'
        self.robot_state = 'none'

        self.wander_begin_timeout = 0.5 # So wander commands are only executed when fresh!
        
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

        # CONTROL LOOP (runs at 4Hz / every 0.1 seconds)
        self.timer = self.create_timer(0.25, self.publish_highest_priority)
        self.get_logger().info("Simple Mux Started. Waiting for commands...")

    def publish_highest_priority(self):
        now = self.get_clock().now()
        selected_msg = Go2Command() # Defaults nil which stops go2

        # Check in order of highest priority to lowest
        # 
        if self.is_active('avoid', now):
            selected_msg = self.state['avoid']['msg']
            self.active_teir = 'avoid'
        elif self.is_active('trick', now):
            selected_msg = self.state['trick']['msg']
            self.active_teir = 'trick'
        elif self.is_active('wander', now) and self.should_wander():
            selected_msg = self.state['wander']['msg']
            self.active_teir = 'wander'
        else:
            self.active_teir = 'none'
        
        self.publish_command(selected_msg)

    # Only begin performing wander if command is fresh
    def should_wander(self):
        if self.active_teir == 'wander':
            return True
        
        last_time = self.state['wander']['time']
        if last_time is None:
            return False
            
        now = self.get_clock().now()
        elapsed_seconds = (now - last_time).nanoseconds / 1e9
        
        if elapsed_seconds < self.wander_begin_timeout:
            return True

        return False

    # If twist publish to cmd_vel
    # if trick publish directly to trick
    def publish_command(self, msg):
        self.get_logger().info(f"NEW ROBOT STATE: {self.robot_state} FOR TIER: {self.active_teir}")

        if msg.command_type == Go2Command.MOVE:
            self.robot_state = 'move'
            self.cmd_vel_pub.publish(msg.twist_command)
        elif msg.command_type == Go2Command.TRICK:
            s = String()
            self.robot_state = msg.trick_name
            s.data = msg.trick_name
            self.trigger_behaviour_pub.publish(s)
        else:
            # If empty command or STAY, just make go2 balance stand
            s = String()
            s.data = 'balance_stand'
            self.robot_state = 'stand'
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
