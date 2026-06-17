#!/usr/bin/env python3

# Multiplexer node for subsumption
# This runs every 0.25 seconds and publishes the highest-priority fresh command.
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
        self.state = {
            'trick':  {'msg': Go2Command(), 'time': None, 'timeout': 1},  # Priority 1 (Highest)
            'approach': {'msg': Go2Command(), 'time': None, 'timeout': 1},  # Priority 2
            'avoid':  {'msg': Go2Command(), 'time': None, 'timeout': 1},  # Priority 3
            'wander': {'msg': Go2Command(), 'time': None, 'timeout': 6}  # Priority 4 (Lowest)
        }
        # General robot state trick/avoid/wander
        self.active_tier = 'none'
        # Allowed states, move/sit/rise_sit/.....
        self.robot_state = {'mode': 'none', 'tick': 0}

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
        self.approach_sub = self.create_subscription(
            Go2Command,
            "/approach_cmd",
            self.approach_callback,
            10
        )
        self.avoidance_sub = self.create_subscription(
            Go2Command,
            "/avoidance_cmd",
            self.avoid_callback,
            10
        )

        # CONTROL LOOP (runs at 4Hz / every 0.25 seconds)
        self.timer = self.create_timer(0.25, self.publish_highest_priority)
        self.get_logger().info("Simple Mux Started. Waiting for commands...")

    def publish_highest_priority(self):
        now = self.get_clock().now()
        selected_msg = Go2Command() # Defaults nil which stops go2

        # Check in order of highest priority to lowest
        if self.is_active('trick', now):
            selected_msg = self.state['trick']['msg']
            self.active_tier = 'trick'
        elif self.is_active('approach', now):
            selected_msg = self.state['approach']['msg']
            self.active_tier = 'approach'
        elif self.is_active('avoid', now):
            selected_msg = self.state['avoid']['msg']
            self.active_tier = 'avoid'
        elif self.is_active('wander', now) and self.should_wander():
            selected_msg = self.state['wander']['msg']
            self.active_tier = 'wander'
        else:
            self.active_tier = 'none'
        
        self.publish_command(selected_msg)

    # Only begin performing wander if command is fresh
    def should_wander(self):
        if self.active_tier == 'wander':
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
        
        # 1. Determine what the new intended mode is
        if msg.command_type == Go2Command.TRICK:
            intended_mode = msg.trick_name
        elif msg.command_type == Go2Command.MOVE:
            intended_mode = 'move'
        else:
            intended_mode = 'stand'

        # 2. INTERCEPTOR: If sitting and wanting to do something else, start rising
        if self.robot_state['mode'] == 'sit' and intended_mode != 'sit':
            self.robot_state = {'mode': 'rise_sit', 'tick': 0}
            s = String()
            s.data = 'rise_sit'
            self.trigger_behaviour_pub.publish(s)
            self.get_logger().info(f"INTERCEPT: Rising from sit before executing {intended_mode}.")
            return # Block the actual intended command for now

        # 3. INTERCEPTOR: If currently rising, hold until 4 ticks have passed
        if self.robot_state['mode'] == 'rise_sit':
            if self.robot_state['tick'] < 4:
                self.robot_state['tick'] += 1
                self.get_logger().info(f"INTERCEPT: Waiting for rise_sit to finish ({self.robot_state['tick']}/4)...")
                return # Block the intended command while rising
            # If tick is >= 4, it falls through to normal execution below!

        # --- NEW STRETCH INTERCEPTOR START ---
        # 3.5 INTERCEPTOR: If currently stretching, block all other commands for 20 ticks (5 seconds)
        if self.robot_state['mode'] == 'stretch':
            if self.robot_state['tick'] < 20:
                self.robot_state['tick'] += 1
                self.get_logger().info(f"INTERCEPT: Waiting for stretch to finish ({self.robot_state['tick']}/20)...")
                return # Block all incoming commands while stretching
            # If tick is >= 20, the stretch is over and it falls through to normal execution!
        # --- NEW STRETCH INTERCEPTOR END ---

        # 4. NORMAL EXECUTION
        if msg.command_type == Go2Command.TRICK:
            s = String()
            s.data = msg.trick_name
            
            # Reset tick to 0 if we are switching to a new trick
            if self.robot_state['mode'] != msg.trick_name:
                self.robot_state = {'mode': msg.trick_name, 'tick': 0}
                
            # Publish for the first 2 ticks (tick 0 and tick 1)
            if self.robot_state['tick'] < 2:
                self.trigger_behaviour_pub.publish(s)
                self.robot_state['tick'] += 1
                self.get_logger().info(f"PUBLISH TRICK ({self.robot_state['tick']}/2): {self.robot_state}; FOR TIER: {self.active_tier}")
            else:
                self.robot_state['tick'] += 1
                self.get_logger().info(f"SKIPPING PUBLISH TRICK: {self.robot_state}; FOR TIER: {self.active_tier}")
                
        elif msg.command_type == Go2Command.MOVE:
            if self.robot_state['mode'] == 'move':
                self.robot_state['tick'] += 1
            else:
                self.robot_state = {'mode': 'move', 'tick': 0}
            self.cmd_vel_pub.publish(msg.twist_command)
            self.get_logger().info(f"PUBLISH MOVE: {self.robot_state}; FOR TIER: {self.active_tier}")
            
        else:
            # If empty command or STAY, just make go2 balance stand
            s = String()
            s.data = 'balance_stand'
            if self.robot_state['mode'] == 'stand':
                self.robot_state['tick'] += 1
                self.get_logger().info(f"SKIPPING PUBLISHING STAND: {self.robot_state}; FOR TIER: {self.active_tier}")
            else:
                self.robot_state = {'mode': 'stand', 'tick': 0}
                self.get_logger().info(f"PUBLISH STAND: {self.robot_state}; FOR TIER: {self.active_tier}")
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

    def approach_callback(self, msg: Go2Command):
        self.state['approach']['msg'] = msg
        self.state['approach']['time'] = self.get_clock().now()

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
