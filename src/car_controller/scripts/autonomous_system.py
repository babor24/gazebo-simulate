#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path

class AutonomousSystem:
    def __init__(self):
        rospy.init_node('autonomous_system')
        
        # Parameters
        self.control_mode = rospy.get_param('~control_mode', 'obstacle_avoidance')  # 'lane_following', 'obstacle_avoidance', 'path_following'
        self.path_following_weight = rospy.get_param('~path_following_weight', 0.0)
        self.obstacle_avoidance_weight = rospy.get_param('~obstacle_avoidance_weight', 0.7)
        self.lane_following_weight = rospy.get_param('~lane_following_weight', 0.3)
        
        # Control inputs from different systems
        self.pid_control = Twist()
        self.obstacle_avoidance_control = Twist()
        self.path_following_control = Twist()
        
        # Flags to check if we've received data
        self.pid_received = False
        self.obstacle_avoidance_received = False
        self.path_following_received = False
        
        # Publisher
        self.cmd_vel_pub = rospy.Publisher('/my_car/cmd_vel', Twist, queue_size=10)
        
        # Subscribers
        self.pid_sub = rospy.Subscriber('/car_controller/pid_control', Twist, self.pid_callback)
        self.obstacle_sub = rospy.Subscriber('/car_controller/obstacle_avoidance_control', Twist, self.obstacle_callback)
        self.path_sub = rospy.Subscriber('/car_controller/path_following_control', Twist, self.path_callback)
        self.control_mode_sub = rospy.Subscriber('/car_controller/control_mode', String, self.control_mode_callback)
        
        self.rate = rospy.Rate(20)  # 20 Hz
        
        rospy.loginfo("Autonomous system initialized with mode: %s", self.control_mode)
    
    def pid_callback(self, msg):
        self.pid_control = msg
        self.pid_received = True
    
    def obstacle_callback(self, msg):
        self.obstacle_avoidance_control = msg
        self.obstacle_avoidance_received = True
    
    def path_callback(self, msg):
        self.path_following_control = msg
        self.path_following_received = True
    
    def control_mode_callback(self, msg):
        self.control_mode = msg.data
        rospy.loginfo("Control mode changed to: %s", self.control_mode)
    
    def blend_controls(self):
        """
        Blend control commands from different subsystems based on weights
        """
        # Initialize blended control
        blended_control = Twist()
        
        # Handle cases when data isn't available yet
        if not self.pid_received:
            rospy.logwarn("No PID control data received yet")
        if not self.obstacle_avoidance_received and self.obstacle_avoidance_weight > 0:
            rospy.logwarn("No obstacle avoidance data received yet")
        if not self.path_following_received and self.path_following_weight > 0:
            rospy.logwarn("No path following data received yet")
        
        if self.control_mode == 'lane_following':
            # Pure lane following using PID control
            if self.pid_received:
                return self.pid_control
            else:
                return Twist()
        
        elif self.control_mode == 'obstacle_avoidance':
            # Pure obstacle avoidance
            if self.obstacle_avoidance_received:
                return self.obstacle_avoidance_control
            else:
                return Twist()
        
        elif self.control_mode == 'path_following':
            # Pure path following
            if self.path_following_received:
                return self.path_following_control
            else:
                return Twist()
        
        else:  # 'blended' mode - mix all controls based on weights
            # Calculate total weight for normalization
            total_weight = 0.0
            count = 0
            
            if self.pid_received and self.lane_following_weight > 0:
                blended_control.linear.x += self.lane_following_weight * self.pid_control.linear.x
                blended_control.angular.z += self.lane_following_weight * self.pid_control.angular.z
                total_weight += self.lane_following_weight
                count += 1
            
            if self.obstacle_avoidance_received and self.obstacle_avoidance_weight > 0:
                blended_control.linear.x += self.obstacle_avoidance_weight * self.obstacle_avoidance_control.linear.x
                blended_control.angular.z += self.obstacle_avoidance_weight * self.obstacle_avoidance_control.angular.z
                total_weight += self.obstacle_avoidance_weight
                count += 1
            
            if self.path_following_received and self.path_following_weight > 0:
                blended_control.linear.x += self.path_following_weight * self.path_following_control.linear.x
                blended_control.angular.z += self.path_following_weight * self.path_following_control.angular.z
                total_weight += self.path_following_weight
                count += 1
            
            # Normalize if we have any data
            if count > 0 and total_weight > 0:
                blended_control.linear.x /= total_weight
                blended_control.angular.z /= total_weight
            
            return blended_control
    
    def run(self):
        rospy.loginfo("Starting autonomous control system...")
        
        while not rospy.is_shutdown():
            # Blend controls from different subsystems
            cmd_vel = self.blend_controls()
            
            # Publish blended control
            self.cmd_vel_pub.publish(cmd_vel)
            
            # Debug output
            rospy.logdebug(f"Control mode: {self.control_mode}, Linear: {cmd_vel.linear.x:.2f}, Angular: {cmd_vel.angular.z:.2f}")
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        system = AutonomousSystem()
        system.run()
    except rospy.ROSInterruptException:
        pass