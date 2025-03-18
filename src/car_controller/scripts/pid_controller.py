#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import Twist

class PIDController:
    def __init__(self):
        rospy.init_node('pid_controller')
        
        # PID parameters for linear velocity
        self.kp_linear = rospy.get_param('~kp_linear', 0.5)
        self.ki_linear = rospy.get_param('~ki_linear', 0.01)
        self.kd_linear = rospy.get_param('~kd_linear', 0.1)
        
        # PID parameters for angular velocity
        self.kp_angular = rospy.get_param('~kp_angular', 1.0)
        self.ki_angular = rospy.get_param('~ki_angular', 0.01)
        self.kd_angular = rospy.get_param('~kd_angular', 0.1)
        
        # Target values
        self.target_speed = rospy.get_param('~target_speed', 0.5)
        self.target_distance = rospy.get_param('~target_distance', 1.5)
        self.min_distance = rospy.get_param('~min_distance', 0.5)
        
        # Constraints
        self.max_speed = rospy.get_param('~max_speed', 1.0)
        self.max_angular_speed = rospy.get_param('~max_angular_speed', 1.0)
        
        # PID variables
        self.linear_error_sum = 0.0
        self.last_linear_error = 0.0
        self.angular_error_sum = 0.0
        self.last_angular_error = 0.0
        
        # Data from sensors
        self.lane_offset = 0.0
        self.obstacle_distances = [10.0] * 8  # Default to max range
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/my_car/cmd_vel', Twist, queue_size=10)
        
        # Subscribers
        self.lane_sub = rospy.Subscriber('/car_controller/lane_offset', Float32, self.lane_callback)
        self.obstacles_sub = rospy.Subscriber('/car_controller/obstacles', Float32MultiArray, self.obstacles_callback)
        
        self.rate = rospy.Rate(20)  # 20 Hz
        
        rospy.loginfo("PID controller initialized")
    
    def lane_callback(self, msg):
        self.lane_offset = msg.data
    
    def obstacles_callback(self, msg):
        self.obstacle_distances = msg.data
    
    def calculate_target_velocity(self):
        """
        Calculate target velocity based on obstacle distances
        """
        # Consider front sectors (assuming 8 sectors with 0 being front)
        front_sectors = [0, 7, 1, 6]  # Front and slightly to the sides
        front_distances = [self.obstacle_distances[i] for i in front_sectors]
        min_front_distance = min(front_distances)
        
        # Reduce speed as obstacles get closer
        if min_front_distance < self.min_distance:
            return 0.0  # Stop if too close
        elif min_front_distance < self.target_distance:
            # Gradually slow down based on distance
            return self.target_speed * (min_front_distance - self.min_distance) / (self.target_distance - self.min_distance)
        else:
            return self.target_speed
    
    def calculate_target_angular_velocity(self):
        """
        Calculate target angular velocity based on lane offset and obstacles
        """
        # Obstacle avoidance steering
        obstacle_steering = 0.0
        
        # Check if obstacles are closer on one side
        left_sectors = [1, 2, 3]
        right_sectors = [5, 6, 7]
        
        left_min_distance = min([self.obstacle_distances[i] for i in left_sectors])
        right_min_distance = min([self.obstacle_distances[i] for i in right_sectors])
        
        # If obstacles are closer on one side, steer away from them
        if left_min_distance < self.target_distance or right_min_distance < self.target_distance:
            diff = right_min_distance - left_min_distance
            obstacle_steering = np.clip(diff / self.target_distance, -1.0, 1.0)
        
        # Weight between lane following and obstacle avoidance
        # Lane following has more weight when obstacles are far
        front_min_distance = min(self.obstacle_distances[0], self.obstacle_distances[7])
        obstacle_weight = max(0, min(1, (self.target_distance - front_min_distance) / self.target_distance))
        lane_weight = 1.0 - obstacle_weight
        
        # Calculate combined steering
        return -self.lane_offset * lane_weight + obstacle_steering * obstacle_weight
    
    def pid_linear_control(self, target_velocity):
        """
        PID controller for linear velocity
        """
        # Calculate error
        error = target_velocity - self.last_linear_error
        
        # Update error sum (integral term)
        self.linear_error_sum += error
        self.linear_error_sum = np.clip(self.linear_error_sum, -1.0, 1.0)  # Anti-windup
        
        # Calculate error derivative
        error_derivative = error - self.last_linear_error
        self.last_linear_error = error
        
        # PID formula
        output = (self.kp_linear * error + 
                  self.ki_linear * self.linear_error_sum + 
                  self.kd_linear * error_derivative)
        
        # Limit output
        return np.clip(output, -self.max_speed, self.max_speed)
    
    def pid_angular_control(self, target_angular):
        """
        PID controller for angular velocity
        """
        # Calculate error
        error = target_angular - self.last_angular_error
        
        # Update error sum (integral term)
        self.angular_error_sum += error
        self.angular_error_sum = np.clip(self.angular_error_sum, -1.0, 1.0)  # Anti-windup
        
        # Calculate error derivative
        error_derivative = error - self.last_angular_error
        self.last_angular_error = error
        
        # PID formula
        output = (self.kp_angular * error + 
                  self.ki_angular * self.angular_error_sum + 
                  self.kd_angular * error_derivative)
        
        # Limit output
        return np.clip(output, -self.max_angular_speed, self.max_angular_speed)
    
    def run(self):
        rospy.loginfo("Starting PID control loop...")
        
        while not rospy.is_shutdown():
            # Calculate target velocities
            target_velocity = self.calculate_target_velocity()
            target_angular = self.calculate_target_angular_velocity()
            
            # Apply PID control
            linear_velocity = self.pid_linear_control(target_velocity)
            angular_velocity = self.pid_angular_control(target_angular)
            
            # Create and publish twist message
            twist = Twist()
            twist.linear.x = linear_velocity
            twist.angular.z = angular_velocity
            self.cmd_vel_pub.publish(twist)
            
           # Debug output
            rospy.logdebug(f"Target velocity: {target_velocity:.2f}, Output: {linear_velocity:.2f}")
            rospy.logdebug(f"Target angular: {target_angular:.2f}, Output: {angular_velocity:.2f}")
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = PIDController()
        controller.run()
    except rospy.ROSInterruptException:
        pass