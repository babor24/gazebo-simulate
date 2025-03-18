#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class PIDController:
    def __init__(self):
        rospy.init_node('pid_controller')
        
        # PID parameters
        self.kp = rospy.get_param('~kp', 1.0)  # Proportional gain
        self.ki = rospy.get_param('~ki', 0.0)  # Integral gain
        self.kd = rospy.get_param('~kd', 0.1)  # Derivative gain
        
        # PID variables
        self.error_sum = 0.0
        self.last_error = 0.0
        self.target_distance = 1.0  # Target distance to maintain from obstacles
        
        # Publishers and subscribers
        self.cmd_pub = rospy.Publisher('/my_car/cmd_vel', Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber('/lidar/scan', LaserScan, self.scan_callback)
        self.odom_sub = rospy.Subscriber('/my_car/odom', Odometry, self.odom_callback)
        
        self.twist = Twist()
        self.current_velocity = 0.0
        self.rate = rospy.Rate(10)  # 10 Hz
        
        rospy.loginfo("PID Controller started with kp=%f, ki=%f, kd=%f", self.kp, self.ki, self.kd)
    
    def scan_callback(self, scan_msg):
        # Process LiDAR scan data
        front_ranges = scan_msg.ranges[0:20] + scan_msg.ranges[340:359]
        front_ranges = [r for r in front_ranges if r > 0.1 and r < 30.0]  # Filter invalid readings
        
        if not front_ranges:
            return
            
        # Calculate distance to closest obstacle
        min_front_range = min(front_ranges)
        
        # Calculate error (difference between actual and target distance)
        error = min_front_range - self.target_distance
        
        # PID calculations
        self.error_sum += error * 0.1  # dt = 0.1s
        error_diff = error - self.last_error
        
        # Calculate control output
        output = self.kp * error + self.ki * self.error_sum + self.kd * error_diff
        
        # Set speed and turning based on PID output
        if min_front_range < self.target_distance - 0.2:
            # Too close to obstacle, turn more
            self.twist.linear.x = 0.1
            self.twist.angular.z = 0.5 - output
        elif min_front_range > self.target_distance + 1.0:
            # Far from obstacle, go faster
            self.twist.linear.x = 0.5
            self.twist.angular.z = -output * 0.5
        else:
            # Maintain distance with controlled speed
            self.twist.linear.x = 0.3
            self.twist.angular.z = -output * 0.5
        
        # Publish velocity command
        self.cmd_pub.publish(self.twist)
        
        # Update last error for next iteration
        self.last_error = error
        
        rospy.loginfo("Distance: %.2f, Error: %.2f, Output: %.2f, Linear: %.2f, Angular: %.2f", 
                     min_front_range, error, output, self.twist.linear.x, self.twist.angular.z)
    
    def odom_callback(self, odom_msg):
        # Get current velocity for potential future improvements
        self.current_velocity = odom_msg.twist.twist.linear.x
    
    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = PIDController()
        controller.run()
    except rospy.ROSInterruptException:
        pass