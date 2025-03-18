#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2

class AutonomousDriver:
    def __init__(self):
        rospy.init_node('autonomous_driver')
        
        # Parameters
        self.min_front_distance = rospy.get_param('~min_front_distance', 1.0)
        self.max_speed = rospy.get_param('~max_speed', 0.5)
        self.max_angular_speed = rospy.get_param('~max_angular_speed', 1.0)
        
        # PID parameters for linear velocity
        self.kp_linear = rospy.get_param('~kp_linear', this.default_kp_linear())
        self.ki_linear = rospy.get_param('~ki_linear', 0.01)
        self.kd_linear = rospy.get_param('~kd_linear', 0.05)
        
        # PID parameters for angular velocity
        self.kp_angular = rospy.get_param('~kp_angular', 1.0)
        self.ki_angular = rospy.get_param('~ki_angular', 0.01)
        self.kd_angular = rospy.get_param('~kd_angular', 0.1)
        
        # PID variables
        self.linear_error_sum = 0.0
        self.last_linear_error = 0.0
        self.angular_error_sum = 0.0
        self.last_angular_error = 0.0
        
        # Target values
        self.target_distance = 2.0  # Target distance to maintain from obstacles
        self.target_direction = 0.0  # Target direction (0 = straight ahead)
        
        # Robot state
        self.current_linear_speed = 0.0
        self.current_angular_speed = 0.0
        self.obstacle_detected = False
        self.lane_center_offset = 0.0  # Offset from lane center (from camera)
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/my_car/cmd_vel', Twist, queue_size=10)
        
        # Subscribers
        self.scan_sub = rospy.Subscriber('/lidar/scan', LaserScan, self.scan_callback)
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        
        self.rate = rospy.Rate(20)  # 20 Hz control loop
        
        rospy.loginfo("Autonomous driver initialized")
    
    def default_kp_linear(self):
        # Adjust Kp based on the robot and environment
        return 0.5

    def scan_callback(self, scan_msg):
        # Extract ranges in front of the robot (assuming 0 degrees is front)
        front_angle_range = 30  # Consider ±30 degrees as "front"
        front_indices = list(range(0, front_angle_range)) + list(range(len(scan_msg.ranges) - front_angle_range, len(scan_msg.ranges)))
        
        # Filter out invalid measurements
        front_ranges = [r for r in [scan_msg.ranges[i] for i in front_indices] if r > scan_msg.range_min and r < scan_msg.range_max]
        
        if front_ranges:
            min_front_range = min(front_ranges)
            self.obstacle_detected = min_front_range < self.min_front_distance
            
            # Update target distance for PID control
            self.front_distance = min_front_range
        else:
            # If no valid measurements, assume safe distance
            self.obstacle_detected = False
            self.front_distance = scan_msg.range_max
        
        # Also check for obstacles to the sides for steering decisions
        left_indices = range(30, 60)
        right_indices = range(300, 330)
        
        left_ranges = [r for r in [scan_msg.ranges[i] for i in left_indices] if r > scan_msg.range_min and r < scan_msg.range_max]
        right_ranges = [r for r in [scan_msg.ranges[i] for i in right_indices] if r > scan_msg.range_min and r < scan_msg.range_max]
        
        # Calculate average distances to each side
        self.left_distance = np.mean(left_ranges) if left_ranges else scan_msg.range_max
        self.right_distance = np.mean(right_ranges) if right_ranges else scan_msg.range_max

    def image_callback(self, img_msg):
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
            
            # Process image to detect lane (simplified for tutorial)
            self.lane_center_offset = self.detect_lane(cv_image)
            
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")
    
    def detect_lane(self, img):
        """
        A simplified lane detection function
        Returns offset from center of lane (-1.0 to 1.0, where 0 is centered)
        """
        try:
            # Convert to grayscale
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            
            # Apply Gaussian blur
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            
            # Apply threshold to get binary image (road vs. non-road)
            _, binary = cv2.threshold(blurred, 120, 255, cv2.THRESH_BINARY)
            
            # Find contours
            contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                # Find largest contour (assumed to be the road)
                largest_contour = max(contours, key=cv2.contourArea)
                
                # Get bounding box
                x, y, w, h = cv2.boundingRect(largest_contour)
                
                # Calculate center of contour vs. center of image
                contour_center_x = x + w/2
                image_center_x = img.shape[1]/2
                
                # Calculate offset (-1 to 1)
                offset = (contour_center_x - image_center_x) / (img.shape[1]/2)
                return offset
            else:
                return 0.0
        except Exception as e:
            rospy.logerr(f"Lane detection error: {e}")
            return 0.0
    
    def pid_linear_control(self):
        """
        PID controller for linear velocity based on front obstacle distance
        """
        # Calculate error (difference between target and actual)
        error = self.target_distance - self.front_distance
        
        # Update error sum for integral term
        self.linear_error_sum += error
        
        # Anti-windup: limit the error sum
        self.linear_error_sum = max(-1.0, min(1.0, self.linear_error_sum))
        
        # Calculate error delta for derivative term
        delta_error = error - self.last_linear_error
        self.last_linear_error = error
        
        # PID formula
        output = self.kp_linear * error + self.ki_linear * self.linear_error_sum + self.kd_linear * delta_error
        
        # Limit the speed
        if self.obstacle_detected:
            return 0.0  # Stop if obstacle detected
        else:
            return max(-self.max_speed, min(self.max_speed, output))
    
    def pid_angular_control(self):
        """
        PID controller for angular velocity based on lane offset and obstacle avoidance
        """
        # Weigh lane offset vs. obstacle avoidance
        obstacle_steering = 0.0
        if self.left_distance < self.right_distance:
            # Obstacle more on left, steer right
            obstacle_steering = 0.5
        elif self.right_distance < self.left_distance:
            # Obstacle more on right, steer left
            obstacle_steering = -0.5
        
        # Combine lane following with obstacle avoidance
        # Lane following has more weight when no obstacle, obstacle avoidance has more weight when obstacle is close
        obstacle_weight = max(0, min(1, (self.target_distance - self.front_distance) / self.target_distance))
        lane_weight = 1.0 - obstacle_weight
        
        # Calculate combined error
        error = lane_weight * self.lane_center_offset + obstacle_weight * obstacle_steering
        
        # Update error sum for integral term
        self.angular_error_sum += error
        
        # Anti-windup: limit the error sum
        self.angular_error_sum = max(-1.0, min(1.0, self.angular_error_sum))
        
        # Calculate error delta for derivative term
        delta_error = error - self.last_angular_error
        self.last_angular_error = error
        
        # PID formula
        output = self.kp_angular * error + self.ki_angular * self.angular_error_sum + self.kd_angular * delta_error
        
        # Limit the angular speed
        return max(-self.max_angular_speed, min(self.max_angular_speed, output))
    
    def run(self):
        """
        Main control loop
        """
        rospy.loginfo("Starting autonomous control...")
        
        while not rospy.is_shutdown():
            # Calculate control actions
            linear_velocity = self.pid_linear_control()
            angular_velocity = self.pid_angular_control()
            
            # Create and publish Twist message
            cmd_vel = Twist()
            cmd_vel.linear.x = linear_velocity
            cmd_vel.angular.z = angular_velocity
            self.cmd_vel_pub.publish(cmd_vel)
            
            # Print debug info
            rospy.logdebug(f"Linear: {linear_velocity:.2f}, Angular: {angular_velocity:.2f}, " +
                         f"Obstacle: {self.obstacle_detected}, Lane offset: {self.lane_center_offset:.2f}")
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = AutonomousDriver()
        controller.run()
    except rospy.ROSInterruptException:
        pass