#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path, Odometry
import tf.transformations

class SimplePathPlanner:
    def __init__(self):
        rospy.init_node('simple_path_planner')
        
        # Parameters
        self.waypoint_radius = rospy.get_param('~waypoint_radius', 0.5)
        self.lookahead_distance = rospy.get_param('~lookahead_distance', 1.0)
        self.max_angular_speed = rospy.get_param('~max_angular_speed', 1.0)
        self.max_linear_speed = rospy.get_param('~max_linear_speed', 0.5)
        
        # State variables
        self.current_pose = None
        self.path = None
        self.current_waypoint_idx = 0
        
        # Publishers
        self.target_pub = rospy.Publisher('/car_controller/target_point', PoseStamped, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/car_controller/path_following_control', Twist, queue_size=10)
        
        # Subscribers
        self.odom_sub = rospy.Subscriber('/my_car/odom', Odometry, self.odom_callback)
        self.path_sub = rospy.Subscriber('/car_controller/path', Path, self.path_callback)
        
        # Create a simple predefined path
        self.create_predefined_path()
        
        self.rate = rospy.Rate(10)
        
        rospy.loginfo("Simple path planner initialized")
    
    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
    
    def path_callback(self, msg):
        self.path = msg
        self.current_waypoint_idx = 0
    
    def create_predefined_path(self):
        """
        Create a simple predefined path (square)
        """
        self.path = Path()
        self.path.header.frame_id = "odom"
        self.path.header.stamp = rospy.Time.now()
        
        waypoints = [
            (0, 0),
            (3, 0),
            (3, 3),
            (0, 3),
            (0, 0)
        ]
        
        for x, y in waypoints:
            pose = PoseStamped()
            pose.header = self.path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0
            pose.pose.orientation.w = 1.0
            self.path.poses.append(pose)
    
    def get_target_point(self):
        """
        Get target point to navigate to
        """
        if self.path is None or self.current_pose is None:
            return None
        
        # Check if we've reached the current waypoint
        current_waypoint = self.path.poses[self.current_waypoint_idx].pose
        distance_to_waypoint = np.sqrt(
            (current_waypoint.position.x - self.current_pose.position.x)**2 +
            (current_waypoint.position.y - self.current_pose.position.y)**2
        )
        
        if distance_to_waypoint < self.waypoint_radius:
            # Move to next waypoint
            self.current_waypoint_idx += 1
            if self.current_waypoint_idx >= len(self.path.poses):
                # We've reached the end of the path
                self.current_waypoint_idx = 0
        
        return self.path.poses[self.current_waypoint_idx].pose
    
    def calculate_control(self, target_point):
        """
        Calculate control commands to reach target point
        """
        if target_point is None or self.current_pose is None:
            return Twist()
        
        # Extract current position and orientation
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        current_quaternion = [
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w
        ]
        euler = tf.transformations.euler_from_quaternion(current_quaternion)
        current_yaw = euler[2]  # Yaw angle
        
        # Calculate vector to target
        target_x = target_point.position.x
        target_y = target_point.position.y
        
        # Calculate distance to target
        distance = np.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
        
        # Calculate angle to target
        target_angle = np.arctan2(target_y - current_y, target_x - current_x)
        
        # Calculate error in angle
        angle_error = target_angle - current_yaw
        
        # Normalize angle error to [-pi, pi]
        while angle_error > np.pi:
            angle_error -= 2 * np.pi
        while angle_error < -np.pi:
            angle_error += 2 * np.pi
        
        # Calculate angular velocity (proportional control)
        angular_velocity = 0.5 * angle_error
        
        # Limit angular velocity
        angular_velocity = max(-self.max_angular_speed, min(angular_velocity, self.max_angular_speed))
        
        # Calculate linear velocity (reduce speed when turning and near target)
        angular_factor = 1.0 - abs(angle_error) / np.pi
        distance_factor = min(1.0, distance / self.lookahead_distance)
        linear_velocity = self.max_linear_speed * angular_factor * distance_factor
        
        # Create and return Twist message
        twist = Twist()
        twist.linear.x = linear_velocity
        twist.angular.z = angular_velocity
        
        return twist
    
    def run(self):
        rospy.loginfo("Starting path following...")
        
        while not rospy.is_shutdown():
            if self.current_pose is not None:
                # Get target point
                target_point = self.get_target_point()
                
                if target_point is not None:
                    # Publish target point for visualization
                    target_msg = PoseStamped()
                    target_msg.header.frame_id = "odom"
                    target_msg.header.stamp = rospy.Time.now()
                    target_msg.pose = target_point
                    self.target_pub.publish(target_msg)
                    
                    # Calculate control commands
                    twist = self.calculate_control(target_point)
                    
                    # Publish control commands
                    self.cmd_vel_pub.publish(twist)
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        path_planner = SimplePathPlanner()
        path_planner.run()
    except rospy.ROSInterruptException:
        pass