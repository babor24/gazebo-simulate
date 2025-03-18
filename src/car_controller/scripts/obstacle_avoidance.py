#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray, Float32
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Twist

class ObstacleAvoidance:
    def __init__(self):
        rospy.init_node('obstacle_avoidance')
        
        # Parameters
        self.safety_distance = rospy.get_param('~safety_distance', 0.5)
        self.look_ahead_distance = rospy.get_param('~look_ahead_distance', 2.0)
        self.num_paths = rospy.get_param('~num_paths', 7)
        self.path_width = rospy.get_param('~path_width', 0.3)
        
        # Path scoring weights
        self.distance_weight = rospy.get_param('~distance_weight', 0.4)
        self.clearance_weight = rospy.get_param('~clearance_weight', 0.3)
        self.direction_weight = rospy.get_param('~direction_weight', 0.3)
        
        # State variables
        self.obstacle_distances = None
        self.lane_offset = 0.0
        self.current_steering = 0.0
        
        # Publisher
        self.recommended_steering_pub = rospy.Publisher('/car_controller/recommended_steering', Float32, queue_size=10)
        self.path_scores_pub = rospy.Publisher('/car_controller/path_scores', Float32MultiArray, queue_size=10)
        self.control_pub = rospy.Publisher('/car_controller/obstacle_avoidance_control', Twist, queue_size=10)
        
        # Subscribers
        self.obstacles_sub = rospy.Subscriber('/car_controller/obstacles', Float32MultiArray, self.obstacles_callback)
        self.lane_sub = rospy.Subscriber('/car_controller/lane_offset', Float32, self.lane_callback)
        self.cmd_vel_sub = rospy.Subscriber('/my_car/cmd_vel', Twist, self.cmd_vel_callback)
        
        self.rate = rospy.Rate(10)
        
        rospy.loginfo("Obstacle avoidance system initialized")
    
    def obstacles_callback(self, msg):
        self.obstacle_distances = np.array(msg.data)
    
    def lane_callback(self, msg):
        self.lane_offset = msg.data
    
    def cmd_vel_callback(self, msg):
        self.current_steering = msg.angular.z
    
    def evaluate_paths(self):
        """
        Evaluate potential paths based on obstacle data and lane position
        """
        if self.obstacle_distances is None:
            return 0.0, []
        
        # Generate potential steering angles
        max_steering = 0.5  # Maximum steering angle in radians
        steering_angles = np.linspace(-max_steering, max_steering, self.num_paths)
        
        # Calculate path scores
        path_scores = np.zeros(self.num_paths)
        
        for i, angle in enumerate(steering_angles):
            # Calculate path sectors (which LiDAR sectors this path would travel through)
            if angle < -0.3:
                path_sectors = [2, 1, 0]  # Sharp left
            elif angle < -0.1:
                path_sectors = [1, 0, 7]  # Moderate left
            elif angle < 0.1:
                path_sectors = [0, 7]     # Straight
            elif angle < 0.3:
                path_sectors = [7, 0, 1]  # Moderate right
            else:
                path_sectors = [6, 7, 0]  # Sharp right
            
            # Calculate minimum distance to obstacles along this path
            path_distances = [self.obstacle_distances[s] for s in path_sectors]
            min_distance = min(path_distances)
            
            # Calculate clearance (higher is better)
            clearance = min_distance - self.safety_distance
            clearance = max(clearance, 0)  # Clearance can't be negative
            
            # Calculate direction score (higher when path aligns with lane)
            direction_deviation = abs(angle + self.lane_offset)  # Angle compensates for lane offset
            direction_score = max(0, 1 - direction_deviation)
            
            # Calculate continuity score (higher when path is similar to current path)
            continuity_deviation = abs(angle - self.current_steering)
            continuity_score = max(0, 1 - continuity_deviation)
            
            # Calculate path score (weighted sum of factors)
            path_scores[i] = (
                self.distance_weight * min(1.0, min_distance / self.look_ahead_distance) +
                self.clearance_weight * min(1.0, clearance / self.look_ahead_distance) +
                self.direction_weight * direction_score
            )
        
        # Find best path
        best_path_idx = np.argmax(path_scores)
        best_steering = steering_angles[best_path_idx]
        
        return best_steering, path_scores.tolist()
    
    def run(self):
        rospy.loginfo("Starting obstacle avoidance...")
        
        while not rospy.is_shutdown():
            if self.obstacle_distances is not None:
                # Evaluate paths and get best steering angle
                best_steering, path_scores = self.evaluate_paths()
                
                # Publish recommended steering
                steering_msg = Float32()
                steering_msg.data = best_steering
                self.recommended_steering_pub.publish(steering_msg)
                
                # Publish path scores for visualization
                scores_msg = Float32MultiArray()
                scores_msg.data = path_scores
                self.path_scores_pub.publish(scores_msg)
                
                # Publish control message
                control_msg = Twist()
                
                # Determine speed based on obstacle proximity
                front_distance = min(self.obstacle_distances[0], self.obstacle_distances[7])
                if front_distance < self.safety_distance:
                    control_msg.linear.x = 0.0  # Stop if too close
                else:
                    max_speed = 0.5
                    # Reduce speed with sharper turns
                    control_msg.linear.x = max_speed * (1.0 - abs(best_steering) / 0.5)
                
                control_msg.angular.z = best_steering
                self.control_pub.publish(control_msg)
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        obstacle_avoidance = ObstacleAvoidance()
        obstacle_avoidance.run()
    except rospy.ROSInterruptException:
        pass