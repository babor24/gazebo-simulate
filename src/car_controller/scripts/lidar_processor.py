#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray

class LidarProcessor:
    def __init__(self):
        rospy.init_node('lidar_processor')
        
        # Subscribe to raw LiDAR data
        self.scan_sub = rospy.Subscriber('/lidar/scan', LaserScan, self.scan_callback)
        
        # Publish processed data
        self.obstacles_pub = rospy.Publisher('/car_controller/obstacles', Float32MultiArray, queue_size=10)
        
        # Parameters
        self.num_sectors = rospy.get_param('~num_sectors', 8)
        self.max_range = rospy.get_param('~max_range', 10.0)
        
        self.rate = rospy.Rate(10)  # 10 Hz
        
        rospy.loginfo("LiDAR processor initialized")
    
    def scan_callback(self, scan_msg):
        # Get angle information
        angle_min = scan_msg.angle_min
        angle_max = scan_msg.angle_max
        angle_increment = scan_msg.angle_increment
        
        # Calculate number of points per sector
        points_per_sector = len(scan_msg.ranges) // self.num_sectors
        
        # Initialize sector distances
        sector_distances = []
        
        # Process each sector
        for i in range(self.num_sectors):
            start_idx = i * points_per_sector
            end_idx = (i + 1) * points_per_sector if i < self.num_sectors - 1 else len(scan_msg.ranges)
            
            # Extract valid ranges for this sector
            sector_ranges = [r for r in scan_msg.ranges[start_idx:end_idx] 
                              if r >= scan_msg.range_min and r <= scan_msg.range_max]
            
            # Calculate minimum distance for this sector
            if sector_ranges:
                min_distance = min(sector_ranges)
            else:
                min_distance = self.max_range
            
            sector_distances.append(min_distance)
        
        # Create and publish message with sector distances
        msg = Float32MultiArray()
        msg.data = sector_distances
        self.obstacles_pub.publish(msg)
    
    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    try:
        processor = LidarProcessor()
        processor.run()
    except rospy.ROSInterruptException:
        pass