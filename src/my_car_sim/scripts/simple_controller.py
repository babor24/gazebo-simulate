#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class SimpleController:
    def __init__(self):
        rospy.init_node('simple_controller')
        self.cmd_pub = rospy.Publisher('/my_car/cmd_vel', Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber('/lidar/scan', LaserScan, self.scan_callback)
        self.rate = rospy.Rate(10)  # 10 Hz
        self.twist = Twist()
        self.min_obstacle_dist = 1.0  # Jarak minimum ke obstacle

    def scan_callback(self, scan_msg):
        # Cek jarak ke obstacle di depan
        front_ranges = scan_msg.ranges[0:20] + scan_msg.ranges[340:359]
        min_front_range = min([r for r in front_ranges if r > 0.1])
        
        # Jika ada obstacle di depan, berbelok
        if min_front_range < self.min_obstacle_dist:
            self.twist.linear.x = 0.2
            self.twist.angular.z = 0.5
        else:
            self.twist.linear.x = 0.5
            self.twist.angular.z = 0.0
        
        self.cmd_pub.publish(self.twist)

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = SimpleController()
        controller.run()
    except rospy.ROSInterruptException:
        pass