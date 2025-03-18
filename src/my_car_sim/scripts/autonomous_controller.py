#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from pid import PID

class AutonomousController:
    def __init__(self):
        rospy.init_node('autonomous_controller')
        
        # Inisialisasi PID controller untuk kecepatan linear dan angular
        self.pid_linear = PID(kp=1.0, ki=0.1, kd=0.01)
        self.pid_angular = PID(kp=1.0, ki=0.1, kd=0.01)
        
        # Publisher untuk mengirim perintah kecepatan
        self.cmd_pub = rospy.Publisher('/my_car/cmd_vel', Twist, queue_size=10)
        
        # Subscriber untuk menerima data LiDAR
        self.scan_sub = rospy.Subscriber('/lidar/scan', LaserScan, self.scan_callback)
        
        # Parameter kontrol
        self.min_obstacle_dist = 1.0  # Jarak minimum ke obstacle
        self.target_distance = 0.5    # Jarak target dari obstacle
        self.rate = rospy.Rate(10)    # 10 Hz
        
        self.twist = Twist()

    def scan_callback(self, scan_msg):
        # Cek jarak ke obstacle di depan
        front_ranges = scan_msg.ranges[0:20] + scan_msg.ranges[340:359]
        min_front_range = min([r for r in front_ranges if r > 0.1])
        
        # Jika ada obstacle di depan, belok
        if min_front_range < self.min_obstacle_dist:
            rospy.loginfo("Objek terdeteksi di depan! Mobil akan belok.")
            
            # Berhenti sejenak
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.cmd_pub.publish(self.twist)
            rospy.sleep(0.5)  # Berhenti selama 0.5 detik
            
            # Putar ke kiri atau kanan
            if self.should_turn_left(scan_msg):
                self.twist.angular.z = 0.5  # Belok kiri
            else:
                self.twist.angular.z = -0.5  # Belok kanan
            
            self.twist.linear.x = 0.2  # Maju perlahan sambil belok
        else:
            # Jika tidak ada obstacle, maju lurus
            self.twist.linear.x = 0.5
            self.twist.angular.z = 0.0
        
        # Publikasikan perintah kecepatan
        self.cmd_pub.publish(self.twist)

    def should_turn_left(self, scan_msg):
        # Cek jarak di sisi kiri dan kanan untuk memutuskan belok kiri atau kanan
        left_ranges = scan_msg.ranges[30:90]  # Rentang sudut untuk sisi kiri
        right_ranges = scan_msg.ranges[270:330]  # Rentang sudut untuk sisi kanan
        
        # Hitung jarak rata-rata di sisi kiri dan kanan
        avg_left = sum(left_ranges) / len(left_ranges)
        avg_right = sum(right_ranges) / len(right_ranges)
        
        # Belok ke arah yang lebih terbuka (jarak lebih jauh)
        return avg_left > avg_right

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = AutonomousController()
        controller.run()
    except rospy.ROSInterruptException:
        pass