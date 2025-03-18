#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge, CvBridgeError

class CameraProcessor:
    def __init__(self):
        rospy.init_node('camera_processor')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Subscribe to raw camera data
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        
        # Publish processed data
        self.lane_offset_pub = rospy.Publisher('/car_controller/lane_offset', Float32, queue_size=10)
        self.processed_image_pub = rospy.Publisher('/car_controller/processed_image', Image, queue_size=10)
        
        # Parameters
        self.debug_mode = rospy.get_param('~debug_mode', True)
        
        self.rate = rospy.Rate(10)  # 10 Hz
        
        rospy.loginfo("Camera processor initialized")
    
    def image_callback(self, img_msg):
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
            
            # Process image to detect lane
            lane_offset, processed_image = self.process_image(cv_image)
            
            # Publish lane offset
            offset_msg = Float32()
            offset_msg.data = lane_offset
            self.lane_offset_pub.publish(offset_msg)
            
            # Publish processed image if debug mode is enabled
            if self.debug_mode:
                self.processed_image_pub.publish(self.bridge.cv2_to_imgmsg(processed_image, "bgr8"))
                
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge error: {e}")
    
    def process_image(self, image):
        """
        Process the image to detect lane markings
        Returns: lane_offset (-1 to 1), processed_image
        """
        # Create a copy for visualization
        processed_image = image.copy()
        
        # Convert to HSV color space
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Define range for yellow color (lane markings)
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        
        # Define range for white color (lane markings)
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 25, 255])
        
        # Create masks
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        white_mask = cv2.inRange(hsv, lower_white, upper_white)
        
        # Combine masks
        combined_mask = cv2.bitwise_or(yellow_mask, white_mask)
        
        # Apply the mask
        masked_image = cv2.bitwise_and(image, image, mask=combined_mask)
        
        # Convert to grayscale
        gray = cv2.cvtColor(masked_image, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Apply Canny edge detection
        edges = cv2.Canny(blurred, 50, 150)
        
        # Define region of interest (ROI)
        height, width = edges.shape
        roi_vertices = np.array([
            [(0, height), (width/2 - 50, height/2 + 50), (width/2 + 50, height/2 + 50), (width, height)]
        ], dtype=np.int32)
        
        # Apply ROI mask
        mask = np.zeros_like(edges)
        cv2.fillPoly(mask, roi_vertices, 255)
        roi_edges = cv2.bitwise_and(edges, mask)
        
        # Hough transform to detect lines
        lines = cv2.HoughLinesP(roi_edges, 1, np.pi/180, 50, minLineLength=50, maxLineGap=100)
        
        # Process lines to determine lane offset
        left_lines = []
        right_lines = []
        
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                slope = (y2 - y1) / (x2 - x1) if x2 != x1 else float('inf')
                
                # Exclude horizontal lines
                if abs(slope) < 0.3:
                    continue
                
                # Classify lines as left or right based on slope
                if slope < 0:  # Negative slope = left lane
                    left_lines.append((x1, y1, x2, y2, slope))
                else:  # Positive slope = right lane
                    right_lines.append((x1, y1, x2, y2, slope))
                
                # Draw lines on the processed image
                cv2.line(processed_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        
        # Calculate lane center and offset
        image_center = width / 2
        lane_center = image_center  # Default to center if no lines detected
        
        # Calculate average positions of left and right lanes
        if left_lines and right_lines:
            left_x = np.mean([np.mean([x1, x2]) for x1, y1, x2, y2, _ in left_lines])
            right_x = np.mean([np.mean([x1, x2]) for x1, y1, x2, y2, _ in right_lines])
            lane_center = (left_x + right_x) / 2
        elif left_lines:
            left_x = np.mean([np.mean([x1, x2]) for x1, y1, x2, y2, _ in left_lines])
            lane_center = left_x + 160  # Assume standard lane width
        elif right_lines:
            right_x = np.mean([np.mean([x1, x2]) for x1, y1, x2, y2, _ in right_lines])
            lane_center = right_x - 160  # Assume standard lane width
        
        # Calculate offset from center (-1 to 1)
        lane_offset = (lane_center - image_center) / (image_center)
        
        # Draw lane center and offset on processed image
        cv2.line(processed_image, (int(lane_center), height), (int(lane_center), height - 50), (255, 0, 0), 2)
        cv2.line(processed_image, (int(image_center), height), (int(image_center), height - 50), (0, 0, 255), 2)
        cv2.putText(processed_image, f"Offset: {lane_offset:.2f}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        
        return lane_offset, processed_image
    
    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    try:
        processor = CameraProcessor()
        processor.run()
    except rospy.ROSInterruptException:
        pass