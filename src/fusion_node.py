#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import cv2
import numpy as np
import message_filters

class CameraLidarFusion(Node):
    def __init__(self):
        super().__init__('fusion_node')
        self.bridge = CvBridge()
        
        # Camera Intrinsic Parameters (Adjust based on your 1080x720 Gazebo Camera)
        self.fx = 900.0  
        self.fy = 900.0
        self.cx = 540.0
        self.cy = 360.0

        # Subscriptions with Time Synchronizer
        self.image_sub = message_filters.Subscriber(self, Image, '/camera/image_raw')
        self.lidar_sub = message_filters.Subscriber(self, LaserScan, '/scan')
        
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.lidar_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.process_fusion)

    def process_fusion(self, img_msg, scan_msg):
        # Convert ROS Image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        
        # Project Lidar points
        for i, dist in enumerate(scan_msg.ranges):
            if dist < scan_msg.range_min or dist > scan_msg.range_max:
                continue

            # 1. Polar to Cartesian (Lidar Frame)
            angle = scan_msg.angle_min + i * scan_msg.angle_increment
            lx = dist * np.cos(angle)
            ly = dist * np.sin(angle)

            # 2. Transform to Camera Optical Frame
            # Offset between laser (0.122) and camera (0.3) = 0.178m
            tx = 0.178 
            
            # Reorient axes for Camera Optical (Z-forward, X-right, Y-down)
            z_cam = lx - tx
            x_cam = -ly
            y_cam = 0.0 # Assuming flat ground

            if z_cam > 0: # Only points in front of camera
                u = int((self.fx * x_cam / z_cam) + self.cx)
                v = int((self.fy * y_cam / z_cam) + self.cy)

                # 3. Overlay on Image
                if 0 <= u < cv_image.shape[1] and 0 <= v < cv_image.shape[0]:
                    cv2.circle(cv_image, (u, v), 2, (0, 255, 0), -1)

        cv2.imshow("Fused View", cv_image)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = CameraLidarFusion()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()