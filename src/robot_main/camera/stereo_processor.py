#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import struct
from std_msgs.msg import Header
import message_filters
import ctypes
import math

# Direct image conversion without cv_bridge
def imgmsg_to_array(img_msg):
    if img_msg.encoding != "mono8":
        rospy.logerr("Only mono8 images are supported")
        return None
    
    dtype = np.uint8
    height = img_msg.height
    width = img_msg.width
    
    # Create numpy array from byte array
    data = np.frombuffer(img_msg.data, dtype=dtype).reshape(height, width)
    return data

class StereoProcessor:
    def __init__(self):
        self.left_info = None
        self.right_info = None
        
        # Subscribe to camera info once to get calibration data
        rospy.Subscriber('/stereo/left/camera_info', CameraInfo, self.left_info_callback)
        rospy.Subscriber('/stereo/right/camera_info', CameraInfo, self.right_info_callback)
        
        # Wait for camera calibration
        rospy.loginfo("Waiting for camera calibration...")
        while self.left_info is None or self.right_info is None:
            rospy.sleep(0.1)
        
        # Set up stereo parameters
        self.setup_stereo_params()
        
        # Publishers
        self.cloud_pub = rospy.Publisher('/stereo/points2', PointCloud2, queue_size=1)
        
        # Synchronize image subscribers
        self.left_sub = message_filters.Subscriber('/stereo/left/image_raw', Image)
        self.right_sub = message_filters.Subscriber('/stereo/right/image_raw', Image)
        
        # ApproximateTimeSynchronizer for handling slight timing differences
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.left_sub, self.right_sub], 10, 0.1)
        self.ts.registerCallback(self.stereo_callback)
        
        rospy.loginfo("Stereo processor initialized")
    
    def left_info_callback(self, msg):
        self.left_info = msg
    
    def right_info_callback(self, msg):
        self.right_info = msg
    
    def setup_stereo_params(self):
        # Extract baseline from calibration (distance between cameras)
        # Assuming the translation component is in the P matrix of right camera
        self.baseline = -self.right_info.P[3] / self.right_info.P[0]
        
        # Focal length (assuming same for both cameras)
        self.focal_length = self.left_info.P[0]
        
        # Principal points
        self.cx_left = self.left_info.P[2]
        self.cy_left = self.left_info.P[6]
        
        rospy.loginfo(f"Stereo baseline: {self.baseline} meters")
        rospy.loginfo(f"Focal length: {self.focal_length} pixels")
        
        # Disparity range
        self.min_disparity = 0
        self.max_disparity = 64
        self.block_size = 15
        
        # Configure a simple block matcher (would typically use OpenCV, but we're implementing manually)
        self.window_size = 9  # Must be odd
    
    def compute_disparity(self, left_img, right_img):
        """
        Simple Sum of Absolute Differences (SAD) block matching for disparity
        Note: This is a very basic implementation. For production use, 
        consider using a C++ node with OpenCV's StereoBM/SGBM
        """
        height, width = left_img.shape
        half_window = self.window_size // 2
        disparity = np.zeros((height, width), dtype=np.float32)
        
        # Only process every 4th pixel (downsampling for speed on Jetson Nano)
        for y in range(half_window, height - half_window, 4):
            for x in range(half_window, width - half_window, 4):
                best_disparity = 0
                min_sad = float('inf')
                
                # Left image template
                template = left_img[y-half_window:y+half_window+1, 
                                    x-half_window:x+half_window+1]
                
                # Search in right image
                for d in range(self.min_disparity, self.max_disparity):
                    if x - d < half_window:
                        continue
                    
                    # Right image patch
                    patch = right_img[y-half_window:y+half_window+1, 
                                     x-d-half_window:x-d+half_window+1]
                    
                    # SAD calculation
                    sad = np.sum(np.abs(template - patch))
                    
                    if sad < min_sad:
                        min_sad = sad
                        best_disparity = d
                
                # Set disparity for this block
                disparity[y-half_window:y+half_window+1, 
                          x-half_window:x+half_window+1] = best_disparity
        
        return disparity
    
    def create_point_cloud(self, left_img, disparity, header):
        """
        Convert disparity map to 3D point cloud
        """
        height, width = disparity.shape
        points = []
        
        # Only process points with valid disparity
        for y in range(0, height, 4):  # Downsample for speed
            for x in range(0, width, 4):
                d = disparity[y, x]
                
                # Skip invalid disparities
                if d <= 0:
                    continue
                
                # Calculate 3D coordinates
                Z = self.focal_length * self.baseline / d  # depth
                X = (x - self.cx_left) * Z / self.focal_length
                Y = (y - self.cy_left) * Z / self.focal_length
                
                # Limit depth range (adjust as needed)
                if 0.1 < Z < 10.0:
                    # Pack RGB from grayscale
                    gray = left_img[y, x]
                    rgb = struct.unpack('I', struct.pack('BBBB', gray, gray, gray, 255))[0]
                    points.append([X, Y, Z, rgb])
        
        # Create PointCloud2 message
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgb', 12, PointField.UINT32, 1)
        ]
        
        pc2_msg = pc2.create_cloud(header, fields, points)
        return pc2_msg
    
    def stereo_callback(self, left_msg, right_msg):
        # Convert images without cv_bridge
        left_img = imgmsg_to_array(left_msg)
        right_img = imgmsg_to_array(right_msg)
        
        if left_img is None or right_img is None:
            return
        
        # Compute disparity
        disparity = self.compute_disparity(left_img, right_img)
        
        # Create point cloud
        cloud_msg = self.create_point_cloud(left_img, disparity, left_msg.header)
        
        # Publish point cloud
        self.cloud_pub.publish(cloud_msg)
        rospy.loginfo("Published point cloud")

if __name__ == '__main__':
    rospy.init_node('stereo_processor')
    processor = StereoProcessor()
    rospy.spin()
