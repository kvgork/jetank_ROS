#!/usr/bin/env python3

import rospy
import message_filters
from sensor_msgs.msg import Image, PointCloud2
from stereo_msgs.msg import DisparityImage
from cv_bridge import CvBridge
import cv2
import numpy as np
from image_geometry import StereoCameraModel
import sensor_msgs.point_cloud2 as pc2

class StereoPointCloudGenerator:
    def __init__(self):
        rospy.init_node('stereo_point_cloud_generator', anonymous=True)
        
        # Get parameters
        self.camera_info_topic_left = rospy.get_param('~camera_info_left', '/stereo/left/camera_info')
        self.camera_info_topic_right = rospy.get_param('~camera_info_right', '/stereo/right/camera_info')
        self.image_topic_left = rospy.get_param('~image_left', '/stereo/left/image_rect')
        self.image_topic_right = rospy.get_param('~image_right', '/stereo/right/image_rect')
        self.output_cloud_topic = rospy.get_param('~output_cloud', '/stereo/points2')
        self.queue_size = rospy.get_param('~queue_size', 10)
        
        # Initialize camera model
        self.stereo_model = StereoCameraModel()
        self.bridge = CvBridge()
        
        # Subscribe to camera_info topics to initialize the stereo model
        self.camera_info_left = rospy.wait_for_message(self.camera_info_topic_left, rospy.AnyMsg)
        self.camera_info_right = rospy.wait_for_message(self.camera_info_topic_right, rospy.AnyMsg)
        self.stereo_model.fromCameraInfo(self.camera_info_left, self.camera_info_right)
        
        # Initialize stereo matcher
        self.stereo_matcher = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=128,
            blockSize=11,
            P1=8 * 3 * 11**2,
            P2=32 * 3 * 11**2,
            disp12MaxDiff=1,
            uniquenessRatio=10,
            speckleWindowSize=100,
            speckleRange=32
        )
        
        # Publishers
        self.cloud_pub = rospy.Publisher(self.output_cloud_topic, PointCloud2, queue_size=self.queue_size)
        self.disparity_pub = rospy.Publisher('/stereo/disparity', DisparityImage, queue_size=self.queue_size)
        
        # Subscribers
        self.left_sub = message_filters.Subscriber(self.image_topic_left, Image)
        self.right_sub = message_filters.Subscriber(self.image_topic_right, Image)
        
        # Time synchronizer for the stereo images
        ts = message_filters.ApproximateTimeSynchronizer(
            [self.left_sub, self.right_sub], 
            queue_size=self.queue_size, 
            slop=0.1
        )
        ts.registerCallback(self.stereo_callback)
        
        rospy.loginfo("Stereo Point Cloud Generator initialized!")
    
    def stereo_callback(self, left_msg, right_msg):
        try:
            # Convert ROS Image messages to OpenCV images
            left_image = self.bridge.imgmsg_to_cv2(left_msg, "mono8")
            right_image = self.bridge.imgmsg_to_cv2(right_msg, "mono8")
            
            # Compute disparity map
            disparity = self.stereo_matcher.compute(left_image, right_image).astype(np.float32) / 16.0
            
            # Create point cloud
            height, width = disparity.shape
            points = np.zeros((height, width, 3), dtype=np.float32)
            
            # Generate 3D points
            for v in range(height):
                for u in range(width):
                    disp = disparity[v, u]
                    if disp > 0:
                        # Project disparity to 3D point
                        (x, y, z) = self.stereo_model.projectPixelTo3d((u, v), disp)
                        points[v, u] = [x, y, z]
            
            # Filter out points with invalid depth
            mask = np.logical_and(
                np.logical_and(
                    np.isfinite(points[:,:,0]),
                    np.isfinite(points[:,:,1])
                ),
                np.isfinite(points[:,:,2])
            )
            
            # Convert to point cloud
            points_filtered = points[mask]
            
            # Create PointCloud2 message
            header = left_msg.header
            cloud_msg = pc2.create_cloud_xyz32(header, points_filtered)
            
            # Publish the point cloud
            self.cloud_pub.publish(cloud_msg)
            
            # Also publish the disparity image for visualization (optional)
            disparity_msg = DisparityImage()
            disparity_msg.header = left_msg.header
            disparity_msg.image = self.bridge.cv2_to_imgmsg(disparity, "32FC1")
            disparity_msg.f = self.stereo_model.left.fx()
            disparity_msg.T = self.stereo_model.baseline
            disparity_msg.min_disparity = 0.0
            disparity_msg.max_disparity = 128.0
            self.disparity_pub.publish(disparity_msg)
            
            rospy.loginfo(f"Published point cloud with {len(points_filtered)} points")
            
        except Exception as e:
            rospy.logerr(f"Error processing stereo images: {e}")

if __name__ == '__main__':
    try:
        node = StereoPointCloudGenerator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass