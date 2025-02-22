#!/usr/bin/env python3
import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from traitlets.config.configurable import SingletonConfigurable

class StereoCamera(SingletonConfigurable):
    
    width = 640  # Set desired resolution
    height = 480
    fps = 15
    
    def __init__(self):
        super(StereoCamera, self).__init__()

        # Initialize ROS node
        rospy.init_node('stereo_camera', anonymous=True)

        # Publishers
        self.left_pub = rospy.Publisher('/camera/left/image_raw', Image, queue_size=10)
        self.right_pub = rospy.Publisher('/camera/right/image_raw', Image, queue_size=10)
        self.left_info_pub = rospy.Publisher('/camera/left/camera_info', CameraInfo, queue_size=10)
        self.right_info_pub = rospy.Publisher('/camera/right/camera_info', CameraInfo, queue_size=10)

        self.bridge = CvBridge()
        self.rate = rospy.Rate(10)  # 10Hz

        # Open left and right cameras
        self.left_cap = cv2.VideoCapture(0)   # Adjust based on camera index
        self.right_cap = cv2.VideoCapture(1)

        if not self.left_cap.isOpened() or not self.right_cap.isOpened():
            raise RuntimeError("Failed to open stereo cameras.")

        rospy.loginfo("Stereo Cameras Initialized")

        self.run()

    def get_camera_info(self):
        """Returns dummy camera info (replace with calibration data)."""
        cam_info = CameraInfo()
        cam_info.width = self.width
        cam_info.height = self.height
        cam_info.distortion_model = "plumb_bob"
        cam_info.D = [0, 0, 0, 0, 0]  # Dummy distortion coefficients
        cam_info.K = [500, 0, self.width/2, 0, 500, self.height/2, 0, 0, 1]  # Intrinsics
        return cam_info

    def run(self):
        while not rospy.is_shutdown():
            ret_left, left_img = self.left_cap.read()
            ret_right, right_img = self.right_cap.read()

            if ret_left and ret_right:
                # Publish images
                self.left_pub.publish(self.bridge.cv2_to_imgmsg(left_img, "bgr8"))
                self.right_pub.publish(self.bridge.cv2_to_imgmsg(right_img, "bgr8"))

                # Publish camera info
                left_info = self.get_camera_info()
                right_info = self.get_camera_info()
                self.left_info_pub.publish(left_info)
                self.right_info_pub.publish(right_info)

                rospy.loginfo("Published Stereo Images & Camera Info")
            else:
                rospy.logwarn("Failed to read from one or both cameras.")

            self.rate.sleep()

if __name__ == "__main__":
    try:
        StereoCamera()
    except rospy.ROSInterruptException:
        pass
