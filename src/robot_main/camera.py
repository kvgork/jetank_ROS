#!/usr/bin/env python3
# Based on jetbot
import traitlets
from traitlets.config.configurable import SingletonConfigurable
import atexit
import cv2
# import threading
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class StereoCamera(SingletonConfigurable):
    
    left_value = traitlets.Any()
    right_value = traitlets.Any()

    # config
    width = traitlets.Integer(default_value=224).tag(config=True)
    height = traitlets.Integer(default_value=224).tag(config=True)
    fps = traitlets.Integer(default_value=21).tag(config=True)
    capture_width = traitlets.Integer(default_value=3280).tag(config=True)
    capture_height = traitlets.Integer(default_value=2464).tag(config=True)
    left_sensor_id = traitlets.Integer(default_value=0).tag(config=True)
    right_sensor_id = traitlets.Integer(default_value=1).tag(config=True)

    def __init__(self, *args, **kwargs):
        super(StereoCamera, self).__init__(*args, **kwargs)
        self.left_value = np.empty((self.height, self.width, 3), dtype=np.uint8)
        self.right_value = np.empty((self.height, self.width, 3), dtype=np.uint8)

        # Initialize the ROS node
        rospy.init_node('stereo_camera_publisher', anonymous=True)

        # Create a publisher to publish to the /camera/image_raw topic
        self.left_image_pub = rospy.Publisher('/camera/left/image_raw', Image, queue_size=10)
        self.right_image_pub = rospy.Publisher('/camera/right/image_raw', Image, queue_size=10)

        # Use CvBridge to convert between OpenCV and ROS image formats
        self.bridge = CvBridge()

        # Set the publishing rate
        self.rate = rospy.Rate(10)  # 10 Hz

        try:
            self.left_cap = cv2.VideoCapture(self._gst_str(sensor_id=self.left_sensor_id), cv2.CAP_GSTREAMER)
            self.right_cap = cv2.VideoCapture(self._gst_str(sensor_id=self.right_sensor_id), cv2.CAP_GSTREAMER)
            
            left_re, left_image = self.left_cap.read()
            right_re, right_image = self.right_cap.read()

            if not left_re:
                raise RuntimeError('Could not read image from left camera.')
            if not right_re:
                raise RuntimeError('Could not read image from right camera.')

            self.left_value = left_image
            self.right_value = right_image

        except Exception as e:
            rospy.logerr(f"Camera initialization failed: {e}")


        atexit.register(self.stop)

                
    def _gst_str(self, sensor_id):
        return (
            "nvarguscamerasrc sensor-id=%d ! video/x-raw(memory:NVMM), width=%d, height=%d, "
            "format=(string)NV12, framerate=%d/1 ! nvvidconv ! video/x-raw, "
            "width=%d, height=%d, format=(string)BGRx ! videoconvert ! appsink"
            % (sensor_id, self.capture_width, self.capture_height, self.fps, self.width, self.height)
        )
    
    def start(self):
        if not self.left_cap.isOpened():
            self.left_cap.open(self._gst_str(sensor_id=self.left_sensor_id), cv2.CAP_GSTREAMER)
        if not self.right_cap.isOpened():
            self.right_cap.open(self._gst_str(sensor_id=self.right_sensor_id), cv2.CAP_GSTREAMER)

        while not rospy.is_shutdown():
            left_re, left_image = self.left_cap.read()
            right_re, right_image = self.right_cap.read()
            if left_re and right_re:
                # Publish the image
                self.left_image_pub.publish(self.frame_to_ros_image(left_image, "left"))
                self.right_image_pub.publish(self.frame_to_ros_image(right_image, "right"))
                rospy.loginfo("Published stereo camera frames.")
            else:
                rospy.logwarn("Failed to capture frames from both cameras.")
                continue

            self.rate.sleep()

    def stop(self):
        if self.left_cap.isOpened():
            self.left_cap.release()
        if self.right_cap.isOpened():
            self.right_cap.release()
            
    def restart(self):
        self.stop()
        self.start()

    def frame_to_ros_image(self, frame, side):
        ros_image = Image()
        ros_image.header.stamp = rospy.Time.now()
        ros_image.header.frame_id = side
        ros_image.height = frame.shape[0]  # Image height
        ros_image.width = frame.shape[1]   # Image width
        ros_image.encoding = "bgr8"
        ros_image.step = frame.shape[1] * 3  # Width * Channels
        ros_image.data = frame.tobytes()
        return ros_image

if __name__ == '__main__':
    Camera = StereoCamera()
    try:
        Camera.start()
    except rospy.ROSInterruptException:
        pass