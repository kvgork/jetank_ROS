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


class Camera(SingletonConfigurable):
    
    value = traitlets.Any()
    
    # config
    width = traitlets.Integer(default_value=224).tag(config=True)
    height = traitlets.Integer(default_value=224).tag(config=True)
    fps = traitlets.Integer(default_value=21).tag(config=True)
    capture_width = traitlets.Integer(default_value=3280).tag(config=True)
    capture_height = traitlets.Integer(default_value=2464).tag(config=True)

    def __init__(self, *args, **kwargs):
        self.value = np.empty((self.height, self.width, 3), dtype=np.uint8)
        super(Camera, self).__init__(*args, **kwargs)

        # Initialize the ROS node
        rospy.init_node('camera_publisher', anonymous=True)

        # Create a publisher to publish to the /camera/image_raw topic
        self.image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)

        # Use CvBridge to convert between OpenCV and ROS image formats
        self.bridge = CvBridge()

        # Set the publishing rate
        self.rate = rospy.Rate(10)  # 10 Hz

        try:
            self.cap = cv2.VideoCapture(self._gst_str(), cv2.CAP_GSTREAMER)

            re, image = self.cap.read()

            if not re:
                raise RuntimeError('Could not read image from camera.')

            self.value = image
            self.start()
        except:
            self.stop()
            raise RuntimeError(
                'Could not initialize camera.  Please see error trace.')

        atexit.register(self.stop)

    # def _capture_frames(self):
    #     while True:
    #         re, image = self.cap.read()
    #         if re:
    #             self.value = image
    #         else:
    #             break
                
    def _gst_str(self):
        # return 'nvarguscamerasrc ! video/x-raw(memory:NVMM), width=%d, height=%d, format=(string)NV12, framerate=(fraction)%d/1 ! nvvidconv ! video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! videoconvert ! appsink' % (
        #         self.capture_width, self.capture_height, self.fps, self.width, self.height)
        return (
        "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=%d, height=%d, format=(string)NV12, "
        "framerate=%d/1 ! nvvidconv ! video/x-raw, width=%d, height=%d, format=(string)BGRx ! "
        "videoconvert ! appsink"
        % (self.capture_width, self.capture_height, self.fps, self.width, self.height)
    )
    
    def start(self):
        if not self.cap.isOpened():
            self.cap.open(self._gst_str(), cv2.CAP_GSTREAMER)
        # if not hasattr(self, 'thread') or not self.thread.isAlive():
        #     self.thread = threading.Thread(target=self._capture_frames)
        #     self.thread.start()
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if ret:
                # Convert the OpenCV frame to a ROS Image message
                ros_image = self.frame_to_ros_image(frame)

                # Publish the image
                self.image_pub.publish(ros_image)
                rospy.loginfo("Published camera frame.")
            else:
                rospy.logwarn("Failed to capture frame.")
            
            self.rate.sleep()

    def stop(self):
        if hasattr(self, 'cap'):
            self.cap.release()
        # if hasattr(self, 'thread'):
        #     self.thread.join()
            
    def restart(self):
        self.stop()
        self.start()
    
    def frame_to_ros_image(self, frame):
        ros_image = Image()
        ros_image.header.stamp = rospy.Time.now()
        ros_image.height = frame.shape[0]  # Image height
        ros_image.width = frame.shape[1]   # Image width
        ros_image.encoding = "bgr8"
        ros_image.step = frame.shape[1] * 3  # Width * Channels
        ros_image.data = frame.tobytes()
        return ros_image

if __name__ == '__main__':
    try:
        Camera()
    except rospy.ROSInterruptException:
        pass