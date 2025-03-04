#!/usr/bin/env python3
import traitlets
from traitlets.config.configurable import SingletonConfigurable
import atexit
import cv2
import rospy
from sensor_msgs.msg import Image
import threading

class StereoCamera(SingletonConfigurable):
    
    # Camera properties
    width = traitlets.Integer(default_value=1280).tag(config=True)   # Reduce for performance
    height = traitlets.Integer(default_value=800).tag(config=True)
    fps = traitlets.Integer(default_value=20).tag(config=True)      # Lower FPS if needed
    left_sensor_id = traitlets.Integer(default_value=0).tag(config=True)
    right_sensor_id = traitlets.Integer(default_value=1).tag(config=True)

    def __init__(self, *args, **kwargs):
        super(StereoCamera, self).__init__(*args, **kwargs)

        self.left_image_raw = Image()
        self.right_image_raw = Image()

        # Initialize ROS node
        rospy.init_node('stereo_camera_publisher', anonymous=True)


        # Publishers
        self.left_image_pub = rospy.Publisher('/stereo/left/image_raw', Image, queue_size=10)
        self.right_image_pub = rospy.Publisher('/stereo/right/image_raw', Image, queue_size=10)

        self.rate = rospy.Rate(30)  # 10 Hz

        # Open cameras using GStreamer
        self.left_cap = cv2.VideoCapture(self._gst_str(sensor_id=self.left_sensor_id), cv2.CAP_GSTREAMER)
        self.right_cap = cv2.VideoCapture(self._gst_str(sensor_id=self.right_sensor_id), cv2.CAP_GSTREAMER)

        # Ensure cameras opened
        if not self.left_cap.isOpened() or not self.right_cap.isOpened():
            raise RuntimeError('Could not open cameras. Check camera connections.')

        atexit.register(self.stop)

    def _gst_str(self, sensor_id):
        """GStreamer pipeline for Jetson Nano"""
        return (
            "nvarguscamerasrc sensor-id={} ! video/x-raw(memory:NVMM), width={}, height={}, "
            "format=(string)NV12, framerate={}/1 ! nvvidconv ! video/x-raw, "
            "width={}, height={}, format=(string)BGRx ! videoconvert ! appsink"
            .format(sensor_id, self.width, self.height, self.fps, self.width, self.height)
        )

    def start(self):
        self.left_thread = threading.Thread(target=self.capture_left)
        self.right_thread = threading.Thread(target=self.capture_right)

        self.left_thread.start()
        self.right_thread.start()


    def capture_left(self):
        while not rospy.is_shutdown():
            left_re, left_image = self.left_cap.read()
            if left_re:
                self.left_image_raw = self.frame_to_ros_image(cv2.flip(left_image, -1), "left")
                self.left_image_pub.publish(self.left_image_raw)
            else:
                rospy.logwarn("Failed to capture left frames.")

    def capture_right(self):
        while not rospy.is_shutdown():
            right_re, right_image = self.right_cap.read()
            if right_re:
                self.right_image_raw = self.frame_to_ros_image(cv2.flip(right_image, -1), "right")
                self.right_image_pub.publish(self.right_image_raw)
            else:
                rospy.logwarn("Failed to capture left frames.")

    def stop(self):
        self.left_cap.release()
        self.right_cap.release()

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
    camera = StereoCamera()
    try:
        camera.start()
    except rospy.ROSInterruptException:
        pass
