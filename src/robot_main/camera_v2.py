#!/usr/bin/env python3
import traitlets
from traitlets.config.configurable import SingletonConfigurable
import atexit
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image, PointCloud2, PointField
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2

class StereoCamera(SingletonConfigurable):
    
    # Camera properties
    width = traitlets.Integer(default_value=640).tag(config=True)   # Reduce for performance
    height = traitlets.Integer(default_value=480).tag(config=True)
    fps = traitlets.Integer(default_value=60).tag(config=True)      # Lower FPS if needed
    left_sensor_id = traitlets.Integer(default_value=0).tag(config=True)
    right_sensor_id = traitlets.Integer(default_value=1).tag(config=True)

    def __init__(self, *args, **kwargs):
        super(StereoCamera, self).__init__(*args, **kwargs)

        # Initialize ROS node
        rospy.init_node('stereo_camera_publisher', anonymous=True)

        # Publishers
        self.left_image_pub = rospy.Publisher('/camera/left/image_raw', Image, queue_size=10)
        self.right_image_pub = rospy.Publisher('/camera/right/image_raw', Image, queue_size=10)
        self.point_cloud_pub = rospy.Publisher('/camera/depth/points', PointCloud2, queue_size=10)

        self.bridge = CvBridge()
        self.rate = rospy.Rate(100)  # 10 Hz

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
        while not rospy.is_shutdown():
            left_re, left_image = self.left_cap.read()
            right_re, right_image = self.right_cap.read()

            if left_re and right_re:
                # self.left_image_pub.publish(self.bridge.cv2_to_imgmsg(left_image, "bgr8"))
                # self.right_image_pub.publish(self.bridge.cv2_to_imgmsg(right_image, "bgr8"))

                self.left_image_pub.publish(self.frame_to_ros_image(left_image, "left"))
                self.right_image_pub.publish(self.frame_to_ros_image(right_image, "right"))
                
                # Generate and publish point cloud
                self.generate_point_cloud(left_image, right_image)
            else:
                rospy.logwarn("Failed to capture stereo frames.")

            self.rate.sleep()

    def stop(self):
        self.left_cap.release()
        self.right_cap.release()

    def generate_point_cloud(self, left_image, right_image):
        """Generates and publishes a 3D point cloud from stereo images"""
        
        # Convert images to grayscale
        left_gray = cv2.cvtColor(left_image, cv2.COLOR_BGR2GRAY)
        right_gray = cv2.cvtColor(right_image, cv2.COLOR_BGR2GRAY)

        # Stereo matching using CUDA-optimized StereoSGBM
        min_disp = 0
        num_disp = 64  # Must be divisible by 16
        block_size = 5
        
        stereo = cv2.StereoSGBM_create(
            minDisparity=min_disp,
            numDisparities=num_disp,
            blockSize=block_size,
            P1=8 * 3 * block_size**2,
            P2=32 * 3 * block_size**2,
            disp12MaxDiff=1,
            uniquenessRatio=10,
            speckleWindowSize=100,
            speckleRange=32
        )

        disparity = stereo.compute(left_gray, right_gray).astype(np.float32) / 16.0  # Normalize

        # Q matrix (adjust for your camera calibration)
        Q = np.array([[1, 0, 0, -self.width / 2],
                      [0, -1, 0, self.height / 2],
                      [0, 0, 0, -1],  # -fx (focal length)
                      [0, 0, 1, 0]])

        # Convert disparity to 3D points
        points_3D = cv2.reprojectImageTo3D(disparity, Q)

        # Filter invalid points
        mask = disparity > 0
        points = points_3D[mask]
        colors = left_image[mask]

        # Convert RGB values to float32 (ROS expects float RGB)
        rgb = np.left_shift(colors[:, 2], 16) + np.left_shift(colors[:, 1], 8) + colors[:, 0]
        rgb = rgb.astype(np.float32)

        # Create a structured array for PointCloud2 format
        cloud_data = np.zeros(points.shape[0], dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32), ('rgb', np.float32)])
        cloud_data['x'], cloud_data['y'], cloud_data['z'], cloud_data['rgb'] = points[:, 0], points[:, 1], points[:, 2], rgb

        # Define PointCloud2 message fields
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgb', 12, PointField.FLOAT32, 1)
        ]

        # Create PointCloud2 message
        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "camera_link"

        cloud_msg = pc2.create_cloud(header, fields, cloud_data)
        
        self.point_cloud_pub.publish(cloud_msg)
        rospy.loginfo("Published point cloud.")

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
