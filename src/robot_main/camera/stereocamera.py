import cv2
from sensor_msgs.msg import Image, PointCloud2, PointField
from camera import s


import numpy as np
import sensor_msgs.point_cloud2 as pc2
import rospy

class StereoCamera():

    def __init__(self):
        # Left camera settings

        settings = CameraSettings()
        settings.sensor_id = 0
        settings.sensor_side = 'left'

        self.camera_left = SingleCamera(settings)

        # Right camera settings
        settings = CameraSettings()
        settings.sensor_id = 1
        settings.sensor_side = 'right'

        self.camera_right = SingleCamera(settings)
        
        # Initialize ROS node
        rospy.init_node('stereo_camera', anonymous=True)
        self.point_cloud_pub = rospy.Publisher('/camera/depth/points', PointCloud2, queue_size=10)

    
    def start():

    def stop():

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
    