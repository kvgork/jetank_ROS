import rospy
import cv2
import numpy as np

from cv_bridge import CvBridge
from robot_main.camera.camera import StereoCamera
from sensor_msgs.msg import Image, PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from message_filters import ApproximateTimeSynchronizer, Subscriber

class CameraCalibration:

    def __init__(self):
        rospy.init_node("stereo_pointcloud_node")
        Stereo_camera = StereoCamera()
        StereoCamera.start()

        self.bridge = CvBridge()

        self.left_image_sub = Subscriber('/camera/left/image_raw', Image)
        self.right_image_sub = Subscriber('/camera/right/image_raw', Image)

        # Synchronize the image messages
        self.sync = ApproximateTimeSynchronizer([self.left_sub, self.right_sub], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.image_callback)

        # Point Cloud Publisher
        self.pc_pub = rospy.Publisher("/camera/point_cloud", PointCloud2, queue_size=10)

        # Stereo Matcher (CUDA-accelerated StereoBM)
        self.stereo = cv2.cuda.createStereoBM(numDisparities=64, blockSize=15)

        # Camera Calibration Parameters (Modify for your camera)
        self.focal_length = 400  # Pixels
        self.baseline = 0.1  # Meters
        self.cx = 224 / 2  # Optical center x
        self.cy = 224 / 2  # Optical center y

    def image_callback(self, left_msg, right_msg):
        try:
            # Convert ROS Image messages to OpenCV format
            left_img = self.bridge.imgmsg_to_cv2(left_msg, "bgr8")
            right_img = self.bridge.imgmsg_to_cv2(right_msg, "bgr8")

            # Convert to grayscale
            left_gray = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)
            right_gray = cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY)

            # Upload images to GPU
            left_gpu = cv2.cuda_GpuMat()
            right_gpu = cv2.cuda_GpuMat()
            left_gpu.upload(left_gray)
            right_gpu.upload(right_gray)

            # Compute disparity map using CUDA StereoBM
            disparity_gpu = self.stereo.compute(left_gpu, right_gpu)
            disparity = disparity_gpu.download().astype(np.float32) / 16.0

            # Convert disparity map to depth map
            depth_map = self.disparity_to_depth(disparity)

            # Generate point cloud
            cloud_msg = self.create_point_cloud(left_img, depth_map)

            # Publish the point cloud
            self.pc_pub.publish(cloud_msg)

            rospy.loginfo("Published point cloud.")

        except Exception as e:
            rospy.logerr(f"Error processing stereo images: {e}")

    def disparity_to_depth(self, disparity):
        """ Converts disparity map to depth map """
        depth = np.zeros(disparity.shape, dtype=np.float32)
        valid_disp = disparity > 0  # Avoid division by zero
        depth[valid_disp] = (self.focal_length * self.baseline) / disparity[valid_disp]
        return depth

    def create_point_cloud(self, image, depth_map):
        """ Converts depth map and image into a PointCloud2 message """
        height, width = depth_map.shape
        points = []

        for v in range(height):
            for u in range(width):
                Z = depth_map[v, u]
                if Z > 0 and Z < 10:  # Ignore invalid depth values
                    X = (u - self.cx) * Z / self.focal_length
                    Y = (v - self.cy) * Z / self.focal_length
                    B, G, R = image[v, u]
                    points.append([X, Y, Z, (R << 16) | (G << 8) | B])

        # Define PointCloud2 fields
        fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
            PointField("rgb", 12, PointField.UINT32, 1),
        ]

        # Create the PointCloud2 message
        cloud_msg = pc2.create_cloud(PointCloud2(), fields, points)
        cloud_msg.header.stamp = rospy.Time.now()
        cloud_msg.header.frame_id = "camera_link"

        return cloud_msg

if __name__ == "__main__":
    try:
        StereoPointCloudNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass