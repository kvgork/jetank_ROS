#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import os
import glob

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from message_filters import ApproximateTimeSynchronizer, Subscriber

class CameraCalibration:

    def __init__(self, checkerboard_x, checkerboard_y):
        self.CHECKERBOARD= [checkerboard_x , checkerboard_y]
        self.image_dir = "calibration_images"
        os.makedirs(self.image_dir, exist_ok=True)

        rospy.init_node("camera_calibration_node")

        self.bridge = CvBridge()

        self.left_image_sub = Subscriber('/camera/left/image_raw', Image)
        self.right_image_sub = Subscriber('/camera/right/image_raw', Image)

        # Synchronize the image messages
        self.sync = ApproximateTimeSynchronizer([self.left_image_sub, self.right_image_sub], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.image_callback)

    def image_callback(self, left_msg, right_msg):
        # Convert ROS Image messages to OpenCV format
        left_img = self.bridge.imgmsg_to_cv2(left_msg, "bgr8")
        right_img = self.bridge.imgmsg_to_cv2(right_msg, "bgr8")

        self.left_image = left_img
        self.right_image = right_img

    def take_pictures(self):
        while True:
            try:
                left_img = self.left_image
                right_img = self.right_image

                print(f"Frames captured successfully: Left shape {left_img.shape}, Right shape {right_img.shape}")
            
                combined_frame = cv2.hconcat([left_img, right_img])
                cv2.imshow("Stereo camera", combined_frame)

                key = input("Press 's' to save, ' ' to skip, 'q' to quit: ")
                if key == 's':  # Press 's' to save images
                    left_filename = f"{self.image_dir}/left_{frame_count}.png"
                    right_filename = f"{self.image_dir}/right_{frame_count}.png"

                    try:
                        cv2.imwrite(left_filename, left_img)
                        cv2.imwrite(right_filename, right_img)
                        print(f"Saved pair {frame_count}: {left_filename}, {right_filename}")
                    except Exception as e:
                        print(f"Error saving images: {e}")
                    frame_count += 1
                elif key == ' ':
                    pass
                
                elif key == 'q':  # Press 'q' to continue
                    break

                rospy.loginfo("Published point cloud.")

            except Exception as e:
                rospy.logerr(f"Error processing stereo images: {e}")
        self.find_corners()

    def find_corners(self):
        print("Starting corner detection")

        # Termination criteria for corner refinement
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # Prepare 3D points in real-world coordinates (assuming z=0)
        objp = np.zeros((self.CHECKERBOARD[0] * self.CHECKERBOARD[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.CHECKERBOARD[0], 0:self.CHECKERBOARD[1]].T.reshape(-1, 2)

        self.objpoints = []  # 3D world points
        self.imgpointsL = []  # 2D left image points
        self.imgpointsR = []  # 2D right image points

        images_left = sorted(glob.glob("calibration_images/left_*.png"))
        images_right = sorted(glob.glob("calibration_images/right_*.png"))

        for imgL, imgR in zip(images_left, images_right):
            self.imgL_gray = cv2.imread(imgL, cv2.IMREAD_GRAYSCALE)
            self.imgR_gray = cv2.imread(imgR, cv2.IMREAD_GRAYSCALE)

            retL, cornersL = cv2.findChessboardCorners(self.imgL_gray, (self.CHECKERBOARD[0], self.CHECKERBOARD[1]), None)
            retR, cornersR = cv2.findChessboardCorners(self.imgR_gray, (self.CHECKERBOARD[0], self.CHECKERBOARD[1]), None)

            if retL and retR:
                self.objpoints.append(objp)

                cornersL = cv2.cornerSubPix(self.imgL_gray, cornersL, (11, 11), (-1, -1), self.criteria)
                cornersR = cv2.cornerSubPix(self.imgR_gray, cornersR, (11, 11), (-1, -1), self.criteria)

                self.imgpointsL.append(cornersL)
                self.imgpointsR.append(cornersR)
            else:
                print(f"Skipping {imgL} and {imgR} due to detection failure.")
        self.calibration()


    def calibration(self):
        # Calibrate the individual cameras
        retL, mtxL, distL, rvecsL, tvecsL = cv2.calibrateCamera(self.objpoints, self.imgpointsL, self.imgL_gray.shape[::-1], None, None)
        retR, mtxR, distR, rvecsR, tvecsR = cv2.calibrateCamera(self.objpoints, self.imgpointsR, self.imgR_gray.shape[::-1], None, None)

        # Stereo calibration
        flags = cv2.CALIB_FIX_INTRINSIC  # Fix intrinsic parameters from individual calibration
        retS, _, _, _, _, R, T, E, F = cv2.stereoCalibrate(
            self.objpoints, self.imgpointsL, self.imgpointsR, mtxL, distL, mtxR, distR,
            self.imgL_gray.shape[::-1], criteria=self.criteria, flags=flags
        )

        print("Rotation Matrix (R):\n", R)
        print("Translation Vector (T):\n", T)

        # Stereo rectification
        R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
            mtxL, distL, mtxR, distR, self.imgL_gray.shape[::-1], R, T, alpha=0
        )

        print("Q Matrix:\n", Q)
        

if __name__ == "__main__":
    try:
        CameraCalibration()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass