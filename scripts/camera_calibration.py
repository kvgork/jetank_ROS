#!/usr/bin/env python3
import traitlets
from traitlets.config.configurable import SingletonConfigurable
import cv2
import os
import numpy as np
import glob
import keyboard

class CameraCalibration(SingletonConfigurable):

    # Camera properties
    width = traitlets.Integer(default_value=1920).tag(config=True)   # Reduce for performance
    height = traitlets.Integer(default_value=1080).tag(config=True)
    fps = traitlets.Integer(default_value=5).tag(config=True)      # Lower FPS if needed
    left_sensor_id = traitlets.Integer(default_value=0).tag(config=True)
    right_sensor_id = traitlets.Integer(default_value=1).tag(config=True)

    def __init__(self, checkerboard_x, checkerboard_y, *args, **kwargs):
        super(CameraCalibration, self).__init__(*args, **kwargs)
        self.CHECKERBOARD= [checkerboard_x , checkerboard_y]
        # # Initialize ROS node
        # rospy.init_node('camera_calibration', anonymous=True)

        # self.bridge = CvBridge()
        # self.rate = rospy.Rate(10)  # 10 Hz
    
    def take_pictures(self):
        # Chessboard size (number of inner corners per row and column)
        image_dir = "calibration_images"
        os.makedirs(image_dir, exist_ok=True)

        # Initialize video capture for both cameras
        cap_left = cv2.VideoCapture(self._gst_str(sensor_id=self.left_sensor_id), cv2.CAP_GSTREAMER)  # Left camera index
        cap_right = cv2.VideoCapture(self._gst_str(sensor_id=self.right_sensor_id), cv2.CAP_GSTREAMER)  # Right camera index

        # Ensure cameras opened
        if not cap_left.isOpened() or not cap_right.isOpened():
            raise RuntimeError('Could not open cameras. Check camera connections.')

        print("Press 's' to save an image, press 'q' to continue. Try to get about 12 images.")
        frame_count = 0
        while True:
            retL, frameL = cap_left.read()
            retR, frameR = cap_right.read()

            if not (retL and retR):
                print("Failed to capture images.")
                break

            # Check if frames are valid
            if frameL is None or frameR is None:
                print("Error: Captured frames are None. Check the camera connection.")
                break

            print(f"Frames captured successfully: Left shape {frameL.shape}, Right shape {frameR.shape}")
		
            combined_frame = cv2.hconcat([frameL, frameR])
            cv2.imshow("Stereo camera", combined_frame)

            key = input("Press 's' to save, 'q' to quit: ")
            if key == 's':  # Press 's' to save images
                left_filename = f"{image_dir}/left_{frame_count}.png"
                right_filename = f"{image_dir}/right_{frame_count}.png"

                try:
                    cv2.imwrite(left_filename, frameL)
                    cv2.imwrite(right_filename, frameR)
                    print(f"Saved pair {frame_count}: {left_filename}, {right_filename}")
                except Exception as e:
                    print(f"Error saving images: {e}")
                frame_count += 1
            elif key == 'q':  # Press 'q' to continue
                break

        cap_left.release()
        cap_right.release()
        cv2.destroyAllWindows()

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
    
    def _gst_str(self, sensor_id):
        """GStreamer pipeline for Jetson Nano"""
        return (
            "nvarguscamerasrc sensor-id={} ! video/x-raw(memory:NVMM), width={}, height={}, "
            "format=(string)NV12, framerate={}/1 ! nvvidconv ! video/x-raw, "
            "width={}, height={}, format=(string)BGRx ! videoconvert ! appsink"
            .format(sensor_id, self.width, self.height, self.fps, self.width, self.height)
        )

if __name__ == '__main__':
    calib_camera = CameraCalibration(9,7)
    calib_camera.take_pictures()
    # calib_camera.find_corners()
    # try:
    #     calib_camera.find_corners()
    # except:
    #     print("Calibration failed")

