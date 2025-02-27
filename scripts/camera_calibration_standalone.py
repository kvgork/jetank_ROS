#!/usr/bin/env python3
import traitlets
from traitlets.config.configurable import SingletonConfigurable
import atexit
import cv2
import numpy as np
import rospy
import threading

class CameraCalibrationStandalone(SingletonConfigurable):
    
    # Camera properties
    width = traitlets.Integer(default_value=1280).tag(config=True)   # Reduce for performance
    height = traitlets.Integer(default_value=800).tag(config=True)
    fps = traitlets.Integer(default_value=20).tag(config=True)      # Lower FPS if needed
    left_sensor_id = traitlets.Integer(default_value=0).tag(config=True)
    right_sensor_id = traitlets.Integer(default_value=1).tag(config=True)

    def __init__(self, *args, **kwargs):
        super(CameraCalibrationStandalone, self).__init__(*args, **kwargs)

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

        # self.point_cloud_thread = threading.Thread(target=self.generate_point_cloud)

        self.left_thread.start()
        self.right_thread.start()

        self.take_pictures()


    def capture_left(self):
        while not rospy.is_shutdown():
            left_re, left_image = self.left_cap.read()
            if left_re:
                self.left_image_raw = cv2.flip(left_image, -1)
            else:
                rospy.logwarn("Failed to capture left frames.")

    def capture_right(self):
        while not rospy.is_shutdown():
            right_re, right_image = self.right_cap.read()
            if right_re:
                self.right_image_raw = cv2.flip(right_image, -1)
            else:
                rospy.logwarn("Failed to capture left frames.")

    def stop(self):
        self.left_cap.release()
        self.right_cap.release()

    def take_pictures(self):
        frame_count = 0
        while True:
            try:
                left_img = self.left_image_raw
                right_img = self.right_image_raw

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

                # rospy.loginfo("Published point cloud.")

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


if __name__ == '__main__':
    camera = CameraCalibrationStandalone()
    try:
        camera.start()
    except rospy.ROSInterruptException:
        pass
