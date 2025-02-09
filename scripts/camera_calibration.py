import cv2
import os
import numpy as np
import glob

class CameraCalibration():
    def __init__(self, checkerboard_x, checkerboard_y):
        self.CHECKERBOARD= [checkerboard_x , checkerboard_y]
    
    def take_pictures(self):
        # Chessboard size (number of inner corners per row and column)
        CHECKERBOARD = (9, 6)  # Adjust based on your actual checkerboard
        image_dir = "calibration_images"
        os.makedirs(image_dir, exist_ok=True)

        # Initialize video capture for both cameras
        cap_left = cv2.VideoCapture(0)  # Left camera index
        cap_right = cv2.VideoCapture(1)  # Right camera index

        frame_count = 0
        while True:
            retL, frameL = cap_left.read()
            retR, frameR = cap_right.read()

            if not (retL and retR):
                print("Failed to capture images.")
                break

            cv2.imshow("Left Camera", frameL)
            cv2.imshow("Right Camera", frameR)

            key = cv2.waitKey(1)
            if key == ord("s"):  # Press 's' to save images
                cv2.imwrite(f"{image_dir}/left_{frame_count}.png", frameL)
                cv2.imwrite(f"{image_dir}/right_{frame_count}.png", frameR)
                print(f"Saved pair {frame_count}")
                frame_count += 1
            elif key == ord("q"):  # Press 'q' to quit
                break

        cap_left.release()
        cap_right.release()
        cv2.destroyAllWindows()

        self.find_corners()
    
    def find_corners(self):

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

            retL, cornersL = cv2.findChessboardCorners(self.imgL_gray, CHECKERBOARD, None)
            retR, cornersR = cv2.findChessboardCorners(self.imgR_gray, CHECKERBOARD, None)

            if retL and retR:
                self.objpoints.append(objp)

                cornersL = cv2.cornerSubPix(self.imgL_gray, cornersL, (11, 11), (-1, -1), criteria)
                cornersR = cv2.cornerSubPix(self.imgR_gray, cornersR, (11, 11), (-1, -1), criteria)

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

if __name__ == '__main__':
    calib_camera = CameraCalibration(6,6)
    try:
        calib_camera.take_pictures()
    except:
        print("Calibration failed")

