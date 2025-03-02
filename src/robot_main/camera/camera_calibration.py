#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from std_srvs.srv import SetBool, SetBoolResponse
import os
import yaml
from datetime import datetime

class StereoCameraCalibrator:
    def __init__(self):
        # Initialize node
        rospy.init_node('stereo_camera_calibrator', anonymous=True)
        
        # Parameters
        self.chessboard_size = rospy.get_param('~chessboard_size', (8, 6))  # Interior points on chessboard
        self.square_size = rospy.get_param('~square_size', 0.025)  # Size of chessboard square in meters
        self.num_samples = rospy.get_param('~num_samples', 20)  # Number of sample pairs to collect
        self.save_path = rospy.get_param('~save_path', 'calibration_images')
        self.left_camera_name = rospy.get_param('~left_camera_name', 'left_camera')
        self.right_camera_name = rospy.get_param('~right_camera_name', 'right_camera')
        
        # Create directory if it doesn't exist
        if not os.path.exists(self.save_path):
            os.makedirs(self.save_path, exist_ok=True)
        
        # Initialize calibration data structures
        self.image_points_left = []  # 2D points in left image
        self.image_points_right = []  # 2D points in right image
        self.object_points = []  # 3D points in real world space
        
        # Prepare object points (0,0,0), (1,0,0), (2,0,0) ... (8,5,0)
        self.objp = np.zeros((self.chessboard_size[0] * self.chessboard_size[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:self.chessboard_size[0], 0:self.chessboard_size[1]].T.reshape(-1, 2)
        self.objp *= self.square_size  # Scale to actual size
        
        # Subscriber topics
        left_image_topic = rospy.get_param('~left_image_topic', '/camera/left/image_raw')
        right_image_topic = rospy.get_param('~right_image_topic', '/camera/right/image_raw')
        
        # Create subscribers with approximate time synchronization
        self.left_sub = message_filters.Subscriber(left_image_topic, Image)
        self.right_sub = message_filters.Subscriber(right_image_topic, Image)
        
        # Initialize time synchronizer
        ts = message_filters.ApproximateTimeSynchronizer(
            [self.left_sub, self.right_sub], queue_size=10, slop=0.1
        )
        ts.registerCallback(self.image_callback)
        
        # Publisher for calibration status
        self.status_pub = rospy.Publisher('/stereo_calibration/status', String, queue_size=10)
        
        # Publisher for preview images (optional)
        self.preview_pub = rospy.Publisher('/stereo_calibration/preview', Image, queue_size=10)
        
        # Service to trigger calibration
        rospy.Service('/stereo_calibration/trigger', SetBool, self.trigger_calibration)
        
        # Calibration state
        self.collecting_samples = False
        self.sample_count = 0
        
        rospy.loginfo("Stereo camera calibrator initialized")
        self.status_pub.publish("Calibrator ready. Waiting for trigger.")
    
    def image_callback(self, left_msg, right_msg):
        """Process synchronized stereo image pair"""
        if not self.collecting_samples:
            return
        
        try:
            # Manual conversion from ROS Image message to OpenCV format (avoiding CvBridge)
            # For left image
            # rospy.loginfo("Camera callback activated")
            if left_msg.encoding == 'rgb8':
                left_img = np.frombuffer(left_msg.data, dtype=np.uint8).reshape(
                    left_msg.height, left_msg.width, 3
                )
                left_img = cv2.cvtColor(left_img, cv2.COLOR_RGB2BGR)
            elif left_msg.encoding == 'bgr8':
                left_img = np.frombuffer(left_msg.data, dtype=np.uint8).reshape(
                    left_msg.height, left_msg.width, 3
                )
            elif left_msg.encoding == 'mono8':
                left_img = np.frombuffer(left_msg.data, dtype=np.uint8).reshape(
                    left_msg.height, left_msg.width
                )
                left_img = cv2.cvtColor(left_img, cv2.COLOR_GRAY2BGR)
            else:
                rospy.logerr(f"Unsupported encoding: {left_msg.encoding}.")
                return
            
            # For right image
            if right_msg.encoding == 'rgb8':
                right_img = np.frombuffer(right_msg.data, dtype=np.uint8).reshape(
                    right_msg.height, right_msg.width, 3
                )
                right_img = cv2.cvtColor(right_img, cv2.COLOR_RGB2BGR)
            elif right_msg.encoding == 'bgr8':
                right_img = np.frombuffer(right_msg.data, dtype=np.uint8).reshape(
                    right_msg.height, right_msg.width, 3
                )
            elif right_msg.encoding == 'mono8':
                right_img = np.frombuffer(right_msg.data, dtype=np.uint8).reshape(
                    right_msg.height, right_msg.width
                )
                right_img = cv2.cvtColor(right_img, cv2.COLOR_GRAY2BGR)
            else:
                rospy.logerr(f"Unsupported encoding: {right_msg.encoding}.")
                return
            
            # Convert to grayscale
            left_gray = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)
            right_gray = cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY)
            
            # Find chessboard corners
            left_found, left_corners = cv2.findChessboardCornersSB(
                left_gray, self.chessboard_size, flags=cv2.CALIB_CB_EXHAUSTIVE
            )
            right_found, right_corners = cv2.findChessboardCornersSB(
                right_gray, self.chessboard_size, flags=cv2.CALIB_CB_EXHAUSTIVE
            )
            
            # If corners found in both images
            if left_found and right_found:
                # Refine corner positions
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                left_corners_refined = cv2.cornerSubPix(
                    left_gray, left_corners, (11, 11), (-1, -1), criteria
                )
                right_corners_refined = cv2.cornerSubPix(
                    right_gray, right_corners, (11, 11), (-1, -1), criteria
                )
                
                # # Draw the corners on the images for visualization
                # left_display = left_img.copy()
                # right_display = right_img.copy()
                # cv2.drawChessboardCorners(
                #     left_display, self.chessboard_size, left_corners_refined, left_found
                # )
                # cv2.drawChessboardCorners(
                #     right_display, self.chessboard_size, right_corners_refined, right_found
                # )
                
                # # Create a side-by-side image for preview
                # preview = np.hstack((left_display, right_display))
                # preview_resized = cv2.resize(preview, (0, 0), fx=0.5, fy=0.5)
                
                # # Convert preview back to ROS Image message
                # preview_msg = Image()
                # preview_msg.header.stamp = rospy.Time.now()
                # preview_msg.height = preview_resized.shape[0]
                # preview_msg.width = preview_resized.shape[1]
                # preview_msg.encoding = "bgr8"
                # preview_msg.is_bigendian = 0
                # preview_msg.step = 3 * preview_resized.shape[1]
                # preview_msg.data = preview_resized.tobytes()
                # self.preview_pub.publish(preview_msg)
                
                # Save the points
                self.image_points_left.append(left_corners_refined)
                self.image_points_right.append(right_corners_refined)
                self.object_points.append(self.objp)
                
                # Increment sample count
                self.sample_count += 1
                self.status_pub.publish(f"Collected {self.sample_count}/{self.num_samples} image pairs")
                rospy.loginfo(f"Collected {self.sample_count}/{self.num_samples} image pairs")
                
                # Save the images for reference (optional)
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                left_filename = os.path.join(self.save_path, f"left_{timestamp}.png")
                right_filename = os.path.join(self.save_path, f"right_{timestamp}.png")
                cv2.imwrite(left_filename, left_img)
                cv2.imwrite(right_filename, right_img)
                
                # Check if we have enough samples
                if self.sample_count >= self.num_samples:
                    self.collecting_samples = False
                    self.status_pub.publish("All samples collected. Running calibration...")
                    rospy.loginfo("All calibration samples collected. Performing calibration...")
                    self.perform_calibration(left_gray.shape[::-1])
            else:
                # Report when chessboard not found
                if not left_found and not right_found:
                    status = "Chessboard not detected in either image"
                elif not left_found:
                    status = "Chessboard not detected in left image"
                else:
                    status = "Chessboard not detected in right image"
                self.status_pub.publish(status)
                rospy.loginfo(status)
                
        except Exception as e:
            rospy.logerr(f"Error processing images: {e}")
    
    def perform_calibration(self, image_size):
        """Run the stereo calibration algorithm"""
        try:
            # Calibrate each camera individually first
            ret_left, mtx_left, dist_left, rvecs_left, tvecs_left = cv2.calibrateCamera(
                self.object_points, self.image_points_left, image_size, None, None
            )
            ret_right, mtx_right, dist_right, rvecs_right, tvecs_right = cv2.calibrateCamera(
                self.object_points, self.image_points_right, image_size, None, None
            )
            
            # Stereo calibration
            flags = 0
            flags |= cv2.CALIB_FIX_INTRINSIC
            
            ret, mtx_left, dist_left, mtx_right, dist_right, R, T, E, F = cv2.stereoCalibrate(
                self.object_points, 
                self.image_points_left, 
                self.image_points_right, 
                mtx_left, 
                dist_left, 
                mtx_right, 
                dist_right, 
                image_size, 
                flags=flags
            )
            
            # Stereo rectification
            R1, R2, P1, P2, Q, roi_left, roi_right = cv2.stereoRectify(
                mtx_left, dist_left, mtx_right, dist_right, image_size, R, T
            )
            
            # Save calibration results
            calibration_data = {
                'left_camera_matrix': mtx_left.tolist(),
                'left_distortion_coefficients': dist_left.tolist(),
                'right_camera_matrix': mtx_right.tolist(),
                'right_distortion_coefficients': dist_right.tolist(),
                'rotation_matrix': R.tolist(),
                'translation_vector': T.tolist(),
                'essential_matrix': E.tolist(),
                'fundamental_matrix': F.tolist(),
                'left_rectification_matrix': R1.tolist(),
                'right_rectification_matrix': R2.tolist(),
                'left_projection_matrix': P1.tolist(),
                'right_projection_matrix': P2.tolist(),
                'disparity_to_depth_mapping': Q.tolist(),
                'left_roi': roi_left,
                'right_roi': roi_right,
                'image_size': list(image_size),
                'calibration_error': float(ret),
                'calibration_date': datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                'left_camera_name': self.left_camera_name,
                'right_camera_name': self.right_camera_name,
                'chessboard_size': self.chessboard_size,
                'square_size': self.square_size,
                'num_image_pairs': len(self.object_points)
            }
            
            # Save to YAML file
            calibration_file = os.path.join(self.save_path, 'stereo_calibration.yaml')
            with open(calibration_file, 'w') as f:
                yaml.dump(calibration_data, f)
            
            # Create camera info messages
            left_cam_info = self.generate_camera_info(
                self.left_camera_name, image_size, mtx_left, dist_left, R1, P1
            )
            right_cam_info = self.generate_camera_info(
                self.right_camera_name, image_size, mtx_right, dist_right, R2, P2
            )
            
            # Save camera info to separate files
            left_info_file = os.path.join(self.save_path, f'{self.left_camera_name}.yaml')
            right_info_file = os.path.join(self.save_path, f'{self.right_camera_name}.yaml')
            
            with open(left_info_file, 'w') as f:
                yaml.dump(self.camera_info_to_dict(left_cam_info), f)
            
            with open(right_info_file, 'w') as f:
                yaml.dump(self.camera_info_to_dict(right_cam_info), f)
            
            # Report success
            calib_error_mm = ret * 1000  # convert to mm
            status_msg = f"Calibration complete! RMS error: {calib_error_mm:.2f} mm. Files saved to {self.save_path}"
            self.status_pub.publish(status_msg)
            rospy.loginfo(status_msg)
            
        except Exception as e:
            error_msg = f"Calibration failed: {str(e)}"
            self.status_pub.publish(error_msg)
            rospy.logerr(error_msg)
    
    def generate_camera_info(self, camera_name, image_size, K, D, R, P):
        """Generate camera info message from calibration data"""
        camera_info = CameraInfo()
        camera_info.header.frame_id = camera_name
        camera_info.height = image_size[1]
        camera_info.width = image_size[0]
        camera_info.distortion_model = "plumb_bob"
        camera_info.K = K.flatten().tolist()
        camera_info.D = D.flatten().tolist()
        camera_info.R = R.flatten().tolist()
        camera_info.P = P.flatten().tolist()
        return camera_info
    
    def camera_info_to_dict(self, camera_info):
        """Convert CameraInfo message to dictionary for YAML serialization"""
        return {
            'camera_name': camera_info.header.frame_id,
            'image_width': camera_info.width,
            'image_height': camera_info.height,
            'camera_matrix': {
                'rows': 3,
                'cols': 3,
                'data': list(camera_info.K)
            },
            'distortion_model': camera_info.distortion_model,
            'distortion_coefficients': {
                'rows': 1,
                'cols': len(camera_info.D),
                'data': list(camera_info.D)
            },
            'rectification_matrix': {
                'rows': 3,
                'cols': 3,
                'data': list(camera_info.R)
            },
            'projection_matrix': {
                'rows': 3,
                'cols': 4,
                'data': list(camera_info.P)
            }
        }
    
    def trigger_calibration(self, req):
        """Service callback to start/stop calibration collection using standard SetBool service"""
        try:
            if req.data:
                # Start calibration
                self.collecting_samples = True
                self.sample_count = 0
                self.image_points_left = []
                self.image_points_right = []
                self.object_points = []
                rospy.loginfo("Starting stereo camera calibration collection")
                self.status_pub.publish("Calibration started. Collecting image pairs...")
                return SetBoolResponse(success=True, message="Calibration started successfully")
            else:
                # Stop calibration
                self.collecting_samples = False
                rospy.loginfo("Stopping stereo camera calibration collection")
                self.status_pub.publish("Calibration collection stopped.")
                return SetBoolResponse(success=True, message="Calibration stopped successfully")
        except Exception as e:
            error_msg = f"Error in calibration trigger service: {str(e)}"
            rospy.logerr(error_msg)
            return SetBoolResponse(success=False, message=error_msg)


if __name__ == '__main__':
    try:
        calibrator = StereoCameraCalibrator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass