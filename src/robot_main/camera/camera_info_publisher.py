#!/usr/bin/env python3

import rospy
import yaml
from sensor_msgs.msg import CameraInfo
import os

class CameraInfoPublisher:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('camera_info_publisher', anonymous=True)
        
        # Get parameters
        self.camera_name = rospy.get_param('~camera_name', 'stereo')
        self.frame_id = rospy.get_param('~frame_id', 'camera_link')
        self.save_path = rospy.get_param('~save_path', 'calibration_images')
        self.left_yaml_file = self.save_path+'left_camera.yaml'
        self.right_yaml_file = self.save_path+'right_camera.yaml'
        
        self.publish_rate = rospy.get_param('~publish_rate', 10)  # Hz
        
        # Create publishers
        self.left_pub = rospy.Publisher('/stereo/left/camera_info', CameraInfo, queue_size=10)
        self.right_pub = rospy.Publisher('/stereo/right/camera_info', CameraInfo, queue_size=10)
        
        # Load camera info from YAML
        self.left_camera_info = self.load_camera_info(self.left_yaml_file)
        self.right_camera_info = self.load_camera_info(self.right_yaml_file)
        
        # Set up timer for publishing
        self.timer = rospy.Timer(rospy.Duration(1.0/self.publish_rate), self.publish_camera_info)
        
        rospy.loginfo(f"Stereo camera info publisher started, for camera: {self.camera_name}")
        
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
    
    def dict_to_camera_info(self, camera_info_dict):
        """Convert dictionary to CameraInfo message"""
        camera_info = CameraInfo()
        
        # Set basic info
        camera_info.header.frame_id = self.frame_id
        camera_info.width = camera_info_dict['image_width']
        camera_info.height = camera_info_dict['image_height']
        camera_info.distortion_model = camera_info_dict['distortion_model']
        
        # Set camera matrices
        camera_info.K = camera_info_dict['camera_matrix']['data']
        camera_info.D = camera_info_dict['distortion_coefficients']['data']
        camera_info.R = camera_info_dict['rectification_matrix']['data']
        camera_info.P = camera_info_dict['projection_matrix']['data']
        
        return camera_info
    
    def load_camera_info(self, yaml_file):
        """Load camera info from YAML file"""
        if not os.path.isfile(yaml_file):
            rospy.logerr(f"Camera calibration file not found for file: {yaml_file}")
            return None
        
        with open(yaml_file, 'r') as file:
            try:
                camera_info_dict = yaml.safe_load(file)
                return self.dict_to_camera_info(camera_info_dict)
            except yaml.YAMLError as e:
                rospy.logerr(f"Error parsing YAML file: {e}")
                return None
    
    def save_camera_info(self, camera_info, file_path):
        """Save camera info to YAML file"""
        camera_info_dict = self.camera_info_to_dict(camera_info)
        with open(file_path, 'w') as file:
            yaml.dump(camera_info_dict, file)
            rospy.loginfo(f"Camera info saved to {file_path}")
    
    def publish_camera_info(self, event):
        """Publish camera info at regular intervals"""
        if self.camera_info is not None:
            # Update timestamp
            self.camera_info.header.stamp = rospy.Time.now()
            self.pub.publish(self.camera_info)
    
    def run(self):
        """Run the node"""
        rospy.spin()

if __name__ == '__main__':
    try:
        publisher = CameraInfoPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        pass