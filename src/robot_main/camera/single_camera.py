#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.srv import SetCameraInfo, SetCameraInfoResponse
from std_msgs.msg import Header
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst
import yaml

class JetsonCSICamera:
    def __init__(self):
        Gst.init(None)
        
        # ROS parameters
        self.camera_id = rospy.get_param('~camera_id', 0)
        self.width = rospy.get_param('~width', 640)
        self.height = rospy.get_param('~height', 480)
        self.fps = rospy.get_param('~fps', 30)
        self.camera_frame_id = rospy.get_param('~camera_frame_id', 'camera_frame')
        self.camera_info_url = rospy.get_param('~camera_info_url', '')
        
        # Load camera calibration
        self.camera_info = self.load_camera_info()

        # Set camera info service
        self.set_camera_info_service = rospy.Service('set_camera_info', 
                                            SetCameraInfo, 
                                            self.set_camera_info_callback)
        
        # Publisher setup
        self.image_pub = rospy.Publisher('image_raw', Image, queue_size=1)
        self.info_pub = rospy.Publisher('camera_info', CameraInfo, queue_size=1)
        
        # GStreamer pipeline for CSI camera
        self.setup_gstreamer_pipeline()
        
        # Start the camera
        self.pipeline.set_state(Gst.State.PLAYING)
        rospy.loginfo(f"Started CSI camera {self.camera_id}")
        
        # Timer for publishing images
        rospy.Timer(rospy.Duration(1.0/self.fps), self.publish_image)
    
    def load_camera_info(self):
        info = CameraInfo()
        info.header.frame_id = self.camera_frame_id
        info.width = self.width
        info.height = self.height
        
        # Try to load calibration from file
        if self.camera_info_url and self.camera_info_url.startswith('file://'):
            file_path = self.camera_info_url[7:]
            try:
                with open(file_path, 'r') as file:
                    calib_data = yaml.safe_load(file)
                    info.D = calib_data.get('distortion_coefficients', {}).get('data', [0.0, 0.0, 0.0, 0.0, 0.0])
                    info.K = calib_data.get('camera_matrix', {}).get('data', [self.width, 0, self.width/2, 0, self.height, self.height/2, 0, 0, 1])
                    info.R = calib_data.get('rectification_matrix', {}).get('data', [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
                    info.P = calib_data.get('projection_matrix', {}).get('data', 
                                        [self.width, 0, self.width/2, 0, 
                                        0, self.height, self.height/2, 0, 
                                        0, 0, 1, 0])
                    rospy.loginfo(f"Loaded calibration from {file_path}")
            except (IOError, yaml.YAMLError) as e:
                rospy.logwarn(f"Failed to load camera calibration: {e}")
                
                # Set default values if loading fails
                info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
                info.K = [self.width, 0, self.width/2, 0, self.height, self.height/2, 0, 0, 1]
                info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
                info.P = [self.width, 0, self.width/2, 0, 0, self.height, self.height/2, 0, 0, 0, 1, 0]
        else:
            # Default calibration
            rospy.logwarn("No camera_info_url provided, using default calibration")
            info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
            info.K = [self.width, 0, self.width/2, 0, self.height, self.height/2, 0, 0, 1]
            info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            info.P = [self.width, 0, self.width/2, 0, 0, self.height, self.height/2, 0, 0, 0, 1, 0]
        
        return info
    
    def setup_gstreamer_pipeline(self):
        # GStreamer pipeline for Jetson CSI camera
        pipeline_str = (
            f'nvarguscamerasrc sensor-id={self.camera_id} ! '
            f'video/x-raw(memory:NVMM), width={self.width}, height={self.height}, '
            f'format=NV12, framerate={self.fps}/1 ! '
            'nvvidconv ! video/x-raw, format=BGRx ! '
            'videoconvert ! video/x-raw, format=GRAY8 ! '
            'appsink name=sink max-buffers=1 drop=true'
        )
        
        self.pipeline = Gst.parse_launch(pipeline_str)
        self.sink = self.pipeline.get_by_name('sink')
        
        if not self.sink:
            rospy.logerr("Failed to create GStreamer pipeline")
            rospy.signal_shutdown("Failed to create GStreamer pipeline")
    
    def get_latest_frame(self):
        sample = self.sink.emit("pull-sample")
        if sample:
            buf = sample.get_buffer()
            caps = sample.get_caps()
            struct = caps.get_structure(0)
            
            width = struct.get_value("width")
            height = struct.get_value("height")
            
            result, mapinfo = buf.map(Gst.MapFlags.READ)
            if result:
                # Create numpy array from buffer
                img = np.ndarray(
                    shape=(height, width),
                    dtype=np.uint8,
                    buffer=mapinfo.data
                )
                buf.unmap(mapinfo)
                return img
            
        return None
    
    def publish_image(self, event):
        # Get frame
        frame = self.get_latest_frame()
        if frame is None:
            rospy.logwarn("Failed to get frame from camera")
            return
        
        # Flip frame
        frame = cv2.flip(frame, -1)

        # Create Image message
        img_msg = Image()
        img_msg.header.stamp = rospy.Time.now()
        img_msg.header.frame_id = self.camera_frame_id
        img_msg.height = frame.shape[0]
        img_msg.width = frame.shape[1]
        img_msg.encoding = "mono8"
        img_msg.step = img_msg.width
        img_msg.data = frame.tobytes()
        
        # Update CameraInfo timestamp
        self.camera_info.header.stamp = img_msg.header.stamp
        
        # Publish messages
        self.image_pub.publish(img_msg)
        self.info_pub.publish(self.camera_info)
    
    def set_camera_info_callback(self, req):
        """Callback for service to set the camera calibration."""
        
        # Update the camera_info
        self.camera_info = req.camera_info
        rospy.loginfo("Camera info updated")
        
        # If we have a camera_info_url parameter, save the info to that file
        if self.camera_info_url and self.camera_info_url.startswith('file://'):
            file_path = self.camera_info_url[7:]
            
            # Create the calibration data structure
            calibration = {
                'image_width': req.camera_info.width,
                'image_height': req.camera_info.height,
                'camera_name': self.camera_frame_id,
                'distortion_model': 'plumb_bob',
                'distortion_coefficients': {
                    'data': list(req.camera_info.D),
                    'rows': 1,
                    'cols': len(req.camera_info.D)
                },
                'camera_matrix': {
                    'data': list(req.camera_info.K),
                    'rows': 3,
                    'cols': 3
                },
                'rectification_matrix': {
                    'data': list(req.camera_info.R),
                    'rows': 3,
                    'cols': 3
                },
                'projection_matrix': {
                    'data': list(req.camera_info.P),
                    'rows': 3,
                    'cols': 4
                }
            }
            
            try:
                # Ensure directory exists
                os.makedirs(os.path.dirname(file_path), exist_ok=True)
                # Write to file
                with open(file_path, 'w') as f:
                    yaml.dump(calibration, f)
                rospy.loginfo(f"Camera calibration saved to {file_path}")
                return SetCameraInfoResponse(success=True, status_message=f"Saved to {file_path}")
            except Exception as e:
                error_msg = f"Failed to write camera_info to {file_path}: {str(e)}"
                rospy.logerr(error_msg)
                return SetCameraInfoResponse(success=False, status_message=error_msg)
        else:
            return SetCameraInfoResponse(
                success=False,
                status_message="No valid camera_info_url specified"
            )
        
    def shutdown(self):
        # Stop the pipeline
        self.pipeline.set_state(Gst.State.NULL)
        rospy.loginfo("Camera shutdown complete")

if __name__ == '__main__':
    rospy.init_node('jetson_csi_cam')
    camera = JetsonCSICamera()
    
    # Register shutdown hook
    rospy.on_shutdown(camera.shutdown)
    
    rospy.spin()
