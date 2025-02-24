import traitlets
import rospy
import cv2

from traitlets.config.configurable import SingletonConfigurable
from sensor_msgs.msg import Image



class CameraSettings:
    # Camera properties
    width = traitlets.Integer(default_value=640).tag(config=True)   # Reduce for performance
    height = traitlets.Integer(default_value=480).tag(config=True)
    fps = traitlets.Integer(default_value=60).tag(config=True)      # Lower FPS if needed
    sensor_id = traitlets.Integer(default_value=0).tag(config=True)
    sensor_side: str | None
    node_name: str | None
    publisher_raw: rospy.Publisher | None




class SingleCamera(SingletonConfigurable):

    def __init__(self, camera_settings: CameraSettings):
        super(SingleCamera, self).__init__()
        self.settings = camera_settings
        # self.rate = rospy.Rate(100)  # 10 Hz
    
    def node_init(self):
        # Initialize ROS node and publishers
        if self.settings.sensor_id == None:
            self.settings.node_name = 'camera'
        else:
            self.settings.node_name = f'camera_{self.settings.sensor_side}'
        
        rospy.init_node(self.settings.node_name, anonymous=True)

        self.publisher_raw = rospy.Publisher(f'{self.settings.node_name}/image_raw', Image, queue_size=10)

    def start(self):

        self.node_init()
        self.open()

        while not rospy.is_shutdown():
            re, image = self.cap.read()

            if re:
                self.settings.publisher_raw(self.frame_to_ros_image(image, self.settings.sensor_side))
            else:
                rospy.logwarn("Failed to capture frames.")

            # self.rate.sleep()

    def stop(self):
        self.cap.release()

    def open(self):
        # Open cameras using GStreamer
        self.cap = cv2.VideoCapture(self.GStreamer_settings(), cv2.CAP_GSTREAMER)

        # Ensure cameras opened
        if not self.cap.isOpened():
            raise RuntimeError('Could not open camera. Check camera connection.')


    def GStreamer_settings(self):
        """GStreamer pipeline for Jetson Nano"""
        return (
            "nvarguscamerasrc sensor-id={} ! video/x-raw(memory:NVMM), width={}, height={}, "
            "format=(string)NV12, framerate={}/1 ! nvvidconv ! video/x-raw, "
            "width={}, height={}, format=(string)BGRx ! videoconvert ! appsink"
            .format(self.settings.sensor_id, self.settings.width, self.settings.height, self.settings.fps, self.settings.width, self.settings.height)
        )
    
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