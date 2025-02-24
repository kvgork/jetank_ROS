#!/usr/bin/env python3
# Based on jetbot
import time
import traitlets
from traitlets.config.configurable import SingletonConfigurable
from Adafruit_MotorHAT import Adafruit_MotorHAT
from motor import Motor
from geometry_msgs.msg import Twist
import rospy


class Robot(SingletonConfigurable):
    left_motor = traitlets.Instance(Motor)
    right_motor = traitlets.Instance(Motor)

    # config
    i2c_bus = traitlets.Integer(default_value=1).tag(config=True)
    left_motor_channel = traitlets.Integer(default_value=1).tag(config=True)
    left_motor_alpha = traitlets.Float(default_value=1.0).tag(config=True)
    right_motor_channel = traitlets.Integer(default_value=2).tag(config=True)
    right_motor_alpha = traitlets.Float(default_value=1.0).tag(config=True)

    def __init__(self, *args, **kwargs):
        super(Robot, self).__init__(*args, **kwargs)
        self.motor_driver = Adafruit_MotorHAT(i2c_bus=self.i2c_bus)
        self.left_motor = Motor(self.motor_driver, channel=self.left_motor_channel, alpha=self.left_motor_alpha)
        self.right_motor = Motor(self.motor_driver, channel=self.right_motor_channel, alpha=self.right_motor_alpha)

        ## TODO get from urfd
        self.track_width = 0.11

        # Stop motors for safety
        self.stop()

        # setup ros node
        rospy.init_node('robot_motors')
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)

    def cmd_vel_callback(self, msg):
        lin_vel = msg.linear.x
        ang_vel = msg.angular.z

        self.left_motor.value = lin_vel - (ang_vel * self.track_width / 2)
        self.right_motor.value = lin_vel + (ang_vel * self.track_width / 2)


    def stop(self):
        self.left_motor.value = 0
        self.right_motor.value = 0

if __name__ == '__main__':
    try:
        robot = Robot()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass