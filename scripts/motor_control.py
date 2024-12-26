#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

def left_wheel_callback(data):
    rospy.loginfo("Left wheel command received: %f", data.data)
    # Send command to the left wheel motor controller
    # e.g., left_motor.set_speed(data.data)

def right_wheel_callback(data):
    rospy.loginfo("Right wheel command received: %f", data.data)
    # Send command to the right wheel motor controller
    # e.g., right_motor.set_speed(data.data)

def motor_control():
    rospy.init_node('motor_control')

    rospy.Subscriber("/left_wheel_velocity_controller/command", Float64, left_wheel_callback)
    rospy.Subscriber("/right_wheel_velocity_controller/command", Float64, right_wheel_callback)

    rospy.spin()

if __name__ == '__main__':
    motor_control()
