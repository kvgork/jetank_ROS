#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

def callback(data):
    # Convert the linear and angular velocities into motor velocities
    left_wheel_vel = data.linear.x - data.angular.z
    right_wheel_vel = data.linear.x + data.angular.z

    rospy.loginfo("Left wheel velocity: %f", left_wheel_vel)
    rospy.loginfo("Right wheel velocity: %f", right_wheel_vel)

    pub_left.publish(left_wheel_vel)
    pub_right.publish(right_wheel_vel)

def listener():
    rospy.init_node('cmd_vel_listener')

    rospy.Subscriber("/cmd_vel", Twist, callback)

    global pub_left, pub_right
    pub_left = rospy.Publisher('/left_wheel_velocity_controller/command', Float64, queue_size=10)
    pub_right = rospy.Publisher('/right_wheel_velocity_controller/command', Float64, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    listener()
