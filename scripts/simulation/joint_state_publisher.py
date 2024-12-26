#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import time

class JointStatePublisher:
    def __init__(self):
        rospy.init_node('joint_state_publisher', anonymous=True)
        self.joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)

        self.rate = rospy.Rate(10)  # 10 Hz

        self.wheel_base = 0.5  # Distance between wheels
        self.wheel_radius = 0.1  # Radius of the wheels

        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0

    def cmd_vel_callback(self, cmd_vel):
        left_wheel_vel = (cmd_vel.linear.x - cmd_vel.angular.z * self.wheel_base / 2) / self.wheel_radius
        right_wheel_vel = (cmd_vel.linear.x + cmd_vel.angular.z * self.wheel_base / 2) / self.wheel_radius

        dt = 1.0 / self.rate.sleep_dur.to_sec()
        self.left_wheel_pos += left_wheel_vel * dt
        self.right_wheel_pos += right_wheel_vel * dt

    def run(self):
        while not rospy.is_shutdown():
            joint_state = JointState()
            joint_state.header.stamp = rospy.Time.now()
            joint_state.name = ['left_wheel_joint', 'right_wheel_joint']
            joint_state.position = [self.left_wheel_pos, self.right_wheel_pos]

            self.joint_pub.publish(joint_state)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        joint_state_publisher = JointStatePublisher()
        joint_state_publisher.run()
    except rospy.ROSInterruptException:
        pass
