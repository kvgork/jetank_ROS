#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from SCSCtrl.scservo_sdk import *  # Feetech SDK
import time

def publish_joint_states():
    
    rospy.init_node('joint_state_publisher')
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        joint_msg = JointState()
        joint_msg.header.stamp = rospy.Time.now()
        joint_msg.name = list(self.servo_ids.keys())
        joint_msg.position = []

        for servo_id in self.servo_ids.values():
            pos = self.read_servo_position(self.port, servo_id)
            joint_msg.position.append(pos)

        pub.publish(joint_msg)
        rate.sleep()
    
    self.port.close()

if __name__ == '__main__':
    try:
        publish_joint_states()
    except rospy.ROSInterruptException:
        pass
