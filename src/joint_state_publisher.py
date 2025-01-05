#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from SCSCtrl.scservo_sdk import *  # Feetech SDK
import time

def publish_joint_states():
    rospy.init_node('joint_state_publisher')
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rate = rospy.Rate(10)

    portHandler = PortHandler('/dev/ttyUSB0')
    packetHandler = PacketHandler(1.0)

    portHandler.openPort()
    portHandler.setBaudRate(1000000)

    while not rospy.is_shutdown():
        joint_msg = JointState()
        joint_msg.header.stamp = rospy.Time.now()
        joint_msg.name = ['chassis_to_arm_bearing_joint', 'arm_base_to_camera_joint', 'arm_base_to_long_joint', 'arm_long_to_short_joint']
        
        positions = []
        for servo_id in [1, 2, 3, 4, 5]:
            pos, result, error = packetHandler.read2ByteTxRx(portHandler, servo_id, 0x2A)
            if result == COMM_SUCCESS:
                positions.append(pos * 0.00153 - 3.14)  # Convert to radians
            else:
                positions.append(0)

        joint_msg.position = positions
        pub.publish(joint_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_joint_states()
    except rospy.ROSInterruptException:
        pass
