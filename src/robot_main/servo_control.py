#!/usr/bin/env python3

import os
import rospy
import time
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from SCSCtrl.scservo_sdk import *  

class ServoControl:
    ADDR_SCS_TORQUE_ENABLE     = 40
    ADDR_SCS_PRESENT_POSITION  = 56
    ADDR_STS_GOAL_POSITION     = 42
    ADDR_STS_GOAL_SPEED        = 46
    
    BAUDRATE    = 1000000
    DEVICENAME  = '/dev/ttyTHS1'
    protocol_end = 1  

    def __init__(self):
        rospy.init_node('servo_controller', anonymous=True)
        
        # Servo IDs
        self.servo_ids = {
            'chassis_to_arm_bearing_joint': 1,
            'arm_base_to_long_joint': 2,
            'arm_long_to_short_joint': 3,
            'left_finger_joint': 4,
            'arm_base_to_camera_joint': 5
        }
        
        # ROS Topics
        self.pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.sub = rospy.Subscriber("/joint_trajectory", JointTrajectory, self.joint_callback)

        # Initialize Serial Communication
        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = PacketHandler(self.protocol_end)

        if not self.portHandler.openPort():
            rospy.logerr("Failed to open serial port")
            quit()

        if not self.portHandler.setBaudRate(self.BAUDRATE):
            rospy.logerr("Failed to set baud rate")
            self.portHandler.closePort()
            quit()

        rospy.loginfo("Servo Controller Initialized")

    def nowPosUpdate(self, servo_id):
        """Reads current servo position."""
        try:
            scs_present_position_speed, scs_comm_result, scs_error = self.packetHandler.read4ByteTxRx(
                self.portHandler, servo_id, self.ADDR_SCS_PRESENT_POSITION
            )
            if scs_comm_result != COMM_SUCCESS:
                rospy.logwarn(f"Communication failure with servo {servo_id}")
                return None
            if scs_error:
                rospy.logwarn(f"Servo {servo_id} returned error")
                return None
            return SCS_LOWORD(scs_present_position_speed)
        except Exception as e:
            rospy.logerr(f"Failed to read position from servo {servo_id}: {e}")
            return None

    def updateJointStates(self):
        """Publishes current joint states."""
        joint_msg = JointState()
        joint_msg.header.stamp = rospy.Time.now()
        joint_msg.name = list(self.servo_ids.keys())
        joint_msg.position = []

        for servo_name, servo_id in self.servo_ids.items():
            pos = self.nowPosUpdate(servo_id)
            if pos is not None:
                joint_msg.position.append(pos)
            else:
                joint_msg.position.append(0)  # Default to zero if there's an error

        self.pub.publish(joint_msg)

    def joint_callback(self, msg):
        """Receives trajectory commands and moves servos."""
        for i, servo_name in enumerate(msg.joint_names):
            if servo_name in self.servo_ids:
                servo_id = self.servo_ids[servo_name]
                goal_pos = int(msg.points[0].positions[i])  # Convert to integer

                # Send command to servo
                try:
                    self.packetHandler.write2ByteTxRx(
                        self.portHandler, servo_id, self.ADDR_STS_GOAL_POSITION, goal_pos
                    )
                except Exception as e:
                    rospy.logerr(f"Failed to move servo {servo_id}: {e}")

    def run(self):
        """Main loop to update joint states."""
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.updateJointStates()
            rate.sleep()

if __name__ == '__main__':
    controller = ServoControl()
    try:
        controller.run()
    except rospy.ROSInterruptException:
        pass
