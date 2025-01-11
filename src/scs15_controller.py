#!/usr/bin/env python
import serial
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import time
import struct

# Servo IDs
servo_ids = {
    'arm_base_to_long_joint': 1,
    'arm_long_to_short_joint': 2,
    'chassis_to_arm_bearing_joint': 3,
    'left_finger_joint': 4,
    'arm_base_to_camera_joint': 5
}

port = serial.Serial('/dev/ttyUSB0', baudrate=1000000, timeout=0.1)

def write_servo_position(servo_id, position):
    # Convert radians to servo value (0-1023 for 300 degrees)
    value = int((position + 3.14) * (1023 / 6.28))
    command = struct.pack('<BBBBBB', 0xFF, 0xFF, servo_id, 5, 3, value & 0xFF) + struct.pack('<B', (value >> 8) & 0xFF)
    checksum = (~sum(command[2:]) & 0xFF)
    port.write(command + struct.pack('<B', checksum))

def handle_joint_command(msg):
    for i, name in enumerate(msg.joint_names):
        position = msg.points[0].positions[i]
        if name in servo_ids:
            write_servo_position(servo_ids[name], position)

def publish_joint_states():
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        joint_msg = JointState()
        joint_msg.header.stamp = rospy.Time.now()
        joint_msg.name = list(servo_ids.keys())
        joint_msg.position = []

        for servo_id in servo_ids.values():
            # Read servo position (simulated by incrementing values)
            pos = (servo_id * 0.1) % 3.14
            joint_msg.position.append(pos)

        pub.publish(joint_msg)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('scs15_controller')
    rospy.Subscriber('/arm_controller/command', JointTrajectory, handle_joint_command)
    publish_joint_states()
