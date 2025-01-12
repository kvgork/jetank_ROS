#!/usr/bin/env python
import serial
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import time
import struct


class SCS15_controller():
    def __init__(self):
        # Servo IDs
        self.servo_ids = {
            'arm_base_to_long_joint': 1,
            'arm_long_to_short_joint': 2,
            'chassis_to_arm_bearing_joint': 3,
            'left_finger_joint': 4,
            'arm_base_to_camera_joint': 5
        }
        self.port = serial.Serial('/dev/ttyUSB0', baudrate=1000000, timeout=0.1)

    def calculate_checksum(packet):
        return (~sum(packet[2:]) & 0xFF)

    def write_servo_position(self, servo_id, position):
        # Convert radians to servo value (0-1023 for 300 degrees)
        value = int((position + 3.14) * (1023 / 6.28))
        command = struct.pack('<BBBBBB', 0xFF, 0xFF, servo_id, 5, 3, value & 0xFF) + struct.pack('<B', (value >> 8) & 0xFF)
        checksum = (~sum(command[2:]) & 0xFF)
        self.port.write(command + struct.pack('<B', checksum))

    def read_servo_position(self, ser, servo_id):
        # Build Read Position Packet
        command = [0xFF, 0xFF, servo_id, 0x04, 0x02, 0x2A, 0x02]
        checksum = self.calculate_checksum(command)
        command.append(checksum)

        # Send Command
        ser.write(bytearray(command))

        # Read Response
        response = ser.read(8)
        if len(response) < 8:
            rospy.logwarn(f"No response from servo {servo_id}")
            return None

        # Parse Response
        _, _, id, length, error, pos_low, pos_high, _ = struct.unpack('<BBBBBBB', response)
        position = (pos_high << 8) + pos_low
        return position * (3.14159 / 1023.0) - 3.14159  # Scale to radians

    def handle_joint_command(self, msg):
        for i, name in enumerate(msg.joint_names):
            position = msg.points[0].positions[i]
            if name in self.servo_ids:
                self.write_servo_position(self.servo_ids[name], position)

    def publish_joint_states(self):
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
    rospy.init_node('scs15_controller')
    rospy.Subscriber('/arm_controller/command', JointTrajectory, handle_joint_command)
    publish_joint_states()
