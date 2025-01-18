#!/usr/bin/env python3
#script to tests functions

import rospy
from servo_controller import SCS15_controller

if __name__ == "__main__":
    controller = SCS15_controller()
    try:
        controller.publish_joint_states()
    except rospy.ROSInterruptException:
        pass