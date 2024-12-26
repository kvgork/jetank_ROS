#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
Reading from the keyboard and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

anything else : stop

u/o : increase/decrease max speeds by 10%
j/l : increase/decrease only linear speed by 10%
m/. : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

move_bindings = {
    'i': (1, 0),
    'o': (1, -1),
    'j': (0, 1),
    'l': (0, -1),
    'u': (1, 1),
    ',': (-1, 0),
    '.': (-1, 1),
    'm': (-1, -1),
}

speed_bindings = {
    'u': (1.1, 1.1),
    'o': (1.1, 0.9),
    'j': (0.9, 1.1),
    'l': (0.9, 0.9),
    'm': (0.9, 0.9),
    ',': (1.1, 1.1),
    '.': (1.1, 0.9),
}

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_keyboard')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    speed = 0.5
    turn = 1.0
    x = 0
    th = 0
    status = 0

    try:
        print(msg)
        while(1):
            key = getKey()
            if key in move_bindings.keys():
                x = move_bindings[key][0]
                th = move_bindings[key][1]
            elif key in speed_bindings.keys():
                speed = speed * speed_bindings[key][0]
                turn = turn * speed_bindings[key][1]

                print("currently:\tspeed %s\tturn %s " % (speed, turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                x = 0
                th = 0
                if (key == '\x03'):
                    break

            twist = Twist()
            twist.linear.x = x * speed
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = th * turn
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)