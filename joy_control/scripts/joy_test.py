#!/usr/bin/env python

import rospy
import joy_control.leg_control
from sensor_msgs.msg import Joy
from logging import getLogger

logger = getLogger(__name__)
print("start")
class sub_joy(object):
    def __init__(self):
        self._joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback, queue_size=1)
    def joy_callback(self, joy_msg):
        print(joy_msg)
        if joy_msg.buttons[5] == 1:
            print("[R1] ON")

if __name__ == "__main__":
    rospy.init_node('joy_test')
    joy_test = sub_joy()
    rospy.spin()
