#!/usr/bin/env python
# coding utf-8

import rospy
from std_msgs.msg import String
import serial
from logging import getLogger
logger = getLogger("__name__")

rospy.init_node('arm_controller')

pub = rospy.Publisher('arm_controller_angle', String, queue_size=10)
rate = rospy.Rate(10)
with serial.Serial('/dev/ttyUSB0', 9600) as ser:
    while not rospy.is_shutdown():
        angle_list_str = ser.readline()
        pub.publish(angle_list_str)
        rate.sleep()
        rospy.loginfo(angle_list_str[:-2])
ser.close()

