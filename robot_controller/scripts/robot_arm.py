#!/usr/bin/env python
import rospy
from std_msgs.msg import String


def callback(message):
    rospy.loginfo('I heard {}'.format(message.data[:-2]))


rospy.init_node('listen_arm_angle')
sub = rospy.Subscriber('arm_controller_angle', String, callback)
rospy.spin()
