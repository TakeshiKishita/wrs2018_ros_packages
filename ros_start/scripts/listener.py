#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(message):
    rospy.loginfo('I heard {}'.format(message.data))

rospy.init_node('lostener')
sub = rospy.Subscriber('chatter', String, callback)
rospy.spin()

