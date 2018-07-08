#!/usr/bin/env python

import rospy
from joy_control.leg_control import drive_control
from sensor_msgs.msg import Joy
from logging import getLogger
logger = getLogger(__name__)

print("start")
logger.info("logging.config")
rospy.loginfo("rospy.loginfo")


class sub_joy(object):
    def __init__(self):
        self._joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback, queue_size=1)
        #self.drive_channel = {"r_f": [8, 9],
        #                      "l_f": [10, 11],
        #                      "r_b": [12, 13],
        #                      "l_b": [14, 15]}
        #self.front_channel = [8, 10, 12, 14]
        #self.back_channel = [9, 11, 13, 15]
        self.drive_channel = {"l_f": [8, 9],
                              "r_f": [10, 11],
                              "l_b": [12, 13],
                              "r_b": [14, 15]}
        self.back_channel = [8, 12, 10, 14]
        self.front_channel = [9, 13, 11, 15]
        self._joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback, queue_size=1)

    def joy_callback(self, joy_msg):
        if joy_msg.buttons[5] == 1:
            print("[R1] pushed")

            dc = drive_control()
            y_axis_left = joy_msg.axes[1]
            y_axis_right = joy_msg.axes[5]
            print("y_axis_left: {}".format(y_axis_left))
            print("y_axis_right: {}".format(y_axis_right))
            try:
                if y_axis_left >= 0.0:
                    ret = dc.morter_driver_control(self.front_channel[:2], abs(y_axis_left))
                elif y_axis_left < 0.0:
                    ret = dc.morter_driver_control(self.back_channel[:2], abs(y_axis_left))

                if y_axis_right >= 0.0:
                    ret = dc.morter_driver_control(self.front_channel[2:], abs(y_axis_right))
                elif y_axis_right < 0.0:
                    ret = dc.morter_driver_control(self.back_channel[2:], abs(y_axis_right))
                if not ret:
                    raise Exception("joy_callback error")
            except Exception as e:
                import traceback
                traceback.print_exc()
                logger.exception("ERROR: ".format(e))
                print("[error]: {}".format(e.args))


if __name__ == "__main__":
    rospy.init_node('joy_test')
    joy_test = sub_joy()
    rospy.spin()
