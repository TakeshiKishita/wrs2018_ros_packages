#!/usr/bin/env python
# coding=utf-8

import rospy
from joy_control.leg_control import DriveControl
from sensor_msgs.msg import Joy
from logging import getLogger
logger = getLogger(__name__)

print("start")
logger.info("logging.config")
rospy.loginfo("rospy.loginfo")


class SubJoy(object):
    def __init__(self):
        self._joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback, queue_size=1)

    def joy_callback(self, joy_msg):
        """
        ジョイスティックの操作を受ける
        :param joy_msg:
        :return:
        """
        # 駆動系制御
        if joy_msg.buttons[5] == 1:
            print("[R1] pushed")

            dc = DriveControl()
            ret = True
            # 左右スティックの値取得
            y_axis_left = joy_msg.axes[1]
            y_axis_right = joy_msg.axes[5]
            print("y_axis_left: {}".format(y_axis_left))
            print("y_axis_right: {}".format(y_axis_right))
            try:
                if y_axis_left >= 0.0:
                    ret = dc.morter_driver_control(dc.front_channel[:2], abs(y_axis_left))
                elif y_axis_left < 0.0:
                    ret = dc.morter_driver_control(dc.back_channel[:2], abs(y_axis_left))

                if y_axis_right >= 0.0:
                    ret = dc.morter_driver_control(dc.front_channel[2:], abs(y_axis_right))
                elif y_axis_right < 0.0:
                    ret = dc.morter_driver_control(dc.back_channel[2:], abs(y_axis_right))
                if not ret:
                    raise Exception()
            except Exception as e:
                import traceback
                traceback.print_exc()
                print("[error]: {}".format(e.args))

        # 足関節系制御
        elif joy_msg.buttons[5] == 1:
            # 同時操作できないようelif
            print("[R1] pushed")


if __name__ == "__main__":
    rospy.init_node("joy_control")
    joy_control = SubJoy()
    rospy.spin()
