#!/usr/bin/env python
# coding=utf-8

import rospy
from joy_control.leg_control import DriveControl, JointControl
from sensor_msgs.msg import Joy
from logging import getLogger, StreamHandler, DEBUG
logger = getLogger(__name__)

print("start")
logger.info("logging.config")
rospy.loginfo("rospy.loginfo")


class SubJoy(object):
    def __init__(self):
        self._joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback, queue_size=1)
        self.top_angle = 135
        self.bottom_angle = 45

        self.logger = getLogger("__name__")
        handler = StreamHandler()
        handler.setLevel(DEBUG)
        self.logger.setLevel(DEBUG)
        self.logger.addHandler(handler)
        self.logger.propagate = False

    def joy_callback(self, joy_msg):
        """
        ジョイスティックの操作を受ける
        :param joy_msg:
        :return:
        """
        # クラス初期化
        dc = DriveControl()
        jc = JointControl()

        # 左右スティックの値取得
        y_axis_left = joy_msg.axes[1]
        y_axis_right = joy_msg.axes[5]
        # ボタンの値取得
        l1_button = joy_msg.buttons[4]
        r1_button = joy_msg.buttons[5]
        circle_button = joy_msg.buttons[2]

        # 駆動系制御
        if r1_button == 1:
            print("[R1] pushed")

            ret = True
            print("y_axis_left: {}".format(y_axis_left))
            print("y_axis_right: {}".format(y_axis_right))
            try:
                if y_axis_left >= 0.0:
                    ret = dc.motor_driver_control(dc.front_channel[:2], abs(y_axis_left))
                elif y_axis_left < 0.0:
                    ret = dc.motor_driver_control(dc.back_channel[:2], abs(y_axis_left))

                if y_axis_right >= 0.0:
                    ret = dc.motor_driver_control(dc.front_channel[2:], abs(y_axis_right))
                elif y_axis_right < 0.0:
                    ret = dc.motor_driver_control(dc.back_channel[2:], abs(y_axis_right))
                if not ret:
                    raise Exception()
            except Exception as e:
                import traceback
                traceback.print_exc()
                print("[error]: morter_driver_control, {}".format(e.args))

        # 足関節系制御
        elif l1_button == 1:
            # 同時操作できないようelif
            print("[L1] pushed")
            if circle_button == 1:
                # ○ボタンが押された場合、ホームポジションに戻る
                self.top_angle = 135
                self.bottom_angle = 45
                jc.leg_channel_control(self.top_angle, jc.leg_top_channel)
                jc.leg_channel_control(self.bottom_angle, jc.leg_bottom_channel)
            else:
                self.top_angle = self.top_angle - y_axis_right
                ret = jc.leg_channel_control(self.top_angle, jc.leg_top_channel)

                self.bottom_angle = self.bottom_angle + y_axis_right
                ret = jc.leg_channel_control(self.bottom_angle, jc.leg_bottom_channel)

        else:
            # ボタンを離したら止める
            dc.motor_driver_control(dc.back_channel[2:], abs(0))


if __name__ == "__main__":
    rospy.init_node("joy_control")
    joy_control = SubJoy()
    rospy.spin()
