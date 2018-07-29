#!/usr/bin/env python
# coding=utf-8

import traceback
import rospy
from joy_control.leg_control import DriveControl, JointControl
from sensor_msgs.msg import Joy

# loggerの設定
from logging import getLogger, StreamHandler, DEBUG, config
# config.fileConfig('logging.conf')
logger = getLogger("__name__")
handler = StreamHandler()
handler.setLevel(DEBUG)
logger.setLevel(DEBUG)
logger.addHandler(handler)
logger.propagate = False
logger.info("START")
# クラス初期化
dc = DriveControl()
jc = JointControl()
dc.motor_driver_control(dc.drive_channel, abs(0))
dc.motor_driver_control(dc.back_channel, abs(0))

TOP_MAX_ANGLE = 180
BOTTOM_MAX_ANGLE = 90
TOP_HOME_ANGLE = 135
BOTTOM_HOME_ANGLE = 45


class SubJoy(object):
    def __init__(self):
        self._joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback, queue_size=100)
        self.top_angle = 135
        self.bottom_angle = 45

    def joy_callback(self, joy_msg):
        """
        ジョイスティックの操作を受ける
        :param joy_msg:
        :return:
        """
        # 左右スティックの値取得
        y_axis_left = joy_msg.axes[1]
        y_axis_right = joy_msg.axes[5]
        # ボタンの値取得
        button_l1 = joy_msg.buttons[4]
        button_r1 = joy_msg.buttons[5]
        circle_button = joy_msg.buttons[2]

        # 駆動系制御
        if button_r1 == 1:
            logger.debug("[R1] pushed")

            ret = True
            logger.debug("y_axis_left: {}".format(y_axis_left))
            logger.debug("y_axis_right: {}".format(y_axis_right))
            try:
                if y_axis_left >= 0.0:
                    dc.motor_driver_control(dc.drive_channel[2:], abs(y_axis_left))
                    ret = dc.motor_driver_control(dc.back_channel[2:], abs(0))
                elif y_axis_left < 0.0:
                    dc.motor_driver_control(dc.back_channel[2:],  abs(y_axis_left))
                    ret = dc.motor_driver_control(dc.drive_channel[2:], abs(0))

                if y_axis_right >= 0.0:
                    dc.motor_driver_control(dc.drive_channel[:2],  abs(y_axis_right))
                    ret = dc.motor_driver_control(dc.back_channel[:2], abs(0))
                elif y_axis_right < 0.0:
                    dc.motor_driver_control(dc.back_channel[:2], abs(y_axis_right))
                    ret = dc.motor_driver_control(dc.drive_channel[:2], abs(0))
                if not ret:
                    raise Exception()
            except Exception as e:
                traceback.print_exc()
                logger.error(e.args)

        # 足関節系制御
        elif button_l1 == 1:
            # 同時操作できないようelif
            logger.debug("[L1] pushed")
            try:
                if circle_button == 1:
                    print("[○] pushed")
                    # ○ボタンが押された場合、ホームポジションに戻る
                    ret = jc.leg_channel_control(TOP_HOME_ANGLE, jc.leg_top_channel)
                    ret = jc.leg_channel_control(BOTTOM_HOME_ANGLE, jc.leg_bottom_channel) if ret else ret
                else:
                    if abs(y_axis_right) > 0:
                        self.top_angle = self.top_angle + 5 if (0 <= self.top_angle <= TOP_MAX_ANGLE) else self.top_angle
                        ret = jc.leg_channel_control(self.top_angle, jc.leg_top_channel)

                        self.top_angle = self.top_angle - 5 if (0 <= self.bottom_angle <= BOTTOM_MAX_ANGLE) else self.bottom_angle
                        ret = jc.leg_channel_control(self.bottom_angle, jc.leg_bottom_channel) if ret else ret

                if not ret:
                    raise Exception()
            except Exception as e:
                jc.leg_channel_control(self.bottom_angle, jc.leg_bottom_channel)
                traceback.print_exc()
                logger.error(e.args)

        if button_r1 != 1:
            # ボタンを離したらDCモータを止める
            dc.motor_driver_control(dc.drive_channel, abs(0))
            dc.motor_driver_control(dc.back_channel, abs(0))


if __name__ == "__main__":
    try:
        rospy.init_node("joy_control")
        joy_control = SubJoy()
        rospy.spin()
    except Exception as e:
        # 全てのPWMを初期化する
        logger.error(e.args)
        dc.pwm.set_all_pwm(0, 0)
        jc.pwm.set_all_pwm(0, 0)
