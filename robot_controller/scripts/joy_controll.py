#!/usr/bin/env python
# coding=utf-8

import traceback
from time import sleep
import rospy
from robot_controller.leg_control import DriveControl, JointControl
from sensor_msgs.msg import Joy

# loggerの設定
from logging import getLogger, StreamHandler, DEBUG, INFO, config
# config.fileConfig('logging.conf')
logger = getLogger("__name__")
handler = StreamHandler()
logger.setLevel(INFO)
logger.addHandler(handler)
logger.propagate = False
logger.info("START")
# クラス初期化
dc = DriveControl()
jc = JointControl()
dc.motor_driver_control(dc.drive_channel, 0)
dc.motor_driver_control(dc.back_channel, 0)

TOP_MAX_ANGLE = 180
BOTTOM_MAX_ANGLE = 90
TOP_HOME_ANGLE = 120
BOTTOM_HOME_ANGLE = 45


class SubJoy(object):
    def __init__(self):
        self._joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback, queue_size=100)
        self.top_angle = 120
        self.bottom_angle = 45

    def joy_callback(self, joy_msg):
        """
        ジョイスティックの操作を受ける
        :param joy_msg:
        :return:
        """
        # 左右スティックの値取得
        joy_left_y_axis = joy_msg.axes[1]
        joy_right_y_axis = joy_msg.axes[5]

        # 十字キーの値取得
        plus_buttoon_x_axis = joy_msg.axes[9]
        plus_buttoon_y_axis = joy_msg.axes[10]

        # ボタンの値取得
        button_l1 = joy_msg.buttons[4]
        button_r1 = joy_msg.buttons[5]
        button_r2 = joy_msg.axes[4]
        circle_button = joy_msg.buttons[2]

        # 駆動系制御
        if button_r1 == 1:
            # R1ボタンを押したままジョイスティックを倒すと、左右のキャタピラが回転する
            logger.info("[R1] pushed")
            self.drive_controll(joy_left_y_axis, joy_right_y_axis)

        elif button_r2 != 1.0:
            # R2ボタンを押したままジョイスティックを倒すと、ゆっくり左右のキャタピラが回転する
            max_speed_rate = 0.6
            logger.info("[R2] pushed")
            # 入力値の上限を超えた場合は、上限値で入力する
            input_left = joy_left_y_axis if joy_left_y_axis < max_speed_rate else max_speed_rate
            input_right = joy_right_y_axis if joy_right_y_axis < max_speed_rate else max_speed_rate
            self.drive_controll(input_left, input_right)

        # 足関節系制御
        elif button_l1 == 1:
            # 同時操作できないようelif
            logger.info("[L1] pushed")
            try:
                if circle_button == 1:
                    logger.info("[○] pushed")
                    # ○ボタンが押された場合、ホームポジションに戻る

                    # 補助のためにキャタピラを動かす
                    direction = 1 if self.top_angle > TOP_HOME_ANGLE else 0
                    self.suppurt_drive_controll(direction)
                    sleep_time = abs(self.top_angle - TOP_HOME_ANGLE)*0.01

                    self.top_angle = TOP_HOME_ANGLE
                    self.bottom_angle = BOTTOM_HOME_ANGLE
                    jc.leg_channel_control(TOP_HOME_ANGLE, jc.leg_top_channel)
                    jc.leg_channel_control(BOTTOM_HOME_ANGLE, jc.leg_bottom_channel)

                    # キャタピラ補助のためにsleep
                    sleep(sleep_time)
                else:
                    if plus_buttoon_y_axis != 0:
                        # L1ボタンを押しながら十字キー上下を操作した場合、車体の脚関節が上下に動く
                        self.leg_top_angle_controll(plus_buttoon_y_axis)
                        self.leg_bottom_angle_controll(plus_buttoon_y_axis)
                        # 補助のためにキャタピラを動かす
                        direction = 1 if plus_buttoon_y_axis > 0 else 0
                        self.suppurt_drive_controll(direction)

                    elif plus_buttoon_x_axis != 0:
                        # つま先上げ調整角度
                        adjustment_angle = 10
                        if plus_buttoon_x_axis < 0:
                            # L1ボタンを押しながら十字キー上下を操作した場合、キャタピラが15度傾く
                            self.leg_bottom_angle_controll(adjustment_angle)
                        elif plus_buttoon_x_axis > 0:
                            self.leg_bottom_angle_controll(adjustment_angle*-1)
                        # 誤操入力防止のため処置待機
                        sleep(0.1)

            except Exception as e:
                jc.leg_channel_control(self.bottom_angle, jc.leg_bottom_channel)
                traceback.print_exc()
                logger.error(e.args)

        if button_r1 != 1:
            # ボタンを離したらDCモータを止める
            dc.motor_driver_control(dc.drive_channel, 0)
            dc.motor_driver_control(dc.back_channel, 0)

    def leg_top_angle_controll(self, add_angle):
        """
        脚関節上の角度制御
        :param add_angle:
        :return:
        """
        angle = self.top_angle - add_angle
        self.top_angle = angle if (0 < angle < TOP_MAX_ANGLE) else self.top_angle
        jc.leg_channel_control(self.top_angle, jc.leg_top_channel)

    def leg_bottom_angle_controll(self, add_angle):
        """
        脚関節下の角度制御
        :param add_angle:
        :return:
        """
        angle = self.bottom_angle + add_angle
        self.bottom_angle = angle if (0 < angle < BOTTOM_MAX_ANGLE) else self.bottom_angle
        jc.leg_channel_control(self.bottom_angle, jc.leg_bottom_channel)

    def drive_controll(self, joy_left_y_axis, joy_right_y_axis):
        """
        キャタピラの駆動制御
        :param joy_left_y_axis:
        :param joy_right_y_axis:
        :return:
        """
        logger.debug("joy_left_y_axis: {}".format(joy_left_y_axis))
        logger.debug("joy_right_y_axis: {}".format(joy_right_y_axis))
        try:
            if joy_left_y_axis >= 0.0:
                dc.motor_driver_control(dc.drive_channel[2:], abs(joy_left_y_axis))
                dc.motor_driver_control(dc.back_channel[2:], 0)
            elif joy_left_y_axis < 0.0:
                dc.motor_driver_control(dc.back_channel[2:], abs(joy_left_y_axis))
                dc.motor_driver_control(dc.drive_channel[2:], 0)

            if joy_right_y_axis >= 0.0:
                dc.motor_driver_control(dc.drive_channel[:2], abs(joy_right_y_axis))
                dc.motor_driver_control(dc.back_channel[:2], 0)
            elif joy_right_y_axis < 0.0:
                dc.motor_driver_control(dc.back_channel[:2], abs(joy_right_y_axis))
                dc.motor_driver_control(dc.drive_channel[:2], 0)
        except Exception as e:
            traceback.print_exc()
            logger.error(e.args)

    def suppurt_drive_controll(self, direction, drive_speed=0.7):
        """
        関節角度調整時のキャタピラの駆動制御
        :param direction: 0:CLOSE, 1:OPEN
        :return:
        """
        try:
            if direction:
                # 開脚時
                dc.motor_driver_control([dc.drive_channel[1], dc.drive_channel[3]], drive_speed)
                dc.motor_driver_control([dc.back_channel[1], dc.back_channel[3]], 0)
                dc.motor_driver_control([dc.back_channel[0], dc.back_channel[2]], drive_speed)
                dc.motor_driver_control([dc.drive_channel[0], dc.drive_channel[2]], 0)
            else:
                # 閉じる
                dc.motor_driver_control([dc.drive_channel[0], dc.drive_channel[2]], drive_speed)
                dc.motor_driver_control([dc.back_channel[0], dc.back_channel[2]], 0)
                dc.motor_driver_control([dc.back_channel[1], dc.back_channel[3]], drive_speed)
                dc.motor_driver_control([dc.drive_channel[1], dc.drive_channel[3]], 0)
        except Exception as e:
            traceback.print_exc()
            logger.error(e.args)


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
