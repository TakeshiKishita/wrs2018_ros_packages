# -*- coding:utf-8 -*-

"""
多脚関節のサーボ制御用プログラム
"""
import common
import Adafruit_PCA9685
# ロガー設定
from logging import getLogger, StreamHandler, DEBUG

_logger = getLogger("__name__")
handler = StreamHandler()
handler.setLevel(DEBUG)
_logger.setLevel(DEBUG)
_logger.addHandler(handler)
_logger.propagate = False


class joint_control(object):
    """
    脚関節制御
    """

    def __init__(self):
        self.leg_channel = {"r_f": [0, 1],
                            "l_f": [2, 3],
                            "r_b": [4, 5],
                            "l_b": [6, 7]}
        self.leg_top_channel = [0, 2, 4, 6]
        self.leg_bottom_channel = [1, 3, 5, 7]
        self.dc_min = 0.4  # 最小パルス幅msec
        self.dc_max = 2.5  # 最大パルス幅msec
        self.angle_max = 180  # 最大角（最小を０とした場合）degrees
        self.pwm_period = 50  # 周期幅Hz
        self.logger = _logger
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(self.pwm_period)

    def i2c_angle_control(self, channel_list, angle):
        """
        Adafruit_PCA96851ドライバを使用し、任意のチェンネルのpwmを指定する
        :param channel_list: ドライバのチャンネル番号
        :param angle: 指定角度
        :return bool:
        """
        try:
            # パルスの終点設定
            pulse_width = int((self.dc_min + (self.dc_max - self.dc_min) * angle / self.angle_max) * 4096 / (
                    1000 / self.pwm_period))
            _logger.info("pulse_width:{}".format(pulse_width))
            for channel in channel_list:
                self.pwm.set_pwm(channel, 0, pulse_width)
            _logger.info("channels:{}, pulse_width:{}".format(str(channel_list), pulse_width))
            return True

        except Exception as e:
            _logger.error(e)
            return False

    def all_leg_control(self, top_angle, bottom_angle, logger=None):
        """
        すべての脚を同時に制御する。
        :param top_angle: 脚上関節サーボ角度
        :param bottom_angle: 脚下関節サーボ角度
        :param logger:
        :return bool:
        """
        logger = logger or _logger

        logger.debug("all_leg_control")
        ret = True
        logger.debug("top:{}, bottom:{}".format(top_angle, bottom_angle))
        if common.check_angle(top_angle, "top"):
            if common.check_angle(bottom_angle):
                params = {"pwm": self.pwm,
                          "dc_min": self.dc_min,
                          "dc_max": self.dc_max,
                          "angle_max": self.angle_max,
                          "period_width": self.period_width}
                ret = common.i2c_angle_control(self.leg_top_channel, top_angle, logger=logger,
                        **params) if ret else False
                ret = common.i2c_angle_control(self.leg_bottom_channel, bottom_angle, logger=logger,
                    **params) if ret else False
        else:
            ret = False
        return ret


if __name__ == '__main__':
    """
    デバッグ用
    コマンドラインから直接呼び出し、一定の角度に調整
    """
    def __init__(self):
        self.drive_channel = {"l_f": [8, 9],
                              "l_b": [10, 11],
                              "r_f": [12, 13],
                              "r_b": [14, 15]}
        self.front_channel = [8, 10, 12, 14]
        self.back_channel = [9, 11, 13, 15]
        self.period_width = 50  # 周期幅Hz
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(self.period_width)

    def drive_control(self, channel_list, duty_cycle=1.0, logger=None):
        """
        個別の駆動系の制御
        :param channel_list:
        :param duty_cycle:
        :param logger:
        :return:
        """
        logger = logger or _logger

        ret = True
        try:
            ret = common.i2c_duty_control(self.pwm, channel_list, self.period_width, duty_cycle, logger=logger)
        except:
            self.pwm.set_all_pwm(0, 0)
        return ret
