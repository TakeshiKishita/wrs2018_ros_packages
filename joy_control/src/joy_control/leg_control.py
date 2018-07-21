# coding=utf-8

"""
多脚関節のサーボ制御用プログラム
"""
import Adafruit_PCA9685
# ロガー設定
from logging import getLogger, StreamHandler, DEBUG

_logger = getLogger("__name__")
handler = StreamHandler()
handler.setLevel(DEBUG)
_logger.setLevel(DEBUG)
_logger.addHandler(handler)
_logger.propagate = False


def i2c_angle_control(channel_list, angle, logger=None, **param):
    """
    Adafruit_PCA96851ドライバを使用し、「角度から」任意のチェンネルのpwmを指定する
    :param channel_list:
    :param logger: ドライバのチャンネル番号リスト
    :param angle: 指定角度
    :return bool:
    """
    logger = logger or _logger
    logger.info("channel:{}".format(channel_list))
    logger.info("angle  :{}".format(angle))
    logger.info("logger :{}".format(logger))
    logger.info("params :{}".format(param))
    try:
        # パルス幅 = 4096 * デューティ比
        duty_cycle = param["dc_min"] + (param["dc_max"] - param["dc_min"]) * angle / param["angle_max"]
        pulse_width = int(4096 * duty_cycle)
        logger.info("pulse_width:{}".format(pulse_width))
        for channel in channel_list:
            param["pwm"].set_pwm(channel, 0, pulse_width)
        return True

    except Exception as e:
        logger.error(e)
        return False


def i2c_duty_control(pwm, channel_list, duty_cycle=1.0, logger=None):
    """
    Adafruit_PCA9685ドライバを使用し、「デューティ比」から任意のチェンネルのpwmを指定する
    :param pwm: コンストラクタ Adafruit_PCA9685.PCA9685()
    :param channel_list: ドライバのチャンネル番号リスト
    :param duty_cycle: デューティ比
    :param logger:
    :return bool:
    """
    logger = logger or _logger
    try:
        # パルス幅 = 4096 * デューティ比
        pulse_width = int(2000 * duty_cycle)
        for channel in channel_list:
            print("[i2c_duty_control] pulse_width: {}".format(pulse_width))
            pwm.set_pwm(channel, 0, int(pulse_width))
        return True
    except Exception as e:
        logger.error(e)
        return False


class JointControl(object):
    """
    脚関節制御
    """

    def __init__(self, logger=None):
        self.leg_channel = {"r_f": [0, 1],
                            "l_f": [2, 3],
                            "r_b": [4, 5],
                            "l_b": [6, 7]}
        self.leg_top_channel = [0, 2, 4, 6]
        self.leg_bottom_channel = [1, 3, 5, 7]
        self.dc_min = 0.5  # 最小パルス幅msec
        self.dc_max = 2.5  # 最大パルス幅msec
        self.angle_max = 180  # 最大角（最小を０とした場合）degrees
        self.period_width = 50  # 周期幅Hz
        self.logger = _logger
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(self.period_width)

        self.logger = getLogger("__name__")
        self.handler = StreamHandler()
        self.handler.setLevel(DEBUG)
        self.logger.setLevel(DEBUG)
        self.logger.addHandler(handler)
        self.logger.propagate = False

        self.logger = logger or self.logger

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
                    1000 / self.period_width))
            self.logger.info("pulse_width:{}".format(pulse_width))
            for channel in channel_list:
                self.pwm.set_pwm(channel, 0, pulse_width)
                self.logger.info("channels:{}, pulse_width:{}".format(str(channel_list), pulse_width))
            return True

        except Exception as e:
            self.logger.error(e)
            return False

    def all_leg_control(self, top_angle, bottom_angle):
        """
        すべての脚を同時に制御する。
        :param top_angle: 脚上関節サーボ角度
        :param bottom_angle: 脚下関節サーボ角度
        :return bool:
        """
        self.logger.debug("all_leg_control")
        ret = True
        self.logger.debug("top:{}, bottom:{}".format(top_angle, bottom_angle))
        if self.check_angle(top_angle, self.leg_top_channel):
            if self.check_angle(bottom_angle, self.leg_bottom_channel):
                params = {"pwm": self.pwm,
                          "dc_min": self.dc_min,
                          "dc_max": self.dc_max,
                          "angle_max": self.angle_max,
                          "period_width": self.period_width}
                ret = i2c_angle_control(self.leg_top_channel, top_angle, logger=self.logger,
                                        **params) if ret else False
                ret = i2c_angle_control(self.leg_bottom_channel, bottom_angle, logger=self.logger,
                                        **params) if ret else False
        else:
            ret = False
        return ret

    def leg_channel_control(self, angle, channel_list):
        """
        チャンネル毎に角度を制御する
        :param angle: サーボ角度
        :param channel_list: 対象チャンネル
        :return bool:
        """

        self.logger.debug("leg_channel_control")
        ret = True
        self.logger.debug("angle:{}".format(angle))
        if self.check_angle(angle, channel_list):
            params = {"pwm": self.pwm,
                      "dc_min": self.dc_min,
                      "dc_max": self.dc_max,
                      "angle_max": self.angle_max,
                      "period_width": self.period_width}
            ret = i2c_angle_control(self.leg_top_channel, angle, logger=self.logger,
                                    **params) if ret else False
        else:
            ret = False
        return ret

    def check_angle(self, angle, channel_list):
        """
        角度チェック
        :param angle: 角度
        :param channel_list: チャンネルリスト
        :return:
        """
        channel_set = set(channel_list)
        self.logger.debug("check_angle")
        if len(list(channel_set & set(self.leg_bottom_channel))) > 0 and 0 <= angle <= 90:
            pass
        elif len(list(channel_set & set(self.leg_top_channel))) > 0 and 90 <= angle <= 180:
            pass
        else:
            self.logger.error("position:{} に設定した角度{}度は適切ではありません。".format(channel_list, angle))
            return False
        return True


class DriveControl:
    """
    駆動系制御
    """

    def __init__(self):
        self.drive_channel = {"l_f": [8, 9],
                              "r_f": [10, 11],
                              "l_b": [12, 13],
                              "r_b": [14, 15]}
        self.front_channel = [8, 12, 10, 14]
        self.back_channel = [9, 13, 11, 15]
        self.period_width = 100  # 周期幅Hz
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(self.period_width)

    def motor_driver_control(self, channel_list, duty_cycle=1.0, logger=None):
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
            ret = i2c_duty_control(self.pwm, channel_list, duty_cycle, logger=logger)
        except Exception:
            self.pwm.set_all_pwm(0, 0)
        return ret
