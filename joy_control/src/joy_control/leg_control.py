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
        pulse_width = int(4096 * duty_cycle)
        for channel in channel_list:
            print("pulse_width: {}".format(pulse_width))
            pwm.set_pwm(channel, 0, int(pulse_width))
        return True
    except Exception as e:
        logger.error(e)
        return False


def check_angle(angle, position="default", logger=None):
    """
    角度チェック
    :param angle: 角度
    :param position: サーボ位置
    :param logger:
    :return:
    """
    logger = logger or _logger
    logger.debug("check_angle")
    if position == "default" and 0 <= angle <= 90:
        pass
    elif position == "top" and 0 < angle <= 180:
        pass
    else:
        logger.error("position:{} に設定した角度は適切ではありません。".format(position))
        return False
    return True


class JointControl(object):
    """
    脚関節制御
    """

    def __init__(self):
        self.drive_channel = {"l_f": [8, 9],
                              "r_f": [10, 11],
                              "l_b": [12, 13],
                              "r_b": [14, 15]}
        self.back_channel = [8, 12, 10, 14]
        self.front_channel = [9, 13, 11, 15]
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
        if check_angle(top_angle, "top"):
            if check_angle(bottom_angle):
                params = {"pwm": self.pwm,
                          "dc_min": self.dc_min,
                          "dc_max": self.dc_max,
                          "angle_max": self.angle_max,
                          "period_width": self.period_width}
                ret = i2c_angle_control(self.leg_top_channel, top_angle, logger=logger,
                                        **params) if ret else False
                ret = i2c_angle_control(self.leg_bottom_channel, bottom_angle, logger=logger,
                                        **params) if ret else False
        else:
            ret = False
        return ret


class DriveControl:
    """
    駆動系制御
    """

    def __init__(self):
        self.drive_channel = {"l_f": [8, 9],
                              "r_f": [10, 11],
                              "l_b": [12, 13],
                              "r_b": [14, 15]}
        self.front_channel = [8, 10, 12, 14]
        self.back_channel = [9, 11, 13, 15]
        self.period_width = 100000  # 周期幅Hz
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(self.period_width)

    def morter_driver_control(self, channel_list, duty_cycle=1.0, logger=None):
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
