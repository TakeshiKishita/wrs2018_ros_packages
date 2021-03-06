# coding=utf-8

"""
多脚関節のサーボ制御用プログラム
"""
import Adafruit_PCA9685
# ロガー設定
from logging import getLogger

logger = getLogger("__name__")

PULSE_12bit = 4000


def i2c_angle_control(channel_list, angle, **param):
    """
    Adafruit_PCA96851ドライバを使用し、「角度から」任意のチェンネルのpwmを指定する
    :param channel_list: ドライバのチャンネル番号リスト
    :param angle: 指定角度
    :return bool:
    """
    logger.info("channel:{}".format(channel_list))
    logger.info("angle  :{}".format(angle))
    logger.info("params :{}".format(param))
    try:
        # パルス幅の計算
        pulse_width = int(
            (param["dc_min"] + (param["dc_max"] - param["dc_min"]) * angle / param["angle_max"]) * 4096 / (
                    1000 / param["period_width"]))
        logger.info("pulse_width:{}".format(pulse_width))
        for channel in channel_list:
            param["pwm"].set_pwm(channel, 0, pulse_width)
        return True
    except Exception as e:
        logger.error(e)
        return False


def i2c_duty_control(pwm, channel_list, duty_cycle=1.0):
    """
    Adafruit_PCA9685ドライバを使用し、「デューティ比」から任意のチェンネルのpwmを指定する
    :param pwm: コンストラクタ Adafruit_PCA9685.PCA9685()
    :param channel_list: ドライバのチャンネル番号リスト
    :param duty_cycle: デューティ比
    :return bool:
    """
    try:
        # パルス幅 = 4096 * デューティ比
        pulse_width = int(PULSE_12bit * duty_cycle)
        for channel in channel_list:
            pwm.set_pwm(channel, 0, int(pulse_width))
        return True
    except Exception as e:
        logger.error(e)
        return False


class JointControl:
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
        self.dc_min = 0.5  # 最小パルス幅msec
        self.dc_max = 2.5  # 最大パルス幅msec
        self.angle_max = 180  # 最大角（最小を０とした場合）degrees
        self.period_width = 50  # 周期幅Hz

        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(self.period_width)

    def all_leg_control(self, top_angle, bottom_angle):
        """
        すべての脚を同時に制御する。
        :param top_angle: 脚上関節サーボ角度
        :param bottom_angle: 脚下関節サーボ角度
        :return bool:
        """
        logger.debug("all_leg_control")
        ret = True
        logger.debug("top:{}, bottom:{}".format(top_angle, bottom_angle))
        if self.check_angle(top_angle, self.leg_top_channel):
            if self.check_angle(bottom_angle, self.leg_bottom_channel):
                params = {"pwm": self.pwm,
                          "dc_min": self.dc_min,
                          "dc_max": self.dc_max,
                          "angle_max": self.angle_max,
                          "period_width": self.period_width}
                ret = i2c_angle_control(self.leg_top_channel, top_angle, **params) if ret else False
                ret = i2c_angle_control(self.leg_bottom_channel, bottom_angle, **params) if ret else False
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

        logger.debug("leg_channel_control")
        logger.debug("angle:{}".format(angle))
        params = {"pwm": self.pwm,
                  "dc_min": self.dc_min,
                  "dc_max": self.dc_max,
                  "angle_max": self.angle_max,
                  "period_width": self.period_width}
        ret = i2c_angle_control(channel_list, angle, **params)
        return ret

    def check_angle(self, angle, channel_list):
        """
        角度チェック
        :param angle: 角度
        :param channel_list: チャンネルリスト
        :return:
        """
        channel_set = set(channel_list)
        logger.debug("check_angle")
        if len(list(channel_set & set(self.leg_bottom_channel))) > 0 and 0 <= angle <= 90:
            pass
        elif len(list(channel_set & set(self.leg_top_channel))) > 0 and 90 <= angle <= 180:
            pass
        else:
            logger.error("position:{} に設定した角度{}度は適切ではありません。".format(channel_list, angle))
            return False
        return True


class DriveControl:
    """
    駆動系制御
    """

    def __init__(self):
        self.drive_channel = [8, 12, 10, 14]
        self.back_channel = [9, 13, 11, 15]
        self.period_width = 100  # 周期幅Hz
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(self.period_width)

    def motor_driver_control(self, channel_list, duty_cycle=1.0):
        """
        個別の駆動系の制御
        :param channel_list:
        :param duty_cycle:
        :return:
        """

        ret = True
        try:
            ret = i2c_duty_control(self.pwm, channel_list, duty_cycle)
        except Exception:
            self.pwm.set_all_pwm(0, 0)
        return ret


if __name__ == "__main__":
    try:
        TOP_HOME_ANGLE = 135
        BOTTOM_HOME_ANGLE = 45
        leg_channel = {"r_f": [0, 1],
                       "l_f": [2, 3],
                       "r_b": [4, 5],
                       "l_b": [6, 7]}
        leg_top_channel = [0, 2, 4, 6]
        leg_bottom_channel = [1, 3, 5, 7]
        dc_min = 0.5  # 最小パルス幅msec
        dc_max = 2.5  # 最大パルス幅msec
        angle_max = 180  # 最大角（最小を０とした場合）degrees
        period_width = 50  # 周期幅Hz

        pwm = Adafruit_PCA9685.PCA9685()
        pwm.set_pwm_freq(period_width)

        # パルス幅の計算
        param = {"pwm": pwm,
                 "dc_min": dc_min,
                 "dc_max": dc_max,
                 "angle_max": angle_max,
                 "period_width": period_width}
        pulse_width = int((param["dc_min"] + (param["dc_max"] - param["dc_min"]) * 90 / param["angle_max"]) * 4096 / (
                1000 / param["period_width"]))
        for channel in leg_bottom_channel:
            param["pwm"].set_pwm(channel, 0, pulse_width)
        for channel in leg_top_channel:
            param["pwm"].set_pwm(channel, 0, pulse_width)

    except Exception as e:
        # 全てのPWMを初期化する
        logger.error(e.args)
        jc.pwm.set_all_pwm(0, 0)
