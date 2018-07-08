# -*- coding:utf-8 -*-

"""
共通関数
"""
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
        # パルス幅 = 4096 / 周波数(ms) * デューティ比
        duty_cycle = param["dc_min"] + (param["dc_max"] - param["dc_min"]) * angle / param["angle_max"]
        pulse_width = int(param["period_width"] * duty_cycle)
        logger.info("pulse_width:{}".format(pulse_width))
        for channel in channel_list:
            param["pwm"].set_pwm(channel, 0, pulse_width)
        return True

    except Exception as e:
        logger.error(e)
        return False


def i2c_duty_control(pwm, channel_list, period_width, duty_cycle=1, logger=None):
    """
    Adafruit_PCA96851ドライバを使用し、「デューティ比」から任意のチェンネルのpwmを指定する
    :param pwm: コンストラクタ Adafruit_PCA9685.PCA9685()
    :param channel_list: ドライバのチャンネル番号リスト
    :param period_width:
    :param duty_cycle: デューティ比
    :param logger:
    :return bool:
    """
    logger = logger or _logger
    try:
        # パルス幅 = 4096 / 周波数(ms) * デューティ比
        pulse_width = (4000) * duty_cycle
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
