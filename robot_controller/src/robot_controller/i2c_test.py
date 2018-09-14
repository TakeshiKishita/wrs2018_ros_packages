# -*- coding:utf-8 -*-

import Adafruit_PCA9685
from logging import getLogger

logger = getLogger(__name__)

dc_min = 0.5  # 最小パルス幅msec
dc_max = 2.5  # 最大パルス幅msec
angle_max = 180.0  # 最大角（最小を０とした場合）degrees
pwm_period = 50.0  # 周期幅Hz

pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(50)
try:
    while True:
        cmd = input(">")
        if cmd == "":
            continue
        elif 0 <= float(cmd) <= 180:
            pulse_width = (dc_min + (dc_max - dc_min) * float(cmd) / angle_max) * 4096 / (
                    1000 / pwm_period)
            print("pulse_width:",int(pulse_width))
            pwm.set_pwm(0, 0, int(pulse_width))
except Exception as e:
    logger.error(e)
    pwm.set_all_pwm(0, 0)
finally:
    pwm.set_all_pwm(0, 0)
