#!/usr/bin/env python
# coding=utf-8

import rospy
from std_msgs.msg import String
import serial
"""
アームコントローラからシリアル通信で値を取得し、ROSに角度を投げる
"""

rospy.init_node('arm_controller')

pub = rospy.Publisher('arm_controller_angle', String, queue_size=10)
rate = rospy.Rate(10)
# 起動後、途中でArduinoとの接続を想定しているので、接続待ちの状態で落ちないようにしている。
# TODO ネストが深く、無限ループなので、改善必要
while True:
    try:
        with serial.Serial('/dev/ttyUSB0', 9600) as ser:
            # シリアルの接続を確率
            while not rospy.is_shutdown():
                # ROSが終了しない限り通信し続ける。
                angle_list_str = ser.readline()
                pub.publish(angle_list_str)
                rate.sleep()
                rospy.logdebug(__file__ +"/ angle_list_str: " + angle_list_str[:-2])

            # ROSが終了した段階でループを抜けて終了
            ser.close()
            break
    except:
        # 例外が発生した場合でも、接続を試み続ける
        continue
