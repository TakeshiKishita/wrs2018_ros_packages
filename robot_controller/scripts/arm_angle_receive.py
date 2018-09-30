#!/usr/bin/env python
# coding=utf-8
import rospy
import smbus
from std_msgs.msg import String
"""
ロボット側でアームコントローラの角度を受け取り、ロボットアームのArduinoへ角度を伝達する
"""

bus = smbus.SMBus(1)  # I2C通信するためのモジュールsmbusのインスタンスを作成
adress = 0x10  # arduinoのサンプルプログラムで設定したI2Cチャンネル


def callback(message):
    rospy.loginfo('I heard {}'.format(message.data[:-2]))

    try:
        # Arduinoへ文字『R』を送る、ordはアスキーコードを取得
        bus.write_byte(adress, ord(message.data))
    except Exception as e:
        rospy.loginfo(str(e))


rospy.init_node('listen_arm_angle')
sub = rospy.Subscriber('arm_controller_angle', String, callback)
rospy.spin()
