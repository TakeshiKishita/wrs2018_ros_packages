#!/usr/bin/env python
# coding=utf-8
import rospy
import smbus
from std_msgs.msg import String
"""
ロボット側でアームコントローラの角度を受け取り、ロボットアームのArduinoへ角度を伝達する
"""

bus = smbus.SMBus(0)  # I2C通信するためのモジュールsmbusのインスタンスを作成
address = 0x10  # arduinoのサンプルプログラムで設定したI2Cチャンネル
cmd = 0x11


def callback(message):
    msg = message.data[:-2]
    rospy.loginfo('I heard {}'.format(msg))

    try:
        angle_list = list(msg)
        # Arduinoへ文字『R』を送る、ordはアスキーコードを取得
        bus.write_i2c_block_data(address ,cmd, angle_list)
    except Exception as e:
        rospy.loginfo(str(e))


rospy.init_node('listen_arm_angle')
sub = rospy.Subscriber('arm_controller_angle', String, callback)
rospy.spin()
