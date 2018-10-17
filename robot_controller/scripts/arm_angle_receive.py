#!/usr/bin/env python
# coding=utf-8
import rospy
import smbus
from std_msgs.msg import String
"""
ロボット側でアームコントローラの角度を受け取り、ロボットアームのArduinoへ角度を伝達する
"""

bus = smbus.SMBus(1)  # I2C通信するためのモジュールsmbusのインスタンスを作成
address = 0x10  # arduinoのサンプルプログラムで設定したI2Cチャンネル
cmd = 0x11


def callback(message):
    msg = message.data[:-2]
    rospy.logdebug(__file__ +' callback: {}'.format(msg))

    try:
        angle_list = list(msg)
        # Arduinoへ文字列を一文字ずつのリストにして送る
        bus.write_i2c_block_data(address ,cmd, [int(x) for x in angle_list])
    except Exception as e:
        rospy.logerr(__file__ +"/ Exception: "+ str(e))


rospy.init_node('listen_arm_angle')
sub = rospy.Subscriber('arm_controller_angle', String, callback)
rospy.spin()
