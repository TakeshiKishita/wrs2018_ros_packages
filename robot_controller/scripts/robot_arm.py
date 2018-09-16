#!/usr/bin/env python
# coding=utf-8
import rospy
import smbus
from std_msgs.msg import String

bus = smbus.SMBus(1)  # I2C通信するためのモジュールsmbusのインスタンスを作成
adress = 0x04  # arduinoのサンプルプログラムで設定したI2Cチャンネル
print('test1')


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
