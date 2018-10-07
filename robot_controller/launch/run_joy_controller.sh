#!/usr/bin/env bash

if [ $# -ne 1 ]; then
  echo "指定された引数がありません。" 1>&2
  echo "実行するには1個の引数が必要です。" 1>&2
  echo "1: ROS_IP（端末のIPアドレス）" 1>&2

  exit 1
fi

# コントローラドライバを起動して、接続を待機
sudo ds4drv &
while [[ ! -e /dev/input/js0 ]]; do
  echo "Couldn't open joystick /dev/input/js0. Will retry every 10second."
  sleep 10
done

# 自身IPの指定
export ROS_IP=$1
roslaunch robot_controller robot_controller.launch
