#!/usr/bin/env bash

if [ $# -ne 2 ]; then
  echo "指定された引数がありません。" 1>&2
  echo "実行するには2個の引数が必要です。" 1>&2
  echo "1: ROS_IP" 1>&2
  echo "2: ROS_MASTER_URI" 1>&2
  exit 1
fi

# 自身IPの指定
export ROS_IP=$1
# master側のIPを指定
export ROS_MASTER_URI=http://$2:11311
# コントロールスクリプトの実行
rosrun joy_control joy_test.py &
