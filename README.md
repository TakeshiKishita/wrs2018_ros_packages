# コントローラパッケージ
ROSを使用して、PCとロボット（JETSON）を動かすためのパッケージ

### コントローラ（PC）側設定
**DS4DRV**  
PS4コントローラを使用するためのライブラリ
```bash
sudo pip install ds4drv
```
  
  
**ROS**  
ROS自体のインストールは割愛  
ワークスペースがすでにある場合は「パッケージファイルの追加」以降から
```bash
cd ~
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src/
catkin_init_workspace
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash

# setup.bashの読み込み先の変更
vi ~/.bashrc
-- source /opt/ros/kinetic/setup.bash
++ source ~/catkin_ws/devel/setup.bash
```

パッケージファイルの追加

ROSワークスペース`/src`内にpull  
(cloneは他ファイル、パッケージがあるとできないため)
```bash
cd ~/catkin_ws/src/
git init
git remote add origin https://gitlab.com/ojisan_and_dream/WRS2018/ROS_packages.git
git pull origin master
```

catkinでビルド
```bash
cd ~/catkin_ws
catkin_make
```

ビルド後
```bash
roscd robot_controller
```
上記ができれば成功

### ロボット側（JETSON） 設定  
Adafruit_GPIOライブラリ自体を変更しているので注意  
```bash
# i2c関連のパッケージをインストール
sudo apt install libi2c-dev i2c-tools python-smbus
# i2cが使えるようにユーザーをグループに追加（nvidiaは任意のユーザ名で）
sudo usermod -aG i2c nvidia

# サーボドライバパッケージインストール
sudo pip install adafruit-pca9685
# パッケージソースの変更
sudo vim /usr/local/lib/python2.7/dist-packages/Adafruit_GPIO/I2C.py
                                                                                                                                                                                               
    55   -- raise RuntimeError('Could not determine default I2C bus for platform.')                                                                                                                                
    56   ++ return 0
```

# 使用方法
※ds4drvを使用前提で作成しています。有線でも仕組みは同じですが、  
　ボタン配置などが変わってしまうため、現状のソースコードでは使用できません。
### コントロール端末（PC）側【master】
**joyノードの実行**
```bash
roscd robot_controller/launch

# 自身のIPアドレスを引数に指定
sh ./launch/run_DS4.sh 192.168.xxx.xxx
```

実行後、「ds4drv」も起動されるので、コントローラの"SHARE"ボタンと"PS（ホーム）"ボタンを同時に長押し  
ライトバーが白く点滅したら離す
下記のようなメッセージが出れば接続完了

```bash
[info][bluetooth] Scanning for devices                                    
[info][bluetooth] Found device 30:0E:D5:99:E6:13                          
[info][controller 1] Connected to Bluetooth Controller (30:0E:D5:99:E6:13)
[info][bluetooth] Scanning for devices                                    
[info][controller 1] Battery: 87%                                         
[warning][controller 1] Signal strength is low (31 reports/s)
```

### ロボット（JETSON）側
**joy_test.pyの実行**
```bash
roscd joy_control/launch

# 自身のIPアドレスと、master側のIPアドレスを引数に指定
sh run_joy_control.sh 192.168.yyy.yyy 192.168.xxx.xxx
```

## コントローラ使用メモ
#### axes
 0: L stick horizontal (left=+1, right=-1)  
 1: L stick vertical (up=+1, down=-1)  
 2: R stick horizontal (left=+1, right=-1)  
 3: L2 (neutral=+1, full accel=-1)  
 4: R2 (neutral=+1, full accel=-1)  
 5: R stick vertical (up=+1, down=-1)  
 6: Accelerometer Left(コントローラ左方向が正方向)  
 7: Accelerometer Front(コントローラ手前方向が正方向)  
 8: Accelerometer Up(コントローラ上方向が正方向)  
 9: Axis button(十字キー) LR（L=＋１, R=−１）  
10: Axis button(十字キー) Up/Down（Up=＋１, Down=−１）  
11: Jyrometer Roll (手前から見て右回り：＋、左回り：ー)  
12: Jyrometer Yaw (上から見て左回り：ー、右回り：＋)  
13: Jyrometer Pitch (ライトバー側を上げる：ー, 下げる：＋)  
#### buttons
 0: □Square  
 1: ×Cross  
 2: ○Circle  
 3: △Triangle  
 4: L1  
 ５: R1  
 6: L2 digital(1/3くらい引くと1になる）  
 7: R2 digital(1/3くらい引くと1になる）  
 8: Share  
 9: Options  
10: L3  
11: R3  
12: PS button  
13: Touchpad button  