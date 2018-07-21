# コントローラパッケージ
## 導入
### DS4DRV
PS4コントローラを使用するためのライブラリ
```bash
sudo pip install ds4drv
```
### ROSライブラリ
ROSワークスペース作成は省略  

ROSワークスペース`/src`内にpull  
(cloneは他パッケージがあるとできないため)
```bash
cd ~/<WORK SPACE>/src/
git remote add origin git@gitlab.com:ojisan_and_dream/WRS2018/RasPi_code.git
git pull origin master
```

catkinでビルド
```bash
cd ~/<WORK SPACE>
catkin_make
```
### Jetson TX1設定
I2C.pyの記述を変更（ライブラリ自体を変更しているので注意）  
この場合"BUS 0" を使用  
```bash
sudo chmod 666 /dev/i2c-0
sudo vim /usr/local/lib/python2.7/dist-packages/Adafruit_GPIO/I2C.py
                                                                                                                                                                                               
    55   -- raise RuntimeError('Could not determine default I2C bus for platform.')                                                                                                                                
    56   ++ return 0
```

## 使用方法
※ds4drvを使用前提で作成しています。


コントロール端末側（master）
```bash
roscd joy_control/launch

# 自身のIPアドレスを引数に指定
sh ./launch/run_DS4.sh 192.168.xxx.xxx
```

ロボット側
```bash
roscd joy_control/launch

# 自身のIPアドレスと、master側のIPアドレスを引数に指定
sh run_joy_control.sh 192.168.yyy.yyy 192.168.xxx.xxx
```

## コントローラ使用メモ
### axes
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
### buttons
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