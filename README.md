# コントローラパッケージ
## 導入
ROSワークスペース作成は省略  

ROSワークスペース`/src`内にpull  
(cloneは他パッケージがあるとできないため)
```bash
$ git remote add origin git@gitlab.com:ojisan_and_dream/WRS2018/RasPi_code.git
$ git pull origin master
```

catkinでビルド
```bash
$ cd ~/<WORK SPACE>
$ catkin_make
```