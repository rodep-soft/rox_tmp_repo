# ロボットの動かし方

## 自分のPCから
```
mypc> ssh rodep@raspberrypi 
# raspberrypiはラズパイのIP アドレス。passwdは"rodep"
raspberrypi> bluetoothctl
blue> devices
PS5 Controller MACADDR(:で区切られてるやつ)
blue> connect MACADDR
# 繋がらなかったらコントローラーのselectとpsボタンを青く光るまで押す
ps5controller> exit
raspberrypi> cd
raspberrypi> cd yano_working
raspberrypi> cd rox_tmp_repo
raspberrypi> just 
raspberrypi-docker> bash
raspberrypi-docker> just # ros2_wsのビルド
raspberrypi-docker> just launch 
```

## Precaution
- ゾンビnodeが生きてるときはjust kill
