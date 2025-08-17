# 本番環境(ラズパイ内)のディレクトリ構成

~/rox_tmp_repo以下にこのリポジトリのコードが存在

~はログイン時のホームディレクトリ

# ロボットの起動

```bash
# 1. マキタバッテリーのスイッチを入れる。

# 2. 緊急停止ボタンを上げる。この時、電圧が20V程度でていることを確認する。

# 3. ラズパイのスイッチを入れる。赤点灯は付いてない。黄色点灯でOK

# 4. ssh (raspberrypiはtailscale環境のみで有効。適切なipアドレスに置き換える)
# パスワードはrodep
ssh rodep@raspberrypi

# 5. PS5コントローラを接続する。PSボタンを押せば自動で接続される。
# また、接続確認をする (js0, js1等見えていたらOK)
ls /dev/input

# 6. 足回りのHATが生きてるか確認 (/dev/ttyACM0など見えていればOK)
ls /dev/ttyACM*

# 7. 起動するためのディレクトリに移動
cd ~/rox_tmp_repo

# 8. Dockerを起動 & 環境をsource
./ros2_launch.bash

# 9. ロボットを実際に起動
ros2 launch launch/launch.py
```

# 本番チェック


- ロボットを起動した瞬間にLEDは赤く光っているか
- JOYモードへの移行したらLEDは緑に光るか。また、左スティックで平面移動、右スティックで旋回動作が可能か。
- 昇降機構がボタンで上下するか
- 射出、押し出しが自動で動作するか
- ライントレースモードに移行が可能か。また正常に動作するか。

# Precaution
- PS5コントローラの充電をする
- マキタバッテリーの充電をする
- ネジは外れていないか
- バッテリーは固定されているか
- ワイヤーは外れていないか

# 緊急でコードを戻したいとき

```bash
# 特定のcommitまで戻りたいとき
git reset --hard <commit_hash>
# 一つのファイルだけを戻したいとき (restoreはcheckoutでも可)
git restore <commit_hash> -- ファイル名
```

# ROS2の調子が悪い

```bash
pkill -f ros
pkill -f ros2
ros2 daemon stop
ros2 daemon start
source /opt/ros/humble/setup.bash
source install/setup.bash
...
```


