# 本番環境(ラズパイ内)のディレクトリ構成

~/rox_tmp_repo以下にこのリポジトリのコードが存在

~はログイン時のホームディレクトリ

# ロボットの起動

```bash
# ラズパイが起動していることを確認し、ssh (raspberrypiはtailscale環境のみで有効。適切なipアドレスに置き換える)ユーザ名はrodep
ssh rodep@raspberrypi
# 緊急停止ボタンが上がっており、PS5コントローラが接続されていることを必ず確認する (js0, js1等が存在するか見る)
# コントローラーはPSボタンを押せば自動で接続される。
ls /dev/input
# 足回りが生きてるか確認 (HAT)
ls /dev/ttyACM*
# 起動するためのディレクトリに移動
cd ~/rox_tmp_repo
# Dockerを起動 & 環境をsource
./ros2_launch.bash
# ロボットを実際に起動
ros2 launch launch/launch.py
```

# 本番チェック


- ロボットを起動した瞬間にLEDは赤く光っているか
- JOYモードへの移行したらLEDは緑に光るか。また、左スティックで平面移動、右スティックで旋回動作が可能か。
- 昇降機構がボタンで上下するか
- 射出、押し出しが自動で動作するか
- ライントレースモードに移行が可能か。また正常に動作するか。


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


