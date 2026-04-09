# 本番環境(ラズパイ内)のディレクトリ構成

~/rox_tmp_repo以下にこのリポジトリのコードが存在

~はログイン時のホームディレクトリ

## ロボットの起動

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


# control.md - Joy-Con & Robot Control Guide

このプロジェクトにおける Nintendo Joy-Con およびコントローラーを用いたロボット操作の公式仕様書です。

## 1. ステータスカラー定義 (Status Colors)
ロボットの状態を視覚的に判別するためのカラーコードです。LED制御やGUI実装時には以下の色を適用してください。

| モード (Mode) | 意味 | イメージカラー | カラーコード | 振る舞い |
| :--- | :--- | :--- | :--- | :--- |
| **STOP** | **緊急停止 / 安全待機** | **レッド (Red)** | `#FF0000` | 全モーター出力を強制的に `0` に。初期状態。 |
| **JOY** | **アナログ移動モード** | **グリーン (Green)** | `#00FF00` | スティックによる自由な移動（比例制御）。 |
| **DPAD** | **精密操作モード** | **ブルー (Blue)** | `#0000FF` | 十字キーによる一定速度のステップ移動。 |
| **ACTION** | **上部機構動作中** | **イエロー (Yellow)** | `#FFFF00` | 昇降・アーム等の上部モーターが回転中。 |

---

## 2. ボタン・スティックマッピング (Detailed Mapping)
`joy_driver_node` で定義されている標準的な割り当てです。

### 移動制御 (Chassis Control)
- **Lスティック (上下):** 前後移動 (`linear.x`)
- **Lスティック (左右):** 横スライド (`linear.y`)
- **L2 ボタン:** 左旋回 (`angular.z`) ※ R2が押されていない時
- **R2 ボタン:** 右旋回 (`angular.z`) ※ L2が押されていない時
- **十字キー (Up/Down/Left/Right):** DPADモード時の前後左右移動（一定速度）

### モード切替 (System Mode)
- **L1 / Share (Button 4):** `JOY` モードへ（走行可能）
- **R1 / Option (Button 5):** `STOP` モードへ（即時停止・ロック）
- **PS Button / Center (Button 6):** `DPAD` モードへ（微細な位置調整用）

### 上部機構 (Upper Mechanism)
- **B / ○ ボタン (Button 1):** 上部モーター 駆動 (`Drive`)
- **X / □ ボタン (Button 2):** 上部モーター 停止 (`Stop`)

---

## 3. 運用・実装上の注意 (Mandates)
- **デッドゾーン (Deadzone):** スティックの微小な入力（0.05以下）は、ドリフト現象を防ぐためプログラム側で無視すること。
- **安全停止:** 
  - `R1` ボタン（レッド）は常に「緊急停止」として機能させ、全ての動作を上書きして停止させること。
  - コントローラーの接続が切れた場合、トピックの更新が止まったことを検知し、即座に `STOP` 状態へ移行すること。
- **スケーリング:** 移動速度は `linear_x_scale`, `linear_y_scale`, `angular_scale` のパラメータで調整し、いきなり最大速度で動かないよう制限をかけること。

---

## 4. 接続・トラブルシューティング
1. **Bluetooth接続:**
   ```bash
   bluetoothctl
   [bluetooth]# devices
   [bluetooth]# connect [MACアドレス]
   ```
2. **デバイス確認:** `ls /dev/input/js*` を実行し、`js0` や `js1` が存在することを確認してください。
3. **ノード起動:**
   ```bash
   # ジョイスティック入力を取得
   ros2 launch joy joy_node
   # 指令値に変換
   ros2 run joy_driver joy_driver_node
   ```
4. **デバッグ:** 入力が反映されない場合は `ros2 topic echo /joy` で生データを確認してください。
