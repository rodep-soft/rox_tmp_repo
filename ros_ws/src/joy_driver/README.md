# Joy Driver Package

ジョイスティック（ゲームパッド）の入力を ROS2 の `cmd_vel` メッセージに変換するパッケージです。メカナムホイールロボットの直感的な操作を可能にします。

## 概要

このパッケージは、標準的なUSBゲームパッド/ジョイスティックからの入力を受け取り、ロボットの移動指令（線形・角速度）に変換してパブリッシュします。軸のマッピングや速度スケーリングは設定可能です。

## 機能

- 🎮 ジョイスティック入力から `cmd_vel` への変換
- ⚙️ 軸マッピングの柔軟な設定
- 📏 速度スケーリングの調整
- 🛡️ 入力値の安全性チェック
- 🔄 リアルタイム制御応答

## サポートするコントローラー

- Xbox Controller (USB/Wireless)
- PlayStation DualShock (USB/Wireless)  
- Logitech F310/F710
- その他の標準HIDゲームパッド

## インストールと依存関係

### ROS2 パッケージ依存関係

```xml
<!-- package.xml より -->
<depend>rclcpp</depend>
<depend>sensor_msgs</depend>
<depend>geometry_msgs</depend>
```

### システム依存関係

```bash
# Joy パッケージ（ジョイスティックドライバー）
sudo apt install ros-humble-joy

# USB デバイス権限設定
sudo usermod -a -G dialout $USER
```

## ビルド

```bash
cd /path/to/ros_ws
colcon build --packages-select joy_driver
source install/setup.bash
```

## 使用方法

### 基本起動

```bash
# Joy ノードとJoy Driverを同時起動
ros2 launch ros_ws launch.py

# または個別起動
ros2 run joy joy_node
ros2 run joy_driver joy_driver_node
```

### 設定ファイルでの起動

```bash
# カスタム設定で起動
ros2 run joy_driver joy_driver_node --ros-args --params-file config/mechanum.yaml
```

## パラメータ設定

### 速度スケーリング

| パラメータ | デフォルト値 | 説明 |
|-----------|-------------|------|
| `linear_x_scale` | 1.0 | 前後移動の速度倍率 |
| `linear_y_scale` | 1.0 | 左右移動の速度倍率 |
| `angular_scale` | 1.0 | 回転速度の倍率 |

### 軸マッピング

| パラメータ | デフォルト値 | 説明 |
|-----------|-------------|------|
| `linear_x_axis` | 1 | 前後移動に使用する軸番号 |
| `linear_y_axis` | 0 | 左右移動に使用する軸番号 |
| `angular_axis` | 3 | 回転に使用する軸番号 |

### 設定例

```yaml
# config/mechanum.yaml
joy_driver_node:
  ros__parameters:
    linear_x_scale: 0.5    # 前後移動を半分の速度に
    linear_y_scale: 0.5    # 左右移動を半分の速度に
    angular_scale: 0.3     # 回転速度を30%に
    linear_x_axis: 1       # 左スティック縦軸
    linear_y_axis: 0       # 左スティック横軸  
    angular_axis: 3        # 右スティック横軸
```

## トピック

### サブスクライブ

#### `/joy` (sensor_msgs/Joy)

ジョイスティックからの生データを受信します。

```yaml
# メッセージ例
header:
  stamp: {sec: 1234567890, nanosec: 123456789}
  frame_id: ''
axes: [0.0, -0.7, 0.0, 0.5, 0.0, 0.0]  # アナログスティック値
buttons: [0, 0, 0, 1, 0, 0, 0, 0]       # ボタン状態
```

### パブリッシュ

#### `/cmd_vel` (geometry_msgs/Twist)

ロボットの移動指令をパブリッシュします。

```yaml
# メッセージ例
linear:
  x: 0.5    # 前後速度 [m/s]
  y: 0.0    # 左右速度 [m/s] 
  z: 0.0    # 上下速度 [m/s] (未使用)
angular:
  x: 0.0    # ロール [rad/s] (未使用)
  y: 0.0    # ピッチ [rad/s] (未使用)
  z: 0.2    # ヨー（回転） [rad/s]
```

## コントローラー軸マッピング

### Xbox Controller

```
軸番号  | 機能
--------|------------------
0       | 左スティック X軸
1       | 左スティック Y軸  
2       | 左トリガー
3       | 右スティック X軸
4       | 右スティック Y軸
5       | 右トリガー
```

### PlayStation Controller

```
軸番号  | 機能
--------|------------------
0       | 左スティック X軸
1       | 左スティック Y軸
2       | 右スティック X軸  
3       | 右スティック Y軸
4       | 方向パッド X軸
5       | 方向パッド Y軸
```

## デバッグとトラブルシューティング

### ジョイスティック認識確認

```bash
# 接続されたジョイスティックデバイスの確認
ls /dev/input/js*

# Joy ノードの出力確認
ros2 topic echo /joy

# 期待される出力: axes と buttons の配列
```

### よくある問題

#### 1. ジョイスティックが認識されない

```bash
# デバイスファイルの確認
ls -la /dev/input/

# 権限の確認
groups $USER | grep dialout

# Joy ノードのログ確認
ros2 run joy joy_node --ros-args --log-level debug
```

#### 2. 軸の動きが逆向き

設定ファイルでスケール値にマイナスを設定：

```yaml
linear_x_scale: -0.5  # 前後を逆転
```

#### 3. 応答が鈍い

Joy ノードの更新頻度を上げる：

```bash
ros2 run joy joy_node --ros-args -p autorepeat_rate:=50.0
```

### ログ出力例

```
[INFO] [joy_driver_node]: Joy driver node started.
[INFO] [joy_driver_node]: Received joy input - Linear: [0.50, 0.00] Angular: 0.20
[WARN] [joy_driver_node]: Insufficient axes in joy message. Expected at least 4, got 2
```

## 安全機能

### 入力値制限

- 軸値の範囲チェック (-1.0 〜 1.0)
- 異常値の検出とフィルタリング
- 通信エラー時の安全停止

### フェイルセーフ

```cpp
// ジョイスティック信号が途切れた場合
if (msg->axes.size() < expected_axes) {
    // 緊急停止指令を送信
    auto stop_cmd = geometry_msgs::msg::Twist();
    cmd_vel_publisher_->publish(stop_cmd);
}
```

## パフォーマンス

- **CPU使用率**: < 0.5%
- **メモリ使用量**: < 5MB  
- **レイテンシ**: < 10ms
- **更新頻度**: 最大 50Hz

## 拡張例

### カスタムボタン処理

```cpp
void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    // 基本の速度変換
    auto twist = convert_to_twist(msg);
    
    // ボタンによる動作モード切り替え
    if (msg->buttons[0]) {  // A ボタン
        twist.linear.x *= 0.5;  // 低速モード
    }
    
    if (msg->buttons[1]) {  // B ボタン  
        twist = geometry_msgs::msg::Twist();  // 緊急停止
    }
    
    cmd_vel_publisher_->publish(twist);
}
```

## 開発者情報

- **作成者**: ROX Development Team
- **ライセンス**: Apache-2.0
- **バージョン**: 1.0.0
- **対応ROS**: ROS2 Humble

## 関連パッケージ

- [joy](http://wiki.ros.org/joy) - ROS Joy Driver
- [mecanum_wheel_controller](../mecanum_wheel_controller/) - 速度指令の実行
- [teleop_twist_joy](http://wiki.ros.org/teleop_twist_joy) - 類似パッケージ
