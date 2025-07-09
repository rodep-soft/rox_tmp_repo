# ROX Mecanum Wheel Robot Control System

このプロジェクトは、メカナムホイールロボットの制御システムです。ROS2を使用して、ジョイスティック入力からモーター制御までの完全な制御パイプラインを提供します。

## システム概要

本システムは以下のコンポーネントで構成されています：

### 1. 🎮 Joy Driver Package
- ジョイスティック入力を`cmd_vel`（速度指令）に変換
- 軸のマッピングとスケーリングの設定が可能
- デッドゾーン処理とスムージング機能

### 2. 🚗 Mecanum Wheel Controller Package  
- `cmd_vel`を個別のホイール速度に変換
- メカナムホイールの運動学を実装
- シリアル通信でモーターコントローラーに速度指令を送信
- CRC8チェックサムによる通信エラー検出

### 3. 🌈 Color Sensor Package
- TCS34725カラーセンサーからRGB値を読み取り
- ROSトピックとして色情報をパブリッシュ
- 色に基づいた自律動作のトリガーとして使用可能

### 4. 🐳 Docker Environment
- 完全なROS2開発環境
- シリアル通信とUSBデバイスのサポート
- 簡単なデプロイメントと開発環境の構築

## 特徴

- ✅ 全方向移動制御（前後、左右ストレーフ、回転）
- ✅ パラメータベースの設定システム
- ✅ リアルタイム色検出機能
- ✅ 堅牢なシリアル通信プロトコル
- ✅ Launch fileによる簡単起動
- ✅ 詳細なログ出力とデバッグ機能

## 開発ルール

### Precaution
- 絶対にbranchを切って作業すること。
  - 自分がわかりやすい名前なら何でも良いが、bugfix/...やfeature/...など、prefixをつけたbranch名だとより良い
- mainにマージする際は、必ずビルドと動作確認を行い、pull requestを出す
  - 他人に差分をチェックしてもらうこと。基本的に勝手にmergeしない。
- コードのテストをする際はDockerを使うこと

## クイックスタート

### 1. Docker環境の起動
```bash
docker-compose up -d
docker exec -it rox_container bash
```

### 2. ワークスペースのビルド
```bash
cd /ros_ws
colcon build
source install/setup.bash
```

### 3. システムの起動
```bash
# ジョイスティック制御付きで起動
ros2 launch ros_ws launch.py

# ジョイスティック無しで起動（手動cmd_vel送信用）
ros2 launch ros_ws launch_without_joy.py

# カラーセンサーの起動
ros2 run color_sensor color_publisher
```

## ディレクトリ構造

```
docker/
├── docker-compose.yml          # Docker Compose設定
├── README.md                  # このファイル
├── resources/                 # 開発用リソース
└── ros_ws/                    # ROS2ワークスペース
    ├── src/
    │   ├── color_sensor/      # カラーセンサーパッケージ
    │   ├── joy_driver/        # ジョイスティックドライバー
    │   └── mecanum_wheel_controller/ # メカナムホイール制御
    ├── config/               # 設定ファイル
    ├── launch/              # Launch ファイル
    └── README.md           # ROS2ワークスペース詳細
```

## トラブルシューティング

### シリアル通信エラー
- `/dev/ttyACM0`デバイスの存在確認
- Dockerコンテナの権限確認
- ボーレート設定の確認

### ジョイスティック認識エラー  
- USBデバイスのマウント確認
- `ros2 topic list`でジョイスティックトピックの確認

### ビルドエラー
- 依存関係の確認
- `colcon build --cmake-clean-cache`でクリーンビルド

## 開発者情報

- **メンテナー**: ROX Development Team
- **ライセンス**: Apache-2.0
- **ROS2バージョン**: Humble
