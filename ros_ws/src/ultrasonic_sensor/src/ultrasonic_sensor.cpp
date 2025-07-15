#include <gpiod.h>

#include <chrono>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>  // sensor_msgs/Range.msg
#include <thread>

// using namespace std::chrono_literals; // using
// namespaceは関数内または特定のスコープでの使用を推奨

// GPIOチップの指定 (Raspberry Piのモデルによって変更が必要な場合があります)
// Raspberry Pi 4B以前: "gpiochip0"
// Raspberry Pi 5: "gpiochip4" (40ピンヘッダーのGPIO)
// ターミナルで 'ls /dev/gpiochip*' を実行して確認してください。
constexpr const char* CHIP_NAME = "gpiochip0";
constexpr int TRIG_LINE = 6;  // BCM6 (GPIO6)
constexpr int ECHO_LINE = 5;  // BCM5 (GPIO5)

// センサーの測定周期 (ミリ秒)
constexpr int CYCLE_MS = 500;
// エコー信号待機時のタイムアウト (ミリ秒)
constexpr int TIMEOUT_MS = 100;

// 音速 (メートル/マイクロ秒)
// 343 m/s = 0.000343 m/ms = 0.000000343 m/us
// または 34300 cm/s = 0.0343 cm/us
// 距離 = (音速 * 時間) / 2 なので、ここでは直接m/us単位で計算する
constexpr double SOUND_SPEED_M_PER_US = 0.000343;  // 343 m/s = 0.000343 m/μs

// HC-SR04の測定範囲 (メートル)
constexpr double MIN_RANGE_M = 0.02;  // 2cm
constexpr double MAX_RANGE_M = 4.0;   // 400cm

/**
 * @brief HC-SR04超音波センサーのGPIO操作をカプセル化するクラス。
 */
class GpioHcsr04 {
 private:
  gpiod_chip* chip_;
  gpiod_line* trig_;
  gpiod_line* echo_;
  bool initialized_;

 public:
  // コンストラクタ
  GpioHcsr04() : chip_(nullptr), trig_(nullptr), echo_(nullptr), initialized_(false) {}

  // デストラクタ
  ~GpioHcsr04() { cleanup(); }

  /**
   * @brief GPIOチップとラインを初期化する。
   * @return 成功した場合 true, 失敗した場合 false.
   */
  bool initialize() {
    // GPIO チップを開く
    chip_ = gpiod_chip_open_by_name(CHIP_NAME);
    if (!chip_) {
      std::cerr << "Error: Failed to open GPIO chip " << CHIP_NAME << std::endl;
      return false;
    }

    // GPIO ラインを取得
    trig_ = gpiod_chip_get_line(chip_, TRIG_LINE);
    echo_ = gpiod_chip_get_line(chip_, ECHO_LINE);

    if (!trig_ || !echo_) {
      std::cerr << "Error: Failed to get GPIO lines. TRIG_LINE: " << TRIG_LINE
                << ", ECHO_LINE: " << ECHO_LINE << std::endl;
      cleanup();  // 部分的な初期化失敗でもクリーンアップ
      return false;
    }

    // GPIO ラインを設定 (出力と入力)
    // gpiod_line_request_output は成功したらプログラム終了まで再リクエスト不要
    if (gpiod_line_request_output(trig_, "hcsr04_trig", 0) < 0) {
      std::cerr << "Error: Failed to request trigger line (GPIO" << TRIG_LINE << ") as output."
                << std::endl;
      cleanup();
      return false;
    }

    if (gpiod_line_request_input(echo_, "hcsr04_echo") < 0) {
      std::cerr << "Error: Failed to request echo line (GPIO" << ECHO_LINE << ") as input."
                << std::endl;
      cleanup();
      return false;
    }

    initialized_ = true;
    return true;
  }

  /**
   * @brief 距離を測定する。
   * @return 測定された距離 (メートル)。エラーの場合は -1.0。
   */
  double measureDistance() {
    if (!initialized_) {
      std::cerr << "Error: Sensor not initialized." << std::endl;
      return -1.0;
    }

    // トリガーパルスを送信（10μs）
    if (gpiod_line_set_value(trig_, 1) < 0) {
      std::cerr << "Error: Failed to set trigger high." << std::endl;
      return -1.0;
    }
    std::this_thread::sleep_for(std::chrono::microseconds(10));  // 10マイクロ秒パルス

    if (gpiod_line_set_value(trig_, 0) < 0) {
      std::cerr << "Error: Failed to set trigger low." << std::endl;
      return -1.0;
    }

    // エコー信号の立ち上がりエッジを待機
    auto timeout_duration = std::chrono::milliseconds(TIMEOUT_MS);
    auto start_wait_time = std::chrono::high_resolution_clock::now();

    while (gpiod_line_get_value(echo_) == 0) {
      if (std::chrono::high_resolution_clock::now() - start_wait_time > timeout_duration) {
        std::cerr << "Timeout: No echo rising edge detected." << std::endl;
        return -1.0;
      }
    }
    auto echo_start_time = std::chrono::high_resolution_clock::now();

    // エコー信号の立ち下がりエッジを待機
    while (gpiod_line_get_value(echo_) == 1) {
      if (std::chrono::high_resolution_clock::now() - echo_start_time > timeout_duration) {
        // エコー信号が長すぎる場合もタイムアウト
        std::cerr << "Timeout: Echo signal too long (no falling edge detected)." << std::endl;
        return -1.0;
      }
    }
    auto echo_end_time = std::chrono::high_resolution_clock::now();

    // 距離を計算 (メートル単位)
    auto duration_us =
        std::chrono::duration_cast<std::chrono::microseconds>(echo_end_time - echo_start_time)
            .count();
    double distance_m = duration_us * SOUND_SPEED_M_PER_US / 2.0;  // 往復なので2で割る

    // 範囲チェック
    if (distance_m > MAX_RANGE_M || distance_m < MIN_RANGE_M) {
      // 有効範囲外の距離はエラーとして扱うか、NaNなどを返すことも検討
      // ここでは -1.0 を返すが、sensor_msgs::msg::Range の range フィールドは NaN をサポート
      std::cerr << "Warning: Distance " << distance_m << "m is outside sensor range ["
                << MIN_RANGE_M << "m, " << MAX_RANGE_M << "m]." << std::endl;
      return -1.0;  // もしくは std::numeric_limits<double>::quiet_NaN();
    }

    return distance_m;
  }

 private:
  /**
   * @brief GPIOリソースを解放する。
   */
  void cleanup() {
    if (trig_ && gpiod_line_is_requested(trig_)) {
      gpiod_line_release(trig_);
    }
    if (echo_ && gpiod_line_is_requested(echo_)) {
      gpiod_line_release(echo_);
    }
    if (chip_) {
      gpiod_chip_close(chip_);
      chip_ = nullptr;
    }
    trig_ = nullptr;
    echo_ = nullptr;
    initialized_ = false;
  }
};

/**
 * @brief HC-SR04超音波センサーのROS 2ノード。
 */
class UltrasonicSensorNode : public rclcpp::Node {
 public:
  // コンストラクタ
  UltrasonicSensorNode() : Node("ultrasonic_sensor_node") {
    // GPIOセンサーの初期化
    if (!gpio_sensor_.initialize()) {
      RCLCPP_FATAL(this->get_logger(),
                   "Failed to initialize GPIO for ultrasonic sensor. Shutting down ROS node.");
      rclcpp::shutdown();  // GPIO初期化失敗時はROSノードを終了
      return;              // コンストラクタの残りの処理をスキップ
    } else {
      RCLCPP_INFO(this->get_logger(), "GPIO for ultrasonic sensor initialized successfully.");
    }

    // publisherの作成
    publisher_ = this->create_publisher<sensor_msgs::msg::Range>("/distance", 10);

    // タイマーの作成
    timer_ = this->create_wall_timer(std::chrono::milliseconds(CYCLE_MS),
                                     std::bind(&UltrasonicSensorNode::distance_callback, this));

    RCLCPP_INFO(this->get_logger(), "Ultrasonic Sensor Node has been started.");
  }

 private:
  // GpioHcsr04 クラスのインスタンス
  GpioHcsr04 gpio_sensor_;

  // publisherとtimerの宣言
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  /**
   * @brief タイマーコールバックで距離を測定し、パブリッシュする。
   */
  void distance_callback() {
    double distance_m = gpio_sensor_.measureDistance();

    // 測定エラーの場合はパブリッシュしない
    if (distance_m < 0) {  // エラー値 -1.0 をチェック
      return;
    }

    // sensor_msgs::msg::Range メッセージを作成・設定
    auto range_msg = std::make_unique<sensor_msgs::msg::Range>();

    range_msg->header.stamp = this->now();                  // 現在のROS時間
    range_msg->header.frame_id = "ultrasonic_sensor_link";  // センサーのTFフレームID、適宜変更

    range_msg->radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
    // HC-SR04の視野角はデータシートで正確な値を確認し設定してください
    // 一般的には約30度 (0.52ラジアン) 程度とされますが、センサーによって異なります
    range_msg->field_of_view = 0.52;  // 例: 約30度をラジアンで指定

    range_msg->min_range = MIN_RANGE_M;  // 最小測定距離 (メートル)
    range_msg->max_range = MAX_RANGE_M;  // 最大測定距離 (メートル)

    range_msg->range = distance_m;  // 測定された距離 (メートル)

    // メッセージをパブリッシュ
    publisher_->publish(std::move(range_msg));

    // ここでの sleep_for は不要です。タイマーが周期を制御します。
    // std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
};

/**
 * @brief メイン関数。ROS 2ノードを初期化し、スピンする。
 */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  // ROS 2ノードクラスをインスタンス化し、スピン
  rclcpp::spin(std::make_shared<UltrasonicSensorNode>());
  rclcpp::shutdown();  // ROS終了処理
  return 0;
}
