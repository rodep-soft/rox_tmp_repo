#include <atomic>
#include <bit>
#include <boost/asio.hpp>
#include <cmath>
#include <future>
#include <thread>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <string>
#include <vector>

// Deprecated function for CRC8 calculation
// // A simple CRC8 calculator for motor communication
// uint8_t calc_crc8_maxim(const std::vector<uint8_t>& data)
// {
//   uint8_t crc = 0x00;
//   const uint8_t polynomial = 0x31;
//   for (uint8_t byte : data) {
//     crc ^= byte;
//     for (int i = 0; i < 8; ++i) {
//       if (crc & 0x80) {
//         crc = (crc << 1) ^ polynomial;
//       } else {
//         crc <<= 1;
//       }
//     }
//   }
//   return crc;
// }

// Robust version
uint8_t calc_crc8_maxim(const std::vector<uint8_t>& data) {
  uint8_t crc = 0x00;  // 初期値  (一般的なMaxim CRCの標準)

  const uint8_t reflected_polynomial = 0x8C;

  // データバイトを一つずつ処理
  for (size_t i = 0; i < data.size(); i++) {  // DATA[0]~DATA[8]まで、合計9バイト
    crc ^= data[i];                           // 現在のバイトとCRCレジスタをXOR

    // 各バイトの8ビットを処理 (LSB First)
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (crc & 0x01) {                           // 最下位ビットが1の場合
        crc = (crc >> 1) ^ reflected_polynomial;  // 右シフトして多項式とXOR
      } else {
        crc >>= 1;  // 最下位ビットが0の場合、単に右シフト
      }
    }
  }
  return crc;
}

class MotorController {
 public:
  MotorController(rclcpp::Logger logger) : serial_port_(io_context_), logger_(logger), 
                                           buffer_(), running_(false) {}

  ~MotorController() {
    if (running_) {
      running_ = false;
      io_context_.stop();
      if (io_thread_.joinable()) {
        io_thread_.join();
      }
    }
  }

  bool init_port(const std::string& port_name, int baud_rate) {
    try {
      serial_port_.open(port_name);
      serial_port_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
      serial_port_.set_option(boost::asio::serial_port_base::character_size(8));
      serial_port_.set_option(boost::asio::serial_port_base::flow_control(
          boost::asio::serial_port_base::flow_control::none));
      serial_port_.set_option(
          boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
      serial_port_.set_option(
          boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
      
      // 非同期読み取り開始
      running_ = true;
      start_async_read();
      
      // io_contextを別スレッドで実行
      io_thread_ = std::thread([this]() {
        io_context_.run();
      });
      
    } catch (const std::exception& e) {
      RCLCPP_ERROR(logger_, "Failed to open serial port %s: %s", port_name.c_str(), e.what());
      return false;
    }
    return true;
  }

  void send_velocity_command(uint8_t motor_id, int16_t rpm, bool brake = false) {
    std::vector<uint8_t> data;

    data.push_back(static_cast<uint8_t>(motor_id & 0xFF));
    data.push_back(0x64);  // DDSM115 velocity command
    uint16_t val_u16 = std::bit_cast<uint16_t>(rpm);  // 符号付きを符号なしに変換

    data.push_back(static_cast<uint8_t>((val_u16 >> 8) & 0xFF));  // Highバイト
    data.push_back(static_cast<uint8_t>(val_u16 & 0xFF));         // Lowバイト

    data.push_back(0x00);  // Acceleration time (0 = default)
    data.push_back(0x00);  // Reserved
    data.push_back(0x00);  // Reserved  
    if (brake) {
      data.push_back(0xFF);  // Brake command
    } else {
      data.push_back(0x00);
    }
    data.push_back(0x00);  // Reserved
    data.push_back(calc_crc8_maxim(data));  // CRC8

    try {
      // コマンド送信
      boost::asio::write(serial_port_, boost::asio::buffer(data, data.size()));
      
      // フィードバック待ち（参考コードのアプローチ）
      wait_for_feedback_response(motor_id);
      
    } catch (const std::exception& e) {
      RCLCPP_ERROR(logger_, "Failed to communicate with motor %d: %s", motor_id, e.what());
    }
  }

  void clear_serial_buffer() {
    try {
      // バッファクリアのため短時間で複数回読み取り試行
      for (int i = 0; i < 5; ++i) {
        try {
          std::vector<uint8_t> buffer(64);
          boost::system::error_code error;
          
          std::size_t bytes_read = boost::asio::read(
            serial_port_,
            boost::asio::buffer(buffer),
            boost::asio::transfer_at_least(1),
            error
          );
          
          if (!error && bytes_read > 0) {
            RCLCPP_DEBUG(logger_, "Cleared %zu bytes from serial buffer", bytes_read);
          } else {
            break;  // 読み取るデータがない場合は終了
          }
        } catch (const std::exception&) {
          break;  // エラーまたはタイムアウトで終了
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    } catch (const std::exception& e) {
      RCLCPP_DEBUG(logger_, "Error clearing serial buffer: %s", e.what());
    }
  }

  // 参考コードベースの非同期読み取り
  void start_async_read() {
    if (!running_) return;

    serial_port_.async_read_some(boost::asio::buffer(read_buf_),
      [this](boost::system::error_code ec, std::size_t length) {
        if (!ec && length > 0) {
          buffer_.insert(buffer_.end(), read_buf_.begin(), read_buf_.begin() + length);
          parse_buffer();
          start_async_read();
        } else {
          if (ec) {
            RCLCPP_DEBUG(logger_, "Read error: %s", ec.message().c_str());
          }
        }
      });
  }

  void parse_buffer() {
    // 10バイトパケット単位でチェック
    while (buffer_.size() >= 10) {
      // パケット開始候補を探す(IDは1~4の範囲)
      size_t pos = 0;
      for (; pos <= buffer_.size() - 10; ++pos) {
        if (buffer_[pos] >= 1 && buffer_[pos] <= 4) {
          // CRC8チェック
          std::vector<uint8_t> packet_data(buffer_.begin() + pos, buffer_.begin() + pos + 9);
          uint8_t crc_calculated = calc_crc8_maxim(packet_data);
          uint8_t crc_received = buffer_[pos + 9];
          if (crc_calculated == crc_received) {
            break; // 正しいパケット発見
          }
        }
      }

      if (pos > 0) {
        // 不正な先頭バイトがあれば捨てる
        buffer_.erase(buffer_.begin(), buffer_.begin() + pos);
      }

      if (buffer_.size() < 10) break;

      // 10バイトパケット取り出し
      std::vector<uint8_t> packet(buffer_.begin(), buffer_.begin() + 10);

      // パースして保存
      process_feedback_packet(packet);

      // パケット消費
      buffer_.erase(buffer_.begin(), buffer_.begin() + 10);
    }
  }

  void process_feedback_packet(const std::vector<uint8_t>& packet) {
    uint8_t motor_id = packet[0];
    // uint8_t mode_value = packet[1];
    // int16_t torque_current = (static_cast<int16_t>(packet[2]) << 8) | packet[3];
    int16_t velocity = (static_cast<int16_t>(packet[4]) << 8) | packet[5];
    // uint16_t position = (static_cast<uint16_t>(packet[6]) << 8) | packet[7];
    // uint8_t error_code = packet[8];

    // モーターIDと速度のみを出力
    RCLCPP_INFO(logger_, "Motor ID: %d, Velocity: %d", motor_id, velocity);
    
    // フィードバック受信を記録
    feedback_received_time_ = std::chrono::steady_clock::now();
    last_motor_id_ = motor_id;
  }

  void wait_for_feedback_response(uint8_t motor_id, int timeout_ms = 20) {
    auto start_time = std::chrono::steady_clock::now();
    auto deadline = start_time + std::chrono::milliseconds(timeout_ms);
    
    // フィードバック受信をリセット
    last_motor_id_ = 0;
    
    while (std::chrono::steady_clock::now() < deadline) {
      // 少し待機してフィードバック受信をチェック
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      
      if (last_motor_id_ == motor_id) {
        // 正しいモーターIDからフィードバック受信
        RCLCPP_DEBUG(logger_, "Motor %d feedback received successfully", motor_id);
        return;
      }
    }
    
    // タイムアウト - でも動作は継続
    RCLCPP_DEBUG(logger_, "Motor %d feedback timeout, but continuing", motor_id);
  }

  // シーケンシャル版（一つずつ確実に送信）
  void send_velocity_commands_sequential(const std::vector<std::pair<uint8_t, int16_t>>& commands, bool brake = false) {
    for (const auto& [motor_id, rpm] : commands) {
      send_velocity_command(motor_id, rpm, brake);
      // 次のモーター送信前に少し待機（バス競合防止）
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  bool wait_for_motor_response(uint8_t motor_id, int timeout_ms = 50) {
    try {
      std::vector<uint8_t> response;
      response.reserve(10);  // DDSM115のレスポンスは10バイト
      
      auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
      
      while (std::chrono::steady_clock::now() < deadline) {
        // 短いタイムアウトで読み取り試行
        try {
          std::vector<uint8_t> buffer(1);  // 1バイトずつ読み取り
          boost::system::error_code error;
          
          // 短時間でタイムアウトする読み取り
          std::size_t bytes_read = boost::asio::read(
            serial_port_,
            boost::asio::buffer(buffer),
            boost::asio::transfer_exactly(1),
            error
          );
          
          if (!error && bytes_read > 0) {
            response.insert(response.end(), buffer.begin(), buffer.begin() + bytes_read);
            
            // 完全なパケット（10バイト）を受信したかチェック
            if (response.size() >= 10) {
              // パケットを解析
              if (validate_motor_response(response, motor_id)) {
                return true;  // 正常応答受信
              } else {
                // 無効なパケットの場合、先頭1バイトを削除して再試行
                if (!response.empty()) {
                  response.erase(response.begin());
                }
              }
            }
          }
        } catch (const std::exception&) {
          // 読み取りエラーまたはタイムアウトの場合は続行
        }
        
        // 短い間隔でポーリング
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      
      // タイムアウト
      RCLCPP_WARN(logger_, "Motor %d response timeout after %d ms", motor_id, timeout_ms);
      return false;
      
    } catch (const std::exception& e) {
      RCLCPP_ERROR(logger_, "Failed to read motor %d response: %s", motor_id, e.what());
      return false;
    }
  }

  bool validate_motor_response(const std::vector<uint8_t>& response, uint8_t expected_id) {
    if (response.size() < 10) {
      return false;
    }
    
    // DDSM115 レスポンス形式:
    // DATA[0]: ID
    // DATA[1]: Mode value  
    // DATA[2]: Torque current high 8 bits
    // DATA[3]: Torque current low 8 bits
    // DATA[4]: Velocity high 8 bits
    // DATA[5]: Velocity low 8 bits
    // DATA[6]: Position high 8 bits
    // DATA[7]: Position low 8 bits
    // DATA[8]: Error code
    // DATA[9]: CRC8
    
    // IDチェック
    if (response[0] != expected_id) {
      RCLCPP_DEBUG(logger_, "Motor ID mismatch: expected %d, got %d", expected_id, response[0]);
      return false;
    }
    
    // CRC8チェック
    std::vector<uint8_t> data_for_crc(response.begin(), response.begin() + 9);
    uint8_t calculated_crc = calc_crc8_maxim(data_for_crc);
    
    if (calculated_crc != response[9]) {
      RCLCPP_DEBUG(logger_, "CRC mismatch for motor %d: expected %02X, got %02X", 
                  expected_id, calculated_crc, response[9]);
      return false;
    }
    
    // エラーコードチェック
    uint8_t error_code = response[8];
    if (error_code != 0) {
      RCLCPP_WARN(logger_, "Motor %d error code: 0x%02X", expected_id, error_code);
      // エラーがあっても通信は成功したと見なす
    }
    
    // デバッグ情報：速度フィードバック
    int16_t velocity_feedback = (static_cast<int16_t>(response[4]) << 8) | response[5];
    RCLCPP_DEBUG(logger_, "Motor %d velocity feedback: %d rpm", expected_id, velocity_feedback);
    
    return true;
  }

 private:
  boost::asio::io_context io_context_;
  boost::asio::serial_port serial_port_;
  rclcpp::Logger logger_;
  
  // 参考コードベースの非同期読み取り用
  std::vector<uint8_t> buffer_;
  std::array<uint8_t, 64> read_buf_;
  std::atomic<bool> running_;
  std::thread io_thread_;
  
  // フィードバック管理
  std::chrono::steady_clock::time_point feedback_received_time_;
  std::atomic<uint8_t> last_motor_id_{0};
};

class MecanumWheelControllerNode : public rclcpp::Node {
 public:
  MecanumWheelControllerNode()
      : Node("mecanum_wheel_controller_node"), motor_controller_(this->get_logger()) {
    declare_parameters();
    get_parameters();

    if (!motor_controller_.init_port(serial_port_, baud_rate_)) {
      rclcpp::shutdown();
      return;
    }

    cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", best_effort_qos,
        std::bind(&MecanumWheelControllerNode::cmd_vel_callback, this, std::placeholders::_1));

    brake_service_ = this->create_service<std_srvs::srv::SetBool>(
        "/brake", std::bind(&MecanumWheelControllerNode::brake_service_callback, this,
                            std::placeholders::_1, std::placeholders::_2));

    vx_.store(0.0);
    vy_.store(0.0);
    wz_.store(0.0);
    last_subscription_time_.store(std::chrono::steady_clock::now());
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&MecanumWheelControllerNode::timer_send_velocity_callback, this));

    RCLCPP_INFO(this->get_logger(), "Mecanum wheel controller node started.");
  }

  ~MecanumWheelControllerNode() override {
    if (timer_) {
      timer_->cancel();
    }
    cmd_vel_subscription_.reset();
    vx_.store(0.0);
    vy_.store(0.0);
    wz_.store(0.0);
    RCLCPP_INFO(this->get_logger(), "Stopping motor controller...");
    stop_all_motors();
    RCLCPP_INFO(this->get_logger(), "Motors stopped.");
    RCLCPP_INFO(this->get_logger(), "Mecanum wheel controller node shutting down.");
  }

 private:
  void declare_parameters() {
    this->declare_parameter<double>("wheel_radius", 0.05);
    this->declare_parameter<double>("wheel_base_x", 0.2);
    this->declare_parameter<double>("wheel_base_y", 0.2);
    this->declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
    this->declare_parameter<int>("baud_rate", 115200);
    this->declare_parameter<std::vector<int64_t>>("motor_ids", {1, 2, 3, 4});
    this->declare_parameter<int>("cmd_vel_timeout_ms", 500);  // Timeout for cmd_vel in ms
  }

  void get_parameters() {
    wheel_radius_ = this->get_parameter("wheel_radius").as_double();
    wheel_base_x_ = this->get_parameter("wheel_base_x").as_double();
    wheel_base_y_ = this->get_parameter("wheel_base_y").as_double();
    serial_port_ = this->get_parameter("serial_port").as_string();
    baud_rate_ = this->get_parameter("baud_rate").as_int();
    cmd_vel_timeout_ms_ = this->get_parameter("cmd_vel_timeout_ms").as_int();

    auto motor_ids_int64 = this->get_parameter("motor_ids").as_integer_array();
    motor_ids_.assign(motor_ids_int64.begin(), motor_ids_int64.end());
  }

  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    vx_.store(msg->linear.x);
    vy_.store(msg->linear.y);
    wz_.store(msg->angular.z);

    last_subscription_time_.store(std::chrono::steady_clock::now());
    // RCLCPP_INFO(this->get_logger(), "Received cmd_vel: vx=%.2f, vy=%.2f, wz=%.2f", vx_.load(),
    // vy_.load(), wz_.load());
  }

  // Service callback to handle brake requests
  void brake_service_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                              std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    if (request->data) {
      brake_ = Brake::ENGAGE;
      response->success = true;
      response->message = "Brakes engaged";
    } else {
      brake_ = Brake::NONE;
      response->success = false;
      response->message = "Brakes disengaged";
    }

    RCLCPP_INFO(this->get_logger(), "Brake service called: %s", response->message.c_str());
  }

  void timer_send_velocity_callback() {
    // Check if we have received a cmd_vel message in the last 500 milliseconds
    auto now = std::chrono::steady_clock::now();
    auto last_time = last_subscription_time_.load();
    if (now - last_time > std::chrono::milliseconds(cmd_vel_timeout_ms_)) {
      // RCLCPP_WARN(this->get_logger(),
      //             "No cmd_vel message received in the last %d milliseconds. Stopping motors.",
      //             cmd_vel_timeout_ms_);
      // Stop motors by sending zero velocity commands
      stop_all_motors();
      return;
    }

    // Mecanum wheel kinematics
    // const double vx = vx_.load();
    // const double vy = vy_.load();
    // const double wz = wz_.load();

    // 感度調整のためのゲイン適用
    const double gain = 1.0;  // 必要に応じて調整: 高くするとより敏感になる
    const double vx = gain * vx_.load();
    const double vy = gain * vy_.load();
    const double wz = gain * wz_.load();

    const double lxy_sum = wheel_base_x_ + wheel_base_y_;
    const double rad_to_rpm = 60.0 / (2.0 * M_PI);

    const double wheel_front_left_vel = (vx - vy - lxy_sum * wz) / wheel_radius_;
    const double wheel_front_right_vel = (vx + vy + lxy_sum * wz) / wheel_radius_;
    const double wheel_rear_left_vel = (vx + vy - lxy_sum * wz) / wheel_radius_;
    const double wheel_rear_right_vel = (vx - vy + lxy_sum * wz) / wheel_radius_;

    // Convert to RPM and send to motors
    int16_t rpm_front_left = static_cast<int16_t>(wheel_front_left_vel * rad_to_rpm);
    int16_t rpm_front_right = static_cast<int16_t>(wheel_front_right_vel * rad_to_rpm * -1);
    int16_t rpm_rear_left = static_cast<int16_t>(wheel_rear_left_vel * rad_to_rpm);
    int16_t rpm_rear_right = static_cast<int16_t>(wheel_rear_right_vel * rad_to_rpm * -1);

    // RCLCPP_INFO(this->get_logger(), "RPM values: FL=%d, FR=%d, RL=%d, RR=%d", rpm_front_left,
    // rpm_front_right, rpm_rear_left, rpm_rear_right);

    // フィードバック待ちのシーケンシャル送信
    std::vector<std::pair<uint8_t, int16_t>> commands = {
        {motor_ids_[0], rpm_front_left},
        {motor_ids_[1], rpm_front_right},
        {motor_ids_[2], rpm_rear_left},
        {motor_ids_[3], rpm_rear_right}
    };
    
    motor_controller_.send_velocity_commands_sequential(commands, static_cast<bool>(this->brake_));
  }

  void stop_all_motors() {
    std::vector<std::pair<uint8_t, int16_t>> stop_commands;
    for (const auto& id : motor_ids_) {
      stop_commands.emplace_back(id, 0);
    }
    motor_controller_.send_velocity_commands_sequential(stop_commands, false);
  }

  // ROS 2 components
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr brake_service_;

  // Motor controller instance
  MotorController motor_controller_;

  // Brake on or off
  enum class Brake { NONE = 0, ENGAGE = 1 };

  Brake brake_ = Brake::NONE;

  // Parameters
  double wheel_radius_;
  double wheel_base_x_;
  double wheel_base_y_;
  std::atomic<double> vx_, vy_, wz_;  // Current velocities

  std::atomic<std::chrono::time_point<std::chrono::steady_clock>> last_subscription_time_;
  std::string serial_port_;
  int baud_rate_;
  std::vector<int> motor_ids_;
  int cmd_vel_timeout_ms_;

  rclcpp::TimerBase::SharedPtr timer_;

  const rclcpp::QoS reliable_qos = rclcpp::QoS(1).reliable();
  const rclcpp::QoS best_effort_qos = rclcpp::QoS(10).best_effort();
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MecanumWheelControllerNode>());
  rclcpp::shutdown();
  return 0;
}
