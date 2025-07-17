#include <atomic>
#include <bit>
#include <boost/asio.hpp>
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <string>
#include <vector>
#include <custom_interfaces/msg/motor_feedback.hpp>
#include <optional>
#include <array>
// #include <template>

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

uint8_t calc_crc8_maxim_for_array(const std::array<uint8_t, 9>& data) {
  uint8_t crc = 0x00;
  const uint8_t reflected_polynomial = 0x8C;

  for (size_t i = 0; i < data.size(); i++) {
    crc ^= data[i];

    for (uint8_t bit = 0; bit < 8; bit++) {
      if (crc & 0x01) {
        crc = (crc >> 1) ^ reflected_polynomial;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

template<typename Container>
uint8_t calc_crc8_maxim_temp(const Container& data) {
  uint8_t crc = 0x00;
  const uint8_t reflected_polynomial = 0x8C;

  for (const auto& byte : data) {
    crc ^= byte;

    for (uint8_t bit = 0; bit < 8; bit++) {
      if (crc & 0x01) {
        crc = (crc >> 1) ^ reflected_polynomial;
      } else {
        crc >>= 1;
      }
    }
  }

  return crc;
}

class MotorController {
 public:
  MotorController(rclcpp::Logger logger) : serial_port_(io_context_), logger_(logger) {}

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
    } catch (const std::exception& e) {
      RCLCPP_ERROR(logger_, "Failed to open serial port %s: %s", port_name.c_str(), e.what());
      return false;
    }
    return true;
  }

  std::optional<int16_t> send_velocity_command_with_array(uint8_t motor_id, int16_t rpm, bool brake = false) {
    std::array<uint8_t, 10> data;
    data[0] = static_cast<uint8_t>(motor_id & 0xFF);
    data[1] = 0x64;
    // uint16_t val_u16 = std::bit_cast<uint16_t>(rpm);
    uint16_t val_u16 = static_cast<uint16_t>(rpm);
    data[2] = static_cast<uint8_t>((val_u16 >> 8) & 0xFF);
    data[3] = static_cast<uint8_t>((val_u16) & 0xFF);
    data[4] = 0x00;
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = brake ? 0xFF : 0x00;
    data[8] = 0x00;
    data[9] = calc_crc8_maxim(std::vector<uint8_t>(data.begin(), data.begin() + 9));

    

    try {
      boost::asio::write(serial_port_, boost::asio::buffer(data, data.size()));
    } catch (const std::exception& e) {
      RCLCPP_ERROR(logger_, "Failed to write to serial port: %s", e.what());
    }

    // 一旦廃止
    // std::this_thread::sleep_for(std::chrono::milliseconds(3));

      // 読み取り
    if (auto response_opt = read_response()) {
       const auto& response = *response_opt;
      //  uint8_t id = response[0];
      //  uint8_t mode = response[1];
      //  int16_t current = static_cast<int16_t>((response[2] << 8) | response[3]);
      int16_t speed = static_cast<int16_t>((response[4] << 8) | response[5]);
      //  uint16_t position = static_cast<uint16_t>((response[6] << 8) | response[7]);
      //  uint8_t error = response[8];

      //  RCLCPP_INFO(logger_, "Motor Speed: %d", speed);
      return speed;
    } else {
      return std::nullopt; //応答がない場合
    }
    // else {
    //   RCLCPP_WARN(logger_,  "No valid response");
    // }


  }

  void send_velocity_command(uint8_t motor_id, int16_t rpm, bool brake = false) {


    std::vector<uint8_t> data;

    data.push_back(static_cast<uint8_t>(motor_id & 0xFF));
    data.push_back(0x64);
    // bit_castは本当にいるのか？
    uint16_t val_u16 = std::bit_cast<uint16_t>(rpm);  // 符号付きを符号なしに変換

    data.push_back(static_cast<uint8_t>((val_u16 >> 8) & 0xFF));  // Highバイト
    data.push_back(static_cast<uint8_t>(val_u16 & 0xFF));         // Lowバイト

    data.push_back(0x00);
    data.push_back(0x00);
    data.push_back(0x00);
    // ブレーキを設定
    if (brake) {
      data.push_back(0xFF);
    } else {
      data.push_back(0x00);
    }
    data.push_back(0x00);
    data.push_back(calc_crc8_maxim(data));

    try {
      boost::asio::write(serial_port_, boost::asio::buffer(data, data.size()));
    } catch (const std::exception& e) {
      RCLCPP_ERROR(logger_, "Failed to write to serial port: %s", e.what());
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(3));

    /*

    // 読み取り
    if (auto response_opt = read_response()) {
       const auto& response = *response_opt;
       uint8_t id = response[0];
       uint8_t mode = response[1];
       int16_t current = static_cast<int16_t>((response[2] << 8) | response[3]);
       int16_t speed = static_cast<int16_t>((response[4] << 8) | response[5]);
       uint16_t position = static_cast<uint16_t>((response[6] << 8) | response[7]);
       uint8_t error = response[8];

      //  RCLCPP_INFO(logger_, "Motor Speed: %d", speed);
      return speed;
    }  else {
      return -1;
    }
    // else {
    //   RCLCPP_WARN(logger_,  "No valid response");
    // }


    */
  }

  // モーターからのフィードバックを読み取る関数
  std::optional<std::vector<uint8_t>> read_response() {
    // Read response from the motor controller
    // 10バイトの応答を読み取る
    std::vector<uint8_t> response(10);
    try {
      boost::asio::read(serial_port_, boost::asio::buffer(response, response.size()));

      uint8_t crc = calc_crc8_maxim(std::vector<uint8_t>(response.begin(), response.begin() + 9));
      if (crc != response[9]) {
        RCLCPP_ERROR(logger_, "CRC mismatch");
        return std::nullopt;
      } 

      return response;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(logger_, "Failed to read from serial port: %s", e.what());
      return std::nullopt;
    }

  }

 private:
  boost::asio::io_context io_context_;
  boost::asio::serial_port serial_port_;
  rclcpp::Logger logger_;
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
        "/cmd_vel", 
        reliable_qos,
        std::bind(&MecanumWheelControllerNode::cmd_vel_callback, this, std::placeholders::_1));

    motor_speed_publisher_ = this->create_publisher<custom_interfaces::msg::MotorFeedback>(
        "/motor_feedback", reliable_qos);

    brake_service_ = this->create_service<std_srvs::srv::SetBool>(
        "/brake", std::bind(&MecanumWheelControllerNode::brake_service_callback, this,
                            std::placeholders::_1, std::placeholders::_2)
    );

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

  int16_t motor1_speed;
  int16_t motor2_speed;
  int16_t motor3_speed;
  int16_t motor4_speed;

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
      RCLCPP_WARN(this->get_logger(),
                  "No cmd_vel message received in the last %d milliseconds. Stopping motors.",
                  cmd_vel_timeout_ms_);
      // Stop motors by sending zero velocity commands
      stop_all_motors();
      return;
    }

    // Mecanum wheel kinematics
    // const double vx = vx_.load();
    // const double vy = vy_.load();
    // const double wz = wz_.load();

    // 感度調整のためハイパボリックタンジェントを使用
    const double gain = 1.0;  // 必要に応じて調整: 高くする
    const double linear_scale = 1.0;
    const double angular_scale = 0.5;
    const double vx = gain * std::tanh(vx_.load() / linear_scale);
    const double vy = gain * std::tanh(vy_.load() / linear_scale);
    const double wz = gain * std::tanh(wz_.load() / angular_scale);

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
    

    motor_controller_.send_velocity_command(motor_ids_[0], rpm_front_left, static_cast<bool>(this->brake_));
    motor_controller_.send_velocity_command(motor_ids_[1], rpm_front_right, static_cast<bool>(this->brake_));
    motor_controller_.send_velocity_command(motor_ids_[2], rpm_rear_left, static_cast<bool>(this->brake_));
    motor_controller_.send_velocity_command(motor_ids_[3], rpm_rear_right, static_cast<bool>(this->brake_));
    

    // auto result1 = motor_controller_.send_velocity_command(motor_ids_[0], rpm_front_left, static_cast<bool>(this->brake_));
    // auto result2 = motor_controller_.send_velocity_command(motor_ids_[1], rpm_front_right, static_cast<bool>(this->brake_));
    // auto result3 = motor_controller_.send_velocity_command(motor_ids_[2], rpm_rear_left, static_cast<bool>(this->brake_));
    // auto result4 = motor_controller_.send_velocity_command(motor_ids_[3], rpm_rear_right, static_cast<bool>(this->brake_));

    // motor1_speed = result1.value_or(-9999);
    // motor2_speed = result2.value_or(-9999);
    // motor3_speed = result3.value_or(-9999);
    // motor4_speed = result4.value_or(-9999);

    // // Publish motor speeds
    // auto motor_feedback_msg = custom_interfaces::msg::MotorFeedback();
    // motor_feedback_msg.motor1 = motor1_speed;
    // motor_feedback_msg.motor2 = motor2_speed;
    // motor_feedback_msg.motor3 = motor3_speed;
    // motor_feedback_msg.motor4 = motor4_speed;
    // motor_speed_publisher_->publish(motor_feedback_msg);
  }

  void stop_all_motors() {
    for (const auto& id : motor_ids_) {
      motor_controller_.send_velocity_command(id, 0);
    }
  }

  // ROS 2 components
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;

  rclcpp::Publisher<custom_interfaces::msg::MotorFeedback>::SharedPtr motor_speed_publisher_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr brake_service_;

  
  // Motor controller instance
  MotorController motor_controller_;

  // Brake on or off
  enum class Brake {
    NONE = 0, 
    ENGAGE = 1
  };

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
  //const rclcpp::QoS best_effort_qos = rclcpp::QoS(1).best_effort();

};


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MecanumWheelControllerNode>());
  rclcpp::shutdown();
  return 0;
}
