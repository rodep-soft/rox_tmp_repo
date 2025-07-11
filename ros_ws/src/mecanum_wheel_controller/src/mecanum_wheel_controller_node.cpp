#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <boost/asio.hpp>
#include <memory>
#include <vector>
#include <string>
#include <cmath>
#include <atomic>
#include <bit>

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
    uint8_t crc = 0x00; // 初期値  (一般的なMaxim CRCの標準)

    const uint8_t reflected_polynomial = 0x8C; 

    // データバイトを一つずつ処理
    for (size_t i = 0; i < data.size(); i++) { // DATA[0]~DATA[8]まで、合計9バイト
        crc ^= data[i]; // 現在のバイトとCRCレジスタをXOR

        // 各バイトの8ビットを処理 (LSB First)
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x01) { // 最下位ビットが1の場合
                crc = (crc >> 1) ^ reflected_polynomial; // 右シフトして多項式とXOR
            } else {
                crc >>= 1; // 最下位ビットが0の場合、単に右シフト
            }
        }
    }
    return crc;
}

class MotorController
{
public:
  MotorController(rclcpp::Logger logger) : serial_port_(io_context_), logger_(logger) {}

  bool init_port(const std::string& port_name, int baud_rate)
  {
    try {
      serial_port_.open(port_name);
      serial_port_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
      serial_port_.set_option(boost::asio::serial_port_base::character_size(8));
      serial_port_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
      serial_port_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
      serial_port_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    } catch (const std::exception& e) {
      RCLCPP_ERROR(logger_, "Failed to open serial port %s: %s", port_name.c_str(), e.what());
      return false;
    }
    return true;
  }

  void send_velocity_command(uint8_t motor_id, int16_t rpm)
  {
    // std::vector<uint8_t> command_data = {
    //   motor_id,
    //   0x64, // Command for velocity control
    //   static_cast<uint8_t>((rpm >> 8) & 0xFF), // High byte of RPM
    //   static_cast<uint8_t>(rpm & 0xFF),        // Low byte of RPM
    //   0x00, 0x00, 0x00, 0x00, 0x00 // Reserved bytes
    // };
    // command_data.push_back(calc_crc8_maxim(command_data));

    std::vector<uint8_t> data;

    data.push_back(static_cast<uint8_t>(motor_id & 0xFF));
    data.push_back(0x64);
    uint16_t val_u16 = std::bit_cast<uint16_t>(rpm); // 符号付きを符号なしに変換

    data.push_back(static_cast<uint8_t>((val_u16 >> 8) & 0xFF)); // Highバイト  
    data.push_back(static_cast<uint8_t>(val_u16 & 0xFF));        // Lowバイト

    data.push_back(0x00);
    data.push_back(0x00);
    data.push_back(0x00);
    data.push_back(0x00);
    data.push_back(0x00); 
    data.push_back(calc_crc8_maxim(data));

    try {
      boost::asio::write(serial_port_, boost::asio::buffer(data, data.size()));
    } catch (const std::exception& e) {
      RCLCPP_ERROR(logger_, "Failed to write to serial port: %s", e.what());
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

private:
  boost::asio::io_context io_context_;
  boost::asio::serial_port serial_port_;
  rclcpp::Logger logger_;
};

class MecanumWheelControllerNode : public rclcpp::Node
{
public:
  MecanumWheelControllerNode() : Node("mecanum_wheel_controller_node"), motor_controller_(this->get_logger())
  {
    declare_parameters();
    get_parameters();

    if (!motor_controller_.init_port(serial_port_, baud_rate_)) {
      rclcpp::shutdown();
      return;
    }

    cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&MecanumWheelControllerNode::cmd_vel_callback, this, std::placeholders::_1));
    
    vx_.store(0.0);
    vy_.store(0.0);
    wz_.store(0.0);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&MecanumWheelControllerNode::timer_send_velocity_callback, this));

    RCLCPP_INFO(this->get_logger(), "Mecanum wheel controller node started.");
  }

  ~MecanumWheelControllerNode() override
  {
    if (timer_) {
      timer_->cancel();
    }
    cmd_vel_subscription_.reset();
    RCLCPP_INFO(this->get_logger(), "Stopping motor controller...");
    stop_all_motors();
    RCLCPP_INFO(this->get_logger(), "Motors stopped.");
    RCLCPP_INFO(this->get_logger(), "Mecanum wheel controller node shutting down.");
  }

private:
  void declare_parameters()
  {
    this->declare_parameter<double>("wheel_radius", 0.05);
    this->declare_parameter<double>("wheel_base_x", 0.2);
    this->declare_parameter<double>("wheel_base_y", 0.2);
    this->declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
    this->declare_parameter<int>("baud_rate", 115200);
    this->declare_parameter<std::vector<int64_t>>("motor_ids", {1, 2, 3, 4});
    this->declare_parameter<int>("cmd_vel_timeout_ms", 500); // Timeout for cmd_vel in ms
  }

  void get_parameters()
  {
    wheel_radius_ = this->get_parameter("wheel_radius").as_double();
    wheel_base_x_ = this->get_parameter("wheel_base_x").as_double();
    wheel_base_y_ = this->get_parameter("wheel_base_y").as_double();
    serial_port_ = this->get_parameter("serial_port").as_string();
    baud_rate_ = this->get_parameter("baud_rate").as_int();
    cmd_vel_timeout_ms_ = this->get_parameter("cmd_vel_timeout_ms").as_int();

    auto motor_ids_int64 = this->get_parameter("motor_ids").as_integer_array();
    motor_ids_.assign(motor_ids_int64.begin(), motor_ids_int64.end());
  }
  
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    vx_.store(msg->linear.x);
    vy_.store(msg->linear.y);
    wz_.store(msg->angular.z);

    last_subscription_time_.store(std::chrono::system_clock::now());
    //RCLCPP_INFO(this->get_logger(), "Received cmd_vel: vx=%.2f, vy=%.2f, wz=%.2f", vx_.load(), vy_.load(), wz_.load());
  }

  void timer_send_velocity_callback()
  {
    // Check if we have received a cmd_vel message in the last 500 milliseconds
    auto now = std::chrono::steady_clock::now();
    auto last_time = last_subscription_time_.load();
    if (now - last_time > std::chrono::milliseconds(cmd_vel_timeout_ms_)) {
      RCLCPP_WARN(this->get_logger(), "No cmd_vel message received in the last %d milliseconds. Stopping motors.", cmd_vel_timeout_ms_);
      // Stop motors by sending zero velocity commands
      motor_controller_.send_velocity_command(motor_ids_[0], 0);
      motor_controller_.send_velocity_command(motor_ids_[1], 0);
      motor_controller_.send_velocity_command(motor_ids_[2], 0);
      motor_controller_.send_velocity_command(motor_ids_[3], 0);
      return;
    }

    // Mecanum wheel kinematics
    const double vx = vx_.load();
    const double vy = vy_.load();
    const double wz = wz_.load();
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
    
    RCLCPP_INFO(this->get_logger(), "RPM values: FL=%d, FR=%d, RL=%d, RR=%d", 
                rpm_front_left, rpm_front_right, rpm_rear_left, rpm_rear_right);
    
    motor_controller_.send_velocity_command(motor_ids_[0], rpm_front_left);
    motor_controller_.send_velocity_command(motor_ids_[1], rpm_front_right);
    motor_controller_.send_velocity_command(motor_ids_[2], rpm_rear_left);
    motor_controller_.send_velocity_command(motor_ids_[3], rpm_rear_right);
  }

  void stop_all_motors()
  {
    for (const auto& id : motor_ids_) {
      motor_controller_.send_velocity_command(id, 0);
    }
  }

  // ROS 2 components
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
  MotorController motor_controller_;

  // Parameters
  double wheel_radius_;
  double wheel_base_x_;
  double wheel_base_y_;
  std::atomic<double> vx_, vy_, wz_; // Current velocities

  std::atomic<std::chrono::time_point<std::chrono::steady_clock>> last_subscription_time_;
  std::string serial_port_;
  int baud_rate_;
  std::vector<int> motor_ids_;
  int cmd_vel_timeout_ms_;

  rclcpp::TimerBase::SharedPtr timer_;

};
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MecanumWheelControllerNode>());
  rclcpp::shutdown();
  return 0;
}
