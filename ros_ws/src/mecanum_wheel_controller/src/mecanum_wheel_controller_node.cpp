#include "mecanum_wheel_controller/mecanum_wheel_controller.hpp"

MecanumWheelControllerNode::MecanumWheelControllerNode()
    : Node("mecanum_wheel_controller"), motor_controller_(this->get_logger()) {
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

MecanumWheelControllerNode::~MecanumWheelControllerNode() {
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

void MecanumWheelControllerNode::declare_parameters() {
  this->declare_parameter<double>("wheel_radius", 0.05);
  this->declare_parameter<double>("wheel_base_x", 0.2);
  this->declare_parameter<double>("wheel_base_y", 0.2);
  this->declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
  this->declare_parameter<int>("baud_rate", 115200);
  this->declare_parameter<std::vector<int64_t>>("motor_ids", {1, 2, 3, 4});
  this->declare_parameter<int>("cmd_vel_timeout_ms", 500);  // Timeout for cmd_vel in ms
}

void MecanumWheelControllerNode::get_parameters() {
  wheel_radius_ = this->get_parameter("wheel_radius").as_double();
  wheel_base_x_ = this->get_parameter("wheel_base_x").as_double();
  wheel_base_y_ = this->get_parameter("wheel_base_y").as_double();
  serial_port_ = this->get_parameter("serial_port").as_string();
  baud_rate_ = this->get_parameter("baud_rate").as_int();
  cmd_vel_timeout_ms_ = this->get_parameter("cmd_vel_timeout_ms").as_int();

  auto motor_ids_int64 = this->get_parameter("motor_ids").as_integer_array();
  motor_ids_.assign(motor_ids_int64.begin(), motor_ids_int64.end());
}

void MecanumWheelControllerNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  vx_.store(msg->linear.x);
  vy_.store(msg->linear.y);
  wz_.store(msg->angular.z);

  last_subscription_time_.store(std::chrono::steady_clock::now());
}

void MecanumWheelControllerNode::brake_service_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
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

void MecanumWheelControllerNode::timer_send_velocity_callback() {
  // Check if we have received a cmd_vel message in the last 500 milliseconds
  auto now = std::chrono::steady_clock::now();
  auto last_time = last_subscription_time_.load();
  if (now - last_time > std::chrono::milliseconds(cmd_vel_timeout_ms_)) {
    // Stop motors by sending zero velocity commands
    stop_all_motors();
    return;
  }

  // Mecanum wheel kinematics with gain adjustment
  const double gain = 1.0;  // 必要に応じて調整: 高くするとより敏感になる
  const double vx = gain * vx_.load();
  const double vy = gain * vy_.load();
  const double wz = gain * wz_.load();

  const double lxy_sum = wheel_base_x_ + wheel_base_y_;
  const double rad_to_rpm = 60.0 / (2.0 * M_PI);

  // 標準メカナムホイール運動学（物理的配置に合わせる）
  const double wheel_front_left_vel = (vx - vy - lxy_sum * wz) / wheel_radius_;
  const double wheel_front_right_vel = (vx + vy + lxy_sum * wz) / wheel_radius_;
  const double wheel_rear_left_vel = (vx + vy - lxy_sum * wz) / wheel_radius_;
  const double wheel_rear_right_vel = (vx - vy + lxy_sum * wz) / wheel_radius_;

  // Convert to RPM and send to motors
  int16_t rpm_front_left = static_cast<int16_t>(wheel_front_left_vel * rad_to_rpm);
  int16_t rpm_front_right = static_cast<int16_t>(wheel_front_right_vel * rad_to_rpm * -1);
  int16_t rpm_rear_left = static_cast<int16_t>(wheel_rear_left_vel * rad_to_rpm);
  int16_t rpm_rear_right = static_cast<int16_t>(wheel_rear_right_vel * rad_to_rpm * -1);

  // モーター値の詳細ログ（前後移動とゼロ速度変化を監視）
  static int16_t prev_fl = 0, prev_fr = 0, prev_rl = 0, prev_rr = 0;
  bool velocity_changed = (rpm_front_left != prev_fl || rpm_front_right != prev_fr || 
                          rpm_rear_left != prev_rl || rpm_rear_right != prev_rr);
  
  if (velocity_changed || std::abs(vx) > 0.1 || std::abs(vy) > 0.1) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                         "MOTOR DETAIL: vx=%.3f,vy=%.3f,wz=%.3f -> FL=%d,FR=%d,RL=%d,RR=%d", 
                         vx, vy, wz, rpm_front_left, rpm_front_right, rpm_rear_left, rpm_rear_right);
  }
  
  prev_fl = rpm_front_left; prev_fr = rpm_front_right; 
  prev_rl = rpm_rear_left; prev_rr = rpm_rear_right;

  // 従来の回転ログ  
  if (std::abs(wz) > 0.1) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "MOTOR CMD: wz=%.3f -> FL=%d,FR=%d,RL=%d,RR=%d (%s rotation)", 
                         wz, rpm_front_left, rpm_front_right, rpm_rear_left, rpm_rear_right,
                         (wz > 0) ? "LEFT" : "RIGHT");
  }

  // 並列送信でタイミングずれを防止
  std::vector<std::pair<uint8_t, int16_t>> commands = {{motor_ids_[0], rpm_front_left},
                                                       {motor_ids_[1], rpm_front_right},
                                                       {motor_ids_[2], rpm_rear_left},
                                                       {motor_ids_[3], rpm_rear_right}};

  motor_controller_.send_velocity_commands_parallel(commands, static_cast<bool>(brake_));
}

void MecanumWheelControllerNode::stop_all_motors() {
  std::vector<std::pair<uint8_t, int16_t>> stop_commands;
  for (const auto& id : motor_ids_) {
    stop_commands.emplace_back(id, 0);
  }
  motor_controller_.send_velocity_commands_sequential(stop_commands, false);
}