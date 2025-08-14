#include "mecanum_wheel_controller/mecanum_wheel_controller.hpp"

MecanumWheelControllerNode::MecanumWheelControllerNode()
    : Node("mecanum_wheel_controller"), motor_controller_(this->get_logger()),
      x_(0.0), y_(0.0), theta_(0.0) {
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

  // Initialize odometry publisher
  odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/wheel/odom", 10);
  last_odom_time_ = std::chrono::steady_clock::now();

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
  this->declare_parameter<int>("cmd_vel_timeout_ms", 500);
  this->declare_parameter<std::vector<int64_t>>("motor_ids", {1, 2, 3, 4});
  
  // Motor hardware correction factors
  this->declare_parameter<double>("motor_correction_fl", 1.0);
  this->declare_parameter<double>("motor_correction_fr", 1.0);
  this->declare_parameter<double>("motor_correction_rl", 1.0);
  this->declare_parameter<double>("motor_correction_rr", 1.0);
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
  
  // Get motor correction factors
  motor_correction_fl_ = this->get_parameter("motor_correction_fl").as_double();
  motor_correction_fr_ = this->get_parameter("motor_correction_fr").as_double();
  motor_correction_rl_ = this->get_parameter("motor_correction_rl").as_double();
  motor_correction_rr_ = this->get_parameter("motor_correction_rr").as_double();
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

  // Convert to RPM with hardware correction factors applied
  int16_t rpm_front_left = static_cast<int16_t>(wheel_front_left_vel * rad_to_rpm * motor_correction_fl_);
  int16_t rpm_front_right = static_cast<int16_t>(wheel_front_right_vel * rad_to_rpm * -1 * motor_correction_fr_);
  int16_t rpm_rear_left = static_cast<int16_t>(wheel_rear_left_vel * rad_to_rpm * motor_correction_rl_);
  int16_t rpm_rear_right = static_cast<int16_t>(wheel_rear_right_vel * rad_to_rpm * -1 * motor_correction_rr_);

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

  // 逐次送信に戻す（並列送信は効果が薄いため）
  std::vector<std::pair<uint8_t, int16_t>> commands = {{motor_ids_[0], rpm_front_left},
                                                       {motor_ids_[1], rpm_front_right},
                                                       {motor_ids_[2], rpm_rear_left},
                                                       {motor_ids_[3], rpm_rear_right}};

  motor_controller_.send_velocity_commands_sequential(commands, static_cast<bool>(brake_));
  
  // Publish wheel odometry based on actually sent RPM values
  publish_wheel_odometry(vx, vy, wz, rpm_front_left, rpm_front_right, rpm_rear_left, rpm_rear_right);
}

void MecanumWheelControllerNode::publish_wheel_odometry(double vx, double vy, double wz, 
                                                        int16_t rpm_sent_fl, int16_t rpm_sent_fr, 
                                                        int16_t rpm_sent_rl, int16_t rpm_sent_rr) {
  auto current_time = std::chrono::steady_clock::now();
  double dt = std::chrono::duration<double>(current_time - last_odom_time_).count();
  last_odom_time_ = current_time;

  // 実際のモーターRPMフィードバックを取得
  int16_t rpm_fl = motor_controller_.get_motor_rpm_feedback(1);  // Front Left
  int16_t rpm_fr = motor_controller_.get_motor_rpm_feedback(2);  // Front Right  
  int16_t rpm_rl = motor_controller_.get_motor_rpm_feedback(3);  // Rear Left
  int16_t rpm_rr = motor_controller_.get_motor_rpm_feedback(4);  // Rear Right

  // フィードバックが新しいかチェック
  bool has_valid_feedback = motor_controller_.has_recent_feedback(1, 200) &&
                            motor_controller_.has_recent_feedback(2, 200) &&
                            motor_controller_.has_recent_feedback(3, 200) &&
                            motor_controller_.has_recent_feedback(4, 200);

  double actual_vx, actual_vy, actual_wz;
  
  if (has_valid_feedback) {
    // RPMを角速度に変換（forward kinematicsと同じ符号処理を適用）
    const double rpm_to_rad = 2.0 * M_PI / 60.0;
    
    // Forward kinematicsで使用している符号処理と同じにする
    // 右側モーターは-1を掛けて送信しているので、フィードバックでも逆変換
    double rpm_fl_corrected = rpm_fl / motor_correction_fl_;
    double rpm_fr_corrected = -rpm_fr / motor_correction_fr_;  // 右側は符号を戻す
    double rpm_rl_corrected = rpm_rl / motor_correction_rl_; 
    double rpm_rr_corrected = -rpm_rr / motor_correction_rr_;  // 右側は符号を戻す
    
    // RPMの絶対値が小さい場合は0とみなす（ノイズ除去）
    const int16_t rpm_threshold = 5;
    double wheel_fl_vel = (std::abs(rpm_fl_corrected) > rpm_threshold) ? rpm_fl_corrected * rpm_to_rad : 0.0;
    double wheel_fr_vel = (std::abs(rpm_fr_corrected) > rpm_threshold) ? rpm_fr_corrected * rpm_to_rad : 0.0;
    double wheel_rl_vel = (std::abs(rpm_rl_corrected) > rpm_threshold) ? rpm_rl_corrected * rpm_to_rad : 0.0;
    double wheel_rr_vel = (std::abs(rpm_rr_corrected) > rpm_threshold) ? rpm_rr_corrected * rpm_to_rad : 0.0;
    
    // メカナムホイール逆運動学でロボット速度を計算（forward kinematicsと対応）
    const double lxy_sum = wheel_base_x_ + wheel_base_y_;
    actual_vx = wheel_radius_ * 0.25 * (wheel_fl_vel + wheel_fr_vel + wheel_rl_vel + wheel_rr_vel);
    actual_vy = wheel_radius_ * 0.25 * (-wheel_fl_vel + wheel_fr_vel + wheel_rl_vel - wheel_rr_vel);
    actual_wz = wheel_radius_ * 0.25 * (-wheel_fl_vel + wheel_fr_vel - wheel_rl_vel + wheel_rr_vel) / lxy_sum;
  } else {
    // フィードバックが古い場合はコマンド値を使用（フォールバック）
    actual_vx = vx;
    actual_vy = vy;
    actual_wz = wz;
  }
  
  // ロボットフレームからワールドフレームへの座標変換で位置を積分
  double delta_x = (actual_vx * std::cos(theta_) - actual_vy * std::sin(theta_)) * dt;
  double delta_y = (actual_vx * std::sin(theta_) + actual_vy * std::cos(theta_)) * dt;
  double delta_theta = actual_wz * dt;

  x_ += delta_x;
  y_ += delta_y;
  theta_ += delta_theta;

  // Normalize theta to [-pi, pi]
  while (theta_ > M_PI) theta_ -= 2.0 * M_PI;
  while (theta_ < -M_PI) theta_ += 2.0 * M_PI;

  // Create and publish odometry message
  auto odom_msg = nav_msgs::msg::Odometry();
  odom_msg.header.stamp = this->get_clock()->now();
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_link";

  // Position
  odom_msg.pose.pose.position.x = x_;
  odom_msg.pose.pose.position.y = y_;
  odom_msg.pose.pose.position.z = 0.0;

  // Orientation (quaternion from yaw)
  tf2::Quaternion q;
  q.setRPY(0, 0, theta_);
  odom_msg.pose.pose.orientation = tf2::toMsg(q);

  // Velocity (actual calculated velocity in robot frame)
  odom_msg.twist.twist.linear.x = actual_vx;
  odom_msg.twist.twist.linear.y = actual_vy;
  odom_msg.twist.twist.linear.z = 0.0;
  odom_msg.twist.twist.angular.x = 0.0;
  odom_msg.twist.twist.angular.y = 0.0;
  odom_msg.twist.twist.angular.z = actual_wz;

  // Set covariance based on mecanum wheel characteristics
  std::fill(odom_msg.pose.covariance.begin(), odom_msg.pose.covariance.end(), 0.0);
  std::fill(odom_msg.twist.covariance.begin(), odom_msg.twist.covariance.end(), 0.0);
  
  // Position covariance (higher uncertainty for mecanum wheels due to slippage)
  odom_msg.pose.covariance[0] = 0.05;   // x (higher due to wheel slippage)
  odom_msg.pose.covariance[7] = 0.05;   // y (mecanum wheels have lateral slippage)  
  odom_msg.pose.covariance[35] = 0.1;   // yaw (rotation accuracy depends on wheel calibration)
  
  // Velocity covariance
  odom_msg.twist.covariance[0] = 0.02;   // vx
  odom_msg.twist.covariance[7] = 0.03;   // vy (higher uncertainty for lateral movement)
  odom_msg.twist.covariance[35] = 0.05;  // wz

  odom_publisher_->publish(odom_msg);
  
  // Debug output every 2 seconds
  static auto last_debug = std::chrono::steady_clock::now();
  if (std::chrono::duration<double>(current_time - last_debug).count() > 2.0) {
    int16_t rpm_fl_fb = motor_controller_.get_motor_rpm_feedback(1);
    int16_t rpm_fr_fb = motor_controller_.get_motor_rpm_feedback(2);
    int16_t rpm_rl_fb = motor_controller_.get_motor_rpm_feedback(3);
    int16_t rpm_rr_fb = motor_controller_.get_motor_rpm_feedback(4);
    
    // コマンド速度との差分を計算
    double vx_diff = actual_vx - vx;
    double vy_diff = actual_vy - vy;
    double wz_diff = actual_wz - wz;
    
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "ODOM VALIDATION: pos(%.2f,%.2f,%.1f°) dt=%.3f\n"
                        "  CMD: vx=%.3f vy=%.3f wz=%.3f\n"
                        "  ODOM: vx=%.3f vy=%.3f wz=%.3f\n" 
                        "  DIFF: vx=%.3f vy=%.3f wz=%.3f\n"
                        "  RPM_SENT: FL=%d FR=%d RL=%d RR=%d\n"
                        "  RPM_FEEDBACK: FL=%d FR=%d RL=%d RR=%d valid=%s",
                        x_, y_, theta_ * 180.0 / M_PI, dt,
                        vx, vy, wz,
                        actual_vx, actual_vy, actual_wz, 
                        vx_diff, vy_diff, wz_diff,
                        rpm_sent_fl, rpm_sent_fr, rpm_sent_rl, rpm_sent_rr,
                        rpm_fl_fb, rpm_fr_fb, rpm_rl_fb, rpm_rr_fb,
                        has_valid_feedback ? "YES" : "NO");
    last_debug = current_time;
  }
}

void MecanumWheelControllerNode::calculate_robot_velocity_from_feedback(double& vx, double& vy, double& wz) {
  // メカナムホイール逆運動学：モーターフィードバックからロボット速度を計算
  const double rpm_to_rad = 2.0 * M_PI / 60.0;
  const double lxy_sum = wheel_base_x_ + wheel_base_y_;
  
  // 実際のモーターRPMから車輪角速度に変換（補正係数を考慮）
  double wheel_fl_vel = (actual_rpm_fl_.load() / motor_correction_fl_) * rpm_to_rad * wheel_radius_;
  double wheel_fr_vel = (actual_rpm_fr_.load() / (-motor_correction_fr_)) * rpm_to_rad * wheel_radius_;
  double wheel_rl_vel = (actual_rpm_rl_.load() / motor_correction_rl_) * rpm_to_rad * wheel_radius_;
  double wheel_rr_vel = (actual_rpm_rr_.load() / (-motor_correction_rr_)) * rpm_to_rad * wheel_radius_;
  
  // メカナムホイール逆運動学（forward kinematics）
  // 4つの車輪速度からロボットの並進・回転速度を計算
  vx = (wheel_fl_vel + wheel_fr_vel + wheel_rl_vel + wheel_rr_vel) / 4.0;
  vy = (-wheel_fl_vel + wheel_fr_vel + wheel_rl_vel - wheel_rr_vel) / 4.0;
  wz = (-wheel_fl_vel + wheel_fr_vel - wheel_rl_vel + wheel_rr_vel) / (4.0 * lxy_sum);
}

void MecanumWheelControllerNode::stop_all_motors() {
  std::vector<std::pair<uint8_t, int16_t>> stop_commands;
  for (const auto& id : motor_ids_) {
    stop_commands.emplace_back(id, 0);
  }
  motor_controller_.send_velocity_commands_sequential(stop_commands, false);
}