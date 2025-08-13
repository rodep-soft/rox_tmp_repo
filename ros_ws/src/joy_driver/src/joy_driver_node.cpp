#include "joy_driver/joy_driver_node.hpp"

// Constructor
JoyDriverNode::JoyDriverNode() : Node("joy_driver_node") {
  // Declare and get parameters for velocity scaling and axis mapping
  declare_parameters();
  get_parameters();

  // Create subscription to the /joy topic
  joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", best_effort_qos,
      std::bind(&JoyDriverNode::joy_callback, this, std::placeholders::_1));

  rpy_subscription_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      "/imu/rpy", best_effort_qos,
      std::bind(&JoyDriverNode::rpy_callback, this, std::placeholders::_1));

  imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu", best_effort_qos,
      std::bind(&JoyDriverNode::imu_callback, this, std::placeholders::_1));

  // Create publisher for the /cmd_vel topic
  cmd_vel_publisher_ =
      this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", best_effort_qos);

  brake_client_ = this->create_client<std_srvs::srv::SetBool>("/brake");

  upper_publisher_ = this->create_publisher<custom_interfaces::msg::UpperMotor>("/upper_motor", 3);

  linetrace_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/is_linetrace", 10);

  mode_publisher_ = this->create_publisher<std_msgs::msg::String>("/mode", 10);

  RCLCPP_INFO(this->get_logger(), "Joy driver node started.");
}

void JoyDriverNode::declare_parameters() {
  this->declare_parameter<double>("linear_x_scale", 1.0);
  this->declare_parameter<double>("linear_y_scale", 1.0);
  this->declare_parameter<double>("angular_scale", 1.0);
  this->declare_parameter<int>("linear_x_axis", 1);  // Vertical movement
  this->declare_parameter<int>("linear_y_axis", 0);  // Horizontal movement
  this->declare_parameter<int>("angular_axis", 2);

  this->declare_parameter<double>("Kp", 0.15);  // 比例ゲイン（より控えめに）
  this->declare_parameter<double>("Ki", 0.01);  // 積分ゲイン（より控えめに）
  this->declare_parameter<double>("Kd", 0.02);  // 微分ゲイン（より控えめに）
  this->declare_parameter<double>("deadband", 0.1);   // デッドバンドを大きく（rad）
  this->declare_parameter<double>("max_angular_correction", 0.3);  // 最大補正を制限
}

void JoyDriverNode::get_parameters() {
  linear_x_scale_ = this->get_parameter("linear_x_scale").as_double();
  linear_y_scale_ = this->get_parameter("linear_y_scale").as_double();
  angular_scale_ = this->get_parameter("angular_scale").as_double();
  linear_x_axis_ = this->get_parameter("linear_x_axis").as_int();
  linear_y_axis_ = this->get_parameter("linear_y_axis").as_int();
  angular_axis_ = this->get_parameter("angular_axis").as_int();

  Kp_ = this->get_parameter("Kp").as_double();                     // 比例ゲイン
  Ki_ = this->get_parameter("Ki").as_double();                     // 積分ゲイン
  Kd_ = this->get_parameter("Kd").as_double();                     // 微分ゲイン
  deadband_ = this->get_parameter("deadband").as_double();         // デッドバンド
  max_angular_correction_ =
      this->get_parameter("max_angular_correction").as_double();   // 最大角速度補正

  // パラメータ値をログ出力して確認
  RCLCPP_INFO(
      this->get_logger(),
      "Parameters loaded: angular_axis=%d, linear_x_axis=%d, linear_y_axis=%d, angular_scale=%.2f",
      angular_axis_, linear_x_axis_, linear_y_axis_, angular_scale_);
}

// Joy main callback function
void JoyDriverNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
  auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();

  auto linetrace_msg = std::make_unique<std_msgs::msg::Bool>();

  auto mode_msg = std::make_unique<std_msgs::msg::String>();

  // だめぽ
  // linetrace_msg->data = false;

  // Ensure the message has enough axes and buttons to prevent a crash
  if (msg->axes.size() <=
      static_cast<size_t>(std::max({linear_x_axis_, linear_y_axis_, angular_axis_}))) {
    RCLCPP_WARN(this->get_logger(), "Joystick message has insufficient axes.");
    return;
  }

  if (msg->buttons.size() < 16) {  // 最大使用するボタンインデックスは15
    RCLCPP_WARN(this->get_logger(), "Joystick message has insufficient buttons (%zu < 16).",
                msg->buttons.size());
    return;
  }

  // buttons[11] == 1 && buttons[12] == 1
  // linetrace_msg->data = 1
  //

  // Mode switching logic
  bool current_linetrace_buttons = (msg->buttons[7] == 1 && msg->buttons[8] == 1);

  // ライントレースモードかどうかを変える
  if (current_linetrace_buttons && !prev_linetrace_buttons_) {
    // Toggle LINETRACE mode only on button press (not hold)
    if (mode_ == Mode::LINETRACE) {
      mode_ = Mode::JOY;
      RCLCPP_INFO(this->get_logger(), "Mode: JOY (from LINETRACE)");
    } else {
      mode_ = Mode::LINETRACE;
      RCLCPP_INFO(this->get_logger(), "Mode: LINETRACE");
    }
  }

  // もしライントレースモードでなければ、他のモードに切り替えることができる
  if (mode_ != Mode::LINETRACE) {
    if (msg->buttons[6] == 1 && mode_ != Mode::JOY) {
      mode_ = Mode::JOY;
      RCLCPP_INFO(this->get_logger(), "Mode: JOY");
    } else if (msg->buttons[15] == 1 && mode_ != Mode::STOP) {
      mode_ = Mode::STOP;
      RCLCPP_INFO(this->get_logger(), "Mode: STOP");
    } else if (msg->buttons[4] == 1 && mode_ != Mode::DPAD) {
      mode_ = Mode::DPAD;
      init_yaw_ = yaw_;
      // PID状態をリセット
      integral_error_ = 0.0;
      prev_yaw_error_ = 0.0;
      last_correction_time_ = 0.0;
      RCLCPP_INFO(this->get_logger(), "Mode: DPAD (PID Reset)");
    }
  }

  // Update previous button state
  prev_linetrace_buttons_ = current_linetrace_buttons;

  bool l2_pressed = msg->axes[4] < TRIGGER_THRESHOLD;  // L2 trigger
  bool r2_pressed = msg->axes[5] < TRIGGER_THRESHOLD;  // R2 trigger

  // Initialize the twist message
  twist_msg->linear.x = 0.0;
  twist_msg->linear.y = 0.0;
  twist_msg->angular.z = 0.0;

  // if (r2_pressed && !l2_pressed) {
  //       // R2: rotate left
  //       twist_msg->angular.z = -(msg->axes[4] - 1) / 2.0;
  //     } else if (l2_pressed && !r2_pressed) {
  //       // L2: rotate right
  //       twist_msg->angular.z = (msg->axes[5] - 1) / 2.0;
  //     }

  // switch (mode_){
  //   case Mode::JOY:
  //     if(!l2_pressed && !r2_pressed) {
  //       twist_msg->linear.x = msg->axes[linear_x_axis_] * linear_x_scale_;
  //       twist_msg->linear.y = msg->axes[linear_y_axis_] * linear_y_scale_;
  //       twist_msg->angular.z = 0.0;
  //     } else {
  //       set_angular_velocity(msg, twist_msg);
  //     }
  //     break;
  //   case Mode::DPAD:
  //     if(!l2_pressed && !r2_pressed) {
  //       twist_msg->linear.x = (msg->buttons[11] - msg->buttons[12]) * linear_x_scale_ / 2.0;
  //       twist_msg->linear.y = (msg->buttons[13] - msg->buttons[14]) * linear_y_scale_ / 2.0;
  //       twist_msg->angular.z = 0.0;  // No angular movement in DPAD mode
  //     } else {
  //       set_angular_velocity(msg, twist_msg);
  //     }
  //     break;
  //   case Mode::STOP:
  //     twist_msg->linear.x = 0.0;
  //     twist_msg->linear.y = 0.0;
  //     twist_msg->angular.z = 0.0;  // No movement in STOP mode
  //     break;
  //   default:
  // }

  // 角度差分を正しく計算（-πからπの範囲に正規化） - DPADモード専用
  // double error = yaw_ - init_yaw_;
  // // 角度の連続性を考慮した正規化
  // while (error > M_PI) error -= 2.0 * M_PI;
  // while (error < -M_PI) error += 2.0 * M_PI;

  switch (mode_) {
    case Mode::STOP:
      twist_msg->linear.x = 0.0;
      twist_msg->linear.y = 0.0;
      twist_msg->angular.z = 0.0;
      break;
    case Mode::JOY: {
      // 移動は常に有効
      twist_msg->linear.x = applyDeadzone(msg->axes[linear_x_axis_]) * linear_x_scale_;
      twist_msg->linear.y = applyDeadzone(msg->axes[linear_y_axis_]) * linear_y_scale_;

      // 動的デッドゾーン：移動中は回転のデッドゾーンを大きく、停止中も十分な値に
      bool is_moving = (std::abs(twist_msg->linear.x) > 0.1 || std::abs(twist_msg->linear.y) > 0.1);
      double angular_deadzone = is_moving ? 1.0 : 0.8;  // ジョイスティックノイズ対策で大幅に増加

      // 手動回転入力を取得
      double manual_angular = applyDeadzone(msg->axes[angular_axis_], angular_deadzone) * angular_scale_;
      
      // ジョイスティックノイズのデバッグ（頻度を大幅削減）
      if (std::abs(msg->axes[angular_axis_] * angular_scale_) > 1.0) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "JOYSTICK NOISE: raw=%.3f, final=%.3f",
                             msg->axes[angular_axis_], manual_angular);
      }
      
      // 手動回転中フラグを外部で管理
      static bool was_manual_rotating = false;
      
      // 手動回転入力がない場合、移動中にIMU補正を適用
      if (std::abs(manual_angular) < 0.01 && is_moving) {
        // 手動回転終了時の目標姿勢更新処理
        if (was_manual_rotating) {
          init_yaw_ = yaw_;  // 現在の向きを新しい基準に設定
          integral_error_ = 0.0;
          prev_yaw_error_ = 0.0;
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                               "Manual rotation ended - New target: %.1f°", init_yaw_ * 180.0 / M_PI);
          was_manual_rotating = false;
        }
        
        // 動作開始時にPID状態をリセット（切り返し対応）
        static bool was_moving = false;
        if (!was_moving) {
          integral_error_ = 0.0;
          prev_yaw_error_ = 0.0;
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                               "Movement started - PID state reset");
        }
        was_moving = true;
        
        // 現在時刻の取得
        auto now = this->get_clock()->now();
        double current_time = now.seconds();
        double dt = (last_correction_time_ > 0) ? (current_time - last_correction_time_) : 0.02;
        last_correction_time_ = current_time;
        
        // 角度誤差を計算（正規化済み）
        double error = normalizeAngle(yaw_ - init_yaw_);
        
        // 全方向移動時に補正を適用
        double velocity_magnitude = std::sqrt(twist_msg->linear.x * twist_msg->linear.x + 
                                            twist_msg->linear.y * twist_msg->linear.y);
        double velocity_factor = std::clamp(velocity_magnitude / linear_x_scale_, 0.3, 1.0);
        
        // PID補正を適用
        double pid_correction = calculateAngularCorrectionWithVelocity(error, filtered_angular_vel_z_, dt, velocity_factor);
        
        // PID補正値を適用（機体左旋回→右回転補正、正のエラー→正の補正）
        twist_msg->angular.z = pid_correction * angular_scale_;
        
        // **デバッグ**: 計算過程を詳細表示
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "CALC_DEBUG: PID=%.4f, angular_scale=%.2f, RESULT=%.4f", 
                             pid_correction, angular_scale_, twist_msg->angular.z);
        
        // デバッグログ（IMUデータの詳細確認）
        double yaw_drift_deg = error * 180.0 / M_PI;
        static int log_counter = 0;
        log_counter++;
        
        if (std::abs(yaw_drift_deg) > 0.5 && log_counter % 10 == 0) {  
          RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                               "IMU_CHECK: yaw=%.2f° init=%.2f° error=%.2f° %s, PID=%.4f, FINAL=%.4f", 
                               yaw_ * 180.0 / M_PI, init_yaw_ * 180.0 / M_PI,
                               std::abs(yaw_drift_deg), 
                               (yaw_drift_deg > 0) ? "LEFT" : "RIGHT", 
                               pid_correction, twist_msg->angular.z);
        }
      } else if (std::abs(manual_angular) > 0.01) {
        // 手動回転入力がある場合はそれを優先し、PID状態をリセット
        integral_error_ = 0.0;
        prev_yaw_error_ = 0.0;
        twist_msg->angular.z = manual_angular;
        was_manual_rotating = true;  // 手動回転中フラグを設定
      } else {
        // 手動回転終了時の処理（移動停止時）
        if (was_manual_rotating) {
          init_yaw_ = yaw_;  // 現在の向きを新しい基準に設定
          integral_error_ = 0.0;
          prev_yaw_error_ = 0.0;
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                               "Manual rotation ended (stopped) - New target: %.1f°", init_yaw_ * 180.0 / M_PI);
          was_manual_rotating = false;
        }
        
        // 移動停止時のフラグリセット＋PID状態クリア
        static bool was_moving = true;
        if (was_moving) {
          // 入力停止時にPID状態をクリア（切り返し対応）
          integral_error_ = 0.0;
          prev_yaw_error_ = 0.0;
          last_correction_time_ = 0.0;
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                               "Movement stopped - PID state cleared for smooth restart");
        }
        was_moving = false;
      }
    } break;
    case Mode::DPAD: {
      // 角度誤差を計算（正規化済み）
      double error = normalizeAngle(yaw_ - init_yaw_);

      if (!l2_pressed && !r2_pressed) {
        twist_msg->linear.x = (msg->buttons[11] - msg->buttons[12]) * linear_x_scale_ / 2.0;
        twist_msg->linear.y = (msg->buttons[13] - msg->buttons[14]) * linear_y_scale_ / 2.0;
        
        // 現在時刻の取得
        auto now = this->get_clock()->now();
        double current_time = now.seconds();
        double dt = (last_correction_time_ > 0) ? (current_time - last_correction_time_) : 0.02;
        last_correction_time_ = current_time;
        
        // DPADモードでは最大強度で角度+角速度補正を適用
        // 符号修正：エラーと同じ方向に補正して打ち消す
        twist_msg->angular.z = calculateAngularCorrectionWithVelocity(error, filtered_angular_vel_z_, dt, 1.0);
        
        // デバッグ出力（補正時のみ）
        if (std::abs(twist_msg->angular.z) > 0.01) {
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                               "DPAD correction: error=%.3f, ang_vel=%.3f, correction=%.3f", 
                               error, filtered_angular_vel_z_, twist_msg->angular.z);
        }
      } else {
        // 手動回転時はPID状態をリセット
        integral_error_ = 0.0;
        prev_yaw_error_ = 0.0;
        twist_msg->angular.z = get_angular_velocity(msg);
      }
    } break;
    case Mode::LINETRACE:
      break;
    default:
      RCLCPP_WARN(this->get_logger(), "Unknown mode: %d", static_cast<int>(mode_));
  }

  // 反転させる
  if (prev_reverse_button == 0 && msg->buttons[5] == 1) {
    // twist_msg->linear.x = -twist_msg->linear.x;
    // twist_msg->linear.y = -twist_msg->linear.y;
    reverse_flag = !reverse_flag;
  }

  prev_reverse_button = msg->buttons[5];

  if (reverse_flag) {
    twist_msg->linear.x = -twist_msg->linear.x;
    twist_msg->linear.y = -twist_msg->linear.y;
  }

  // cmd_velのpublish
  if (mode_ != Mode::LINETRACE) {
    // 重要: 実際に送信しているcmd_velを診断ログ
    if (std::abs(twist_msg->angular.z) > 0.1) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "SENDING: wz=%.3f (should fix 90° drift!)", twist_msg->angular.z);
    }
    cmd_vel_publisher_->publish(std::move(twist_msg));
  }

  // Map joystick axes to velocity commands
  // auto twist_msg = set_velocity(msg);
  // auto twist_msg = set_angular_velocity(msg);

  // RCLCPP_INFO(this->get_logger(), "Publishing cmd_vel: linear.x=%.2f, linear.y=%.2f,
  // angular.z=%.2f",
  //             twist_msg->linear.x, twist_msg->linear.y, twist_msg->angular.z);

  // Publish the velocity command

  // auto dpad_msg = std::make_unique<custom_interfaces::msg::CmdDpad>();
  // dpad_msg->up = msg->buttons[11];
  // dpad_msg->down = msg->buttons[12];
  // dpad_msg->left = msg->buttons[13];
  // dpad_msg->right = msg->buttons[14];

  auto upper_msg = std::make_unique<custom_interfaces::msg::UpperMotor>();
  // UpperMotor.msgのフィールド設定

  // システム準備状態（L2 + R2ボタン同時押し）
  // 浮動小数点比較なので閾値を使用
  upper_msg->is_system_ready = (msg->axes[4] < -0.9 && msg->axes[5] < -0.9);

  // square button (射出)
  if (msg->buttons[2] == 1) {
    upper_msg->is_throwing_on = true;
  } else {
    upper_msg->is_throwing_on = false;
  }

  // circle button (押出)
  if (msg->buttons[1] == 1) {
    upper_msg->is_ejection_on = true;
  } else {
    upper_msg->is_ejection_on = false;
  }

  // 昇降制御（方向パッド）
  if (msg->buttons[3] == 1) {         // triangle
    upper_msg->elevation_mode = 1;    // 上昇
  } else if (msg->buttons[0] == 1) {  // x
    upper_msg->elevation_mode = 0;    // 下降
  } else {
    upper_msg->elevation_mode = 2;  // 停止
  }

  //  一旦廃止
  // // Check for state changes
  // if (msg->buttons[1] == 1 && prev_throwing_on == false) {
  //   prev_throwing_on = true;
  //   RCLCPP_INFO(this->get_logger(), "Throwing on");
  // } else if (msg->buttons[3] == 1 && prev_throwing_on == true) {
  //   prev_throwing_on = false;
  //   RCLCPP_INFO(this->get_logger(), "Throwing off");
  // } else if (msg->buttons[0] == 1 && prev_ejection_on == false) {
  //   prev_ejection_on = true;
  //   RCLCPP_INFO(this->get_logger(), "Ejection on");
  // } else if (msg->buttons[2] == 1 && prev_ejection_on == true) {
  //   prev_ejection_on = false;
  //   RCLCPP_INFO(this->get_logger(), "Ejection off");
  // } else if (msg->buttons[10] == 1 && prev_elevation_on == false) {
  //   prev_elevation_on = true;
  //   RCLCPP_INFO(this->get_logger(), "Elevation on");
  // } else if (msg->buttons[9] == 1 && prev_elevation_on == true) {
  //   prev_elevation_on = false;
  //   RCLCPP_INFO(this->get_logger(), "Elevation off");
  // }

  // // Always set current state to the message
  // upper_msg->is_throwing_on = prev_throwing_on;
  // upper_msg->is_ejection_on = prev_ejection_on;
  // upper_msg->is_elevation_on = prev_elevation_on;

  // Always publish current state
  upper_publisher_->publish(std::move(upper_msg));
  // cmd_dpad_publisher_->publish(std::move(dpad_msg));

  if (mode_ == Mode::LINETRACE) {
    linetrace_msg->data = true;
  } else {
    linetrace_msg->data = false;
  }

  linetrace_publisher_->publish(std::move(linetrace_msg));

  mode_msg->data = mode_to_string(mode_);
  mode_publisher_->publish(std::move(mode_msg));
}

void JoyDriverNode::rpy_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
  // BNO055からは度（degrees）で来るのでradianに変換
  roll_ = msg->x * M_PI / 180.0;
  pitch_ = msg->y * M_PI / 180.0;
  
  // 重要: 正しい軸はX軸だった！（テストで確認済み）
  double raw_yaw = msg->x * M_PI / 180.0;  // X軸を使用するように修正
  
  // IMU軸診断：すべての軸を表示して正しい軸を確認
  RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                       "IMU RAW DATA: X=%.1f°, Y=%.1f°, Z=%.1f° (using X for yaw)", 
                       msg->x, msg->y, msg->z);
  
  // IMUデータの詳細確認用ログ（変化量も表示）
  static double prev_x = 0.0, prev_y = 0.0, prev_z = 0.0;
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                       "IMU ANALYSIS: X=%.2f°(Δ%.2f), Y=%.2f°(Δ%.2f), Z=%.2f°(Δ%.2f)", 
                       msg->x, msg->x - prev_x,
                       msg->y, msg->y - prev_y, 
                       msg->z, msg->z - prev_z);
  prev_x = msg->x; prev_y = msg->y; prev_z = msg->z;
  
  // IMUデータの確認用ログ（5秒間隔に削減）
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                       "IMU: yaw=%.1f° (change_rate=%.3f°/s)", msg->x,  // X軸を表示
                       (last_yaw_log_time_ > 0) ? (msg->x - last_yaw_log_) / 5.0 : 0.0);
  
  // ログ用の前回値を保存（X軸を使用）
  last_yaw_log_ = msg->x;
  last_yaw_log_time_ = this->get_clock()->now().seconds();
  
  // ローパスフィルタで角度データを平滑化
  if (filtered_yaw_ == 0.0) {
    // 初回は生データを使用
    filtered_yaw_ = raw_yaw;
  } else {
    // 角度の連続性を考慮したフィルタリング
    double yaw_diff = normalizeAngle(raw_yaw - filtered_yaw_);
    filtered_yaw_ = normalizeAngle(filtered_yaw_ + YAW_FILTER_ALPHA * yaw_diff);
  }
  
  yaw_ = filtered_yaw_;
}

void JoyDriverNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  // 角速度データを取得（既にrad/s）
  angular_vel_x_ = msg->angular_velocity.x;
  angular_vel_y_ = msg->angular_velocity.y;
  angular_vel_z_ = msg->angular_velocity.z;
  
  // IMU角速度データの確認用ログ（1秒間隔）
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                       "IMU Angular Velocity: x=%.3f, y=%.3f, z=%.3f rad/s", 
                       angular_vel_x_, angular_vel_y_, angular_vel_z_);
  
  // Z軸角速度にローパスフィルタを適用
  filtered_angular_vel_z_ = YAW_FILTER_ALPHA * angular_vel_z_ + 
                           (1.0 - YAW_FILTER_ALPHA) * filtered_angular_vel_z_;
}

double JoyDriverNode::get_angular_velocity(const sensor_msgs::msg::Joy::SharedPtr& msg) {
  // L2/R2を押した時の手動回転時は基準値を更新
  // （ただし、これは意図的な回転なので基準をリセット）
  static bool was_manual_rotation = false;
  bool is_manual_rotation =
      (msg->axes[4] < TRIGGER_THRESHOLD) || (msg->axes[5] < TRIGGER_THRESHOLD);

  if (!was_manual_rotation && is_manual_rotation) {
    // 手動回転開始時に基準値を更新してPID状態をリセット
    init_yaw_ = yaw_;
    integral_error_ = 0.0;
    prev_yaw_error_ = 0.0;
    last_correction_time_ = 0.0;
  }
  was_manual_rotation = is_manual_rotation;

  if (msg->axes[4] < TRIGGER_THRESHOLD && msg->axes[5] >= TRIGGER_THRESHOLD) {
    // R2: rotate right
    return -(msg->axes[4] - 1);
  } else if (msg->axes[5] < TRIGGER_THRESHOLD && msg->axes[4] >= TRIGGER_THRESHOLD) {
    // L2: rotate left
    return (msg->axes[5] - 1);
  } else {
    return 0.0;
  }
}

double JoyDriverNode::applyDeadzone(double val, double threshold) {
  return (std::abs(val) < threshold) ? 0.0 : val;
}

std::string JoyDriverNode::mode_to_string(Mode mode) {
  switch (mode) {
    case Mode::STOP:
      return "STOP";
    case Mode::JOY:
      return "JOY";
    case Mode::DPAD:
      return "DPAD";
    case Mode::LINETRACE:
      return "LINETRACE";
    default:
      return "UNKNOWN";
  }
}

double JoyDriverNode::normalizeAngle(double angle) {
  // -πからπの範囲に正規化（より効率的な実装）
  angle = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (angle < 0) angle += 2.0 * M_PI;
  return angle - M_PI;
}

double JoyDriverNode::calculateAngularCorrectionWithVelocity(double angle_error, double angular_vel_z, double dt, double velocity_factor) {
  // デッドバンド処理を一時無効化（すべての誤差に対して補正を適用）
  // if (std::abs(angle_error) < deadband_) {
  //   integral_error_ = 0.0;
  //   prev_yaw_error_ = 0.0;
  //   return 0.0;
  // }

  // 積分項の計算（ウィンドアップ防止付き）
  integral_error_ += angle_error * dt;
  integral_error_ = std::clamp(integral_error_, -MAX_INTEGRAL_ERROR, MAX_INTEGRAL_ERROR);

  // 微分項の計算（角度の変化率）
  double derivative = (dt > 0.001) ? (angle_error - prev_yaw_error_) / dt : 0.0;
  prev_yaw_error_ = angle_error;

  // 速度依存の適応的ゲイン調整（適切な値に調整）
  double adaptive_kp = Kp_ * velocity_factor;        // 標準の比例ゲイン
  double adaptive_ki = Ki_ * velocity_factor;        // 標準の積分ゲイン
  double adaptive_kd = Kd_ * velocity_factor;        // 標準の微分ゲイン

  // 角度ベースのPID計算（標準的なPID制御）
  double angle_correction = adaptive_kp * angle_error + 
                           adaptive_ki * integral_error_ + 
                           adaptive_kd * derivative;

  // 角速度フィードバック制御を適切に調整（現在の回転を適度に抑制）
  double current_angular_vel = (std::abs(angular_vel_z) > 0.001) ? angular_vel_z : filtered_angular_vel_z_;
  double velocity_damping = -0.1 * current_angular_vel * velocity_factor;  // ダンピングを適度に
  
  // 総合補正値
  double total_correction = angle_correction + velocity_damping;
  
  // 微小補正も含めた詳細ログ
  static int debug_counter = 0;
  debug_counter++;
  if (std::abs(angle_error) > 0.01 && debug_counter % 50 == 0) {  // 0.6度以上の誤差で50回に1回ログ
    RCLCPP_INFO(this->get_logger(), 
                "PID_DETAIL: err=%.4f°, P=%.4f, I=%.4f, D=%.4f, VelDamp=%.4f, Total=%.4f",
                angle_error * 180.0 / M_PI, 
                adaptive_kp * angle_error,
                adaptive_ki * integral_error_,
                adaptive_kd * derivative,
                velocity_damping,
                total_correction);
  }

  // 出力制限
  return std::clamp(total_correction, -max_angular_correction_, max_angular_correction_);
}
