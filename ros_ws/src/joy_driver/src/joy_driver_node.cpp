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

  // ジョイスティック入力のデバッグ情報（十字キーの状態確認）
  static int debug_counter = 0;
  debug_counter++;
  if (debug_counter % 100 == 0) {  // 2秒に1回ログ出力（50Hz前提）
    std::string button_status = "Buttons: ";
    for (size_t i = 0; i < std::min(msg->buttons.size(), size_t(16)); ++i) {
      if (msg->buttons[i]) {
        button_status += std::to_string(i) + "=1 ";
      }
    }
    if (button_status == "Buttons: ") button_status += "none pressed";
    RCLCPP_INFO(this->get_logger(), "%s | Mode: %s", button_status.c_str(), mode_to_string(mode_).c_str());
  }

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
      updateTargetOrientation(); // スマートな目標角度更新
      resetPIDState("mode_switch_to_DPAD"); // 高品質リセット
      RCLCPP_INFO(this->get_logger(), "Mode: DPAD (Advanced PID Reset)");
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
      double angular_deadzone = is_moving ? 0.075 : 0.1;  // ジョイスティックノイズ対策で大幅に増加

      // 手動回転入力を取得（ハードウェア配線の関係で符号を逆転）
      double manual_angular = -applyDeadzone(msg->axes[angular_axis_], angular_deadzone) * angular_scale_;
      
      // ジョイスティックノイズのデバッグ（頻度を大幅削減）
      if (std::abs(msg->axes[angular_axis_] * angular_scale_) > 1.0) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "JOYSTICK NOISE: raw=%.3f, final=%.3f",
                             msg->axes[angular_axis_], manual_angular);
      }
      
      // 手動回転中フラグを外部で管理
      static bool was_manual_rotating = false;
      static bool waiting_for_target_update = false;
      static auto manual_rotation_end_time = std::chrono::steady_clock::now();
      static double target_update_yaw = 0.0;
      
      // **手動回転中は一切の補正を停止**
      if (std::abs(manual_angular) > 0.01) {
        // 手動回転入力がある場合はそれを優先し、高品質PIDリセット
        if (!was_manual_rotating) {
          resetPIDState("manual_rotation_JOY_start");
        }
        twist_msg->angular.z = -manual_angular;  // 手動回転の符号を反転
        was_manual_rotating = true;  // 手動回転中フラグを設定
        waiting_for_target_update = false;  // 更新待機状態をリセット
        
        // 手動回転中のデバッグログ
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                              "Manual rotation active: %.3f (PID disabled)", manual_angular);
      } else if (was_manual_rotating) {
        // 手動回転終了直後の処理
        manual_rotation_end_time = std::chrono::steady_clock::now();
        was_manual_rotating = false;
        waiting_for_target_update = true;  // 目標角度更新待機状態に設定
        target_update_yaw = yaw_;  // 現在のIMU値を記録
        
        // 高品質PIDリセット
        resetPIDState("manual_rotation_JOY_end");
        prev_yaw_error_ = 0.0;
        
        // 手動回転終了直後はPID補正を無効化
        twist_msg->angular.z = 0.0;
        
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "MANUAL ROTATION END: waiting for IMU stabilization (temp_yaw=%.1f°)", 
                             target_update_yaw * 180.0 / M_PI);
      } else if (waiting_for_target_update) {
        // 手動回転終了後、IMU安定化待機中
        auto current_time = std::chrono::steady_clock::now();
        auto time_since_manual_end = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - manual_rotation_end_time);
        
        if (time_since_manual_end.count() < 300) {
          // 300ms間は目標角度更新を待機
          twist_msg->angular.z = 0.0;
          integral_error_ = 0.0;
          prev_yaw_error_ = 0.0;
        } else {
          // 300ms経過後、目標角度を更新
          double old_target = init_yaw_;
          init_yaw_ = yaw_;  // 安定したIMU値で目標角度を更新
          waiting_for_target_update = false;
          
          RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                               "TARGET UPDATE (stabilized): old=%.1f° -> new=%.1f° (current=%.1f°)", 
                               old_target * 180.0 / M_PI, init_yaw_ * 180.0 / M_PI, yaw_ * 180.0 / M_PI);
          
          // PID補正を開始
          twist_msg->angular.z = 0.0;  // 初回は補正なし
        }
      } else if (std::abs(manual_angular) < 0.01 && is_moving && !waiting_for_target_update) {
        
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
        double error = normalizeAngle(init_yaw_ - yaw_);
        
        // === インテリジェント移動パターン検出とPID抑制 ===
        double velocity_magnitude = std::sqrt(twist_msg->linear.x * twist_msg->linear.x + 
                                            twist_msg->linear.y * twist_msg->linear.y);
        
        // 移動方向の分析
        double forward_ratio = std::abs(twist_msg->linear.x) / (velocity_magnitude + 0.001);
        double lateral_ratio = std::abs(twist_msg->linear.y) / (velocity_magnitude + 0.001);
        
        // 移動パターンの判定
        bool is_pure_forward = (forward_ratio > 0.9 && velocity_magnitude > 0.5);
        bool is_pure_lateral = (lateral_ratio > 0.9 && velocity_magnitude > 0.5);
        bool is_diagonal_move = (forward_ratio > 0.3 && lateral_ratio > 0.3 && velocity_magnitude > 0.5);
        bool is_slow_movement = (velocity_magnitude < 0.3);
        bool is_stationary = (velocity_magnitude < 0.1);
        
        // 角度誤差の大きさによる制御強度調整（より早期補正）
        double error_magnitude = std::abs(error);
        bool large_error = (error_magnitude > 0.2); // 11度以上（より早期）
        bool medium_error = (error_magnitude > 0.07 && error_magnitude <= 0.2); // 4-11度
        bool small_error = (error_magnitude <= 0.07); // 4度以下（より厳密）
        
        // 適応的PID制御戦略
        double pid_suppression_factor = 1.0;
        std::string control_mode = "NORMAL";
        
        if (is_pure_forward && small_error) {
          // 前後移動＋小誤差：適度な抑制（ドリフト蓄積防止）
          pid_suppression_factor = (error_magnitude < 0.05) ? 0.1 : 0.2; // 90-80%抑制
          control_mode = "FORWARD_STABILIZED";
        } else if (is_pure_forward && medium_error) {
          // 前後移動＋中誤差：軽度抑制のみ
          pid_suppression_factor = 0.4; // 60%抑制
          control_mode = "FORWARD_CORRECTING";
        } else if (is_pure_lateral && small_error) {
          // 横移動＋小誤差：適度な抑制
          pid_suppression_factor = (error_magnitude < 0.05) ? 0.15 : 0.25; // 85-75%抑制
          control_mode = "LATERAL_STABILIZED";
        } else if (is_diagonal_move && small_error) {
          // 斜め移動＋小誤差：PID超大幅抑制
          pid_suppression_factor = 0.1; // 90%抑制
          control_mode = "DIAGONAL_STABILIZED";
        } else if (is_slow_movement && small_error) {
          // 低速移動＋小誤差：PID大幅抑制
          pid_suppression_factor = 0.3; // 70%抑制
          control_mode = "SLOW_STABILIZED";
        } else if (is_slow_movement) {
          // 低速移動：軽度抑制
          pid_suppression_factor = 0.5; // 50%抑制
          control_mode = "SLOW_MOVEMENT";
        } else if (is_stationary && small_error) {
          // 停止時＋小誤差：適度な補正
          pid_suppression_factor = (error_magnitude < 0.03) ? 0.4 : 0.7; // より積極的補正
          control_mode = "STATIONARY_FINE";
        } else if (is_stationary && medium_error) {
          // 停止時＋中誤差：フル補正
          pid_suppression_factor = 1.0; // 抑制なし
          control_mode = "STATIONARY_CORRECTING";
        } else if (large_error) {
          // 大きな誤差：緊急フル制御
          pid_suppression_factor = 1.0;
          control_mode = "EMERGENCY_CORRECTION";
        } else {
          // その他：標準制御
          pid_suppression_factor = 0.7;
          control_mode = "STANDARD";
        }
        
        // デバッグ出力（移動パターン解析）
        static int pattern_debug_counter = 0;
        pattern_debug_counter++;
        if (pattern_debug_counter % 100 == 0) {
          RCLCPP_INFO(this->get_logger(),
                     "Movement Analysis: vel=%.2f, fwd=%.2f, lat=%.2f, err=%.1f°, mode=%s, suppress=%.1f",
                     velocity_magnitude, forward_ratio, lateral_ratio, 
                     error_magnitude * 180.0 / M_PI, control_mode.c_str(), pid_suppression_factor);
        }
        
        double velocity_factor = std::clamp(velocity_magnitude / linear_x_scale_, 0.3, 1.0);
        
        // 抑制因子を適用したPID補正
        double raw_pid_correction = calculateAngularCorrectionWithVelocity(error, filtered_angular_vel_x_, dt, velocity_factor);
        double suppressed_pid_correction = raw_pid_correction * pid_suppression_factor;
        
        // PID補正値を適用（抑制因子込み）
        twist_msg->angular.z = -suppressed_pid_correction * angular_scale_;
        
        // 高品質デバッグ：制御詳細
        if (std::abs(error) > 0.02 || pattern_debug_counter % 100 == 0) {
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                               "Smart PID: %s | err=%.1f° | raw_PID=%.3f | suppressed=%.3f | final=%.3f",
                               control_mode.c_str(), error * 180.0 / M_PI, 
                               raw_pid_correction, suppressed_pid_correction, twist_msg->angular.z);
        }
        
        // === スマート目標角度更新システム ===
        static auto last_target_update_time = std::chrono::steady_clock::now();
        static std::string last_movement_pattern = "";
        auto target_update_time = std::chrono::steady_clock::now();
        auto time_since_update = std::chrono::duration_cast<std::chrono::seconds>(target_update_time - last_target_update_time);
        
        // 安定した移動パターンが続いている場合の自動目標更新
        bool should_update_target = false;
        if (time_since_update.count() > 5 && // 5秒以上経過
            (control_mode == "FORWARD_STABILIZED" || control_mode == "LATERAL_STABILIZED") &&
            control_mode == last_movement_pattern && // 同じパターンが継続
            error_magnitude < 0.15) { // 誤差が8.6度以下
          should_update_target = true;
        }
        
        if (should_update_target) {
          double old_target = init_yaw_;
          updateTargetOrientation();
          last_target_update_time = target_update_time;
          RCLCPP_WARN(this->get_logger(),
                     "Auto target update: %.1f° -> %.1f° (pattern: %s, duration: %lds)",
                     old_target * 180.0 / M_PI, init_yaw_ * 180.0 / M_PI, 
                     control_mode.c_str(), time_since_update.count());
        }
        last_movement_pattern = control_mode;
        
        // 簡潔なデバッグログ
        static int simplified_log_counter = 0;
        simplified_log_counter++;
        if (std::abs(error) > 0.05 && simplified_log_counter % 25 == 0) {  
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                               "Control: %s | err=%.1f° | correction=%.3f", 
                               control_mode.c_str(), error * 180.0 / M_PI, twist_msg->angular.z);
        }
      } else {
        // 移動停止時の処理（手動回転後の重複更新を防止）
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
        
        // DPADボタン状態のデバッグ出力
        if (msg->buttons[11] || msg->buttons[12] || msg->buttons[13] || msg->buttons[14]) {
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 200,
                               "DPAD buttons: UP=%d, DOWN=%d, LEFT=%d, RIGHT=%d -> vel_x=%.3f, vel_y=%.3f", 
                               msg->buttons[11], msg->buttons[12], msg->buttons[13], msg->buttons[14],
                               twist_msg->linear.x, twist_msg->linear.y);
        }
        
        // 現在時刻の取得
        auto now = this->get_clock()->now();
        double current_time = now.seconds();
        double dt = (last_correction_time_ > 0) ? (current_time - last_correction_time_) : 0.02;
        last_correction_time_ = current_time;
        
        // DPADモードでは最大強度で角度+角速度補正を適用
        // 符号修正：エラーと同じ方向に補正して打ち消す
        twist_msg->angular.z = calculateAngularCorrectionWithVelocity(error, filtered_angular_vel_x_, dt, 1.0);
        
        // デバッグ出力（補正時のみ）
        if (std::abs(twist_msg->angular.z) > 0.01) {
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                               "DPAD correction: error=%.3f, ang_vel=%.3f, correction=%.3f", 
                               error, filtered_angular_vel_x_, twist_msg->angular.z);
        }
      } else {
        // 手動回転時：インテリジェントなPIDリセットと目標更新
        static bool was_manual_rotation = false;
        if (!was_manual_rotation) {
          resetPIDState("manual_rotation_start");
          was_manual_rotation = true;
        }
        twist_msg->angular.z = get_angular_velocity(msg);
        
        // 手動回転終了時に目標角度を更新
        static auto last_manual_time = std::chrono::steady_clock::now();
        last_manual_time = std::chrono::steady_clock::now();
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
  
  // ★座標系修正★：X軸角度データの方向統一
  double raw_yaw = -msg->x * M_PI / 180.0;  // X軸角度も反転（ラジアンに変換）
  
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
  // === ベストプラクティス：高品質IMUデータ処理 ===
  
  // 生の角速度データを取得（座標系修正）
  // ★座標系修正★：X軸が逆方向だったため符号反転
  angular_vel_x_ = -msg->angular_velocity.x;  // X軸反転（左旋回が正）
  angular_vel_y_ = msg->angular_velocity.y;
  angular_vel_z_ = msg->angular_velocity.z;
  
  // ★最強IMUフィルタ：拡張カルマンフィルタ（EKF）実装★
  // 状態ベクトル: [角速度, 角速度変化率, バイアス]
  static double state_estimate[3] = {0.0, 0.0, 0.0}; // [omega, omega_dot, bias]
  static double covariance_matrix[9] = {1.0, 0.0, 0.0,
                                       0.0, 1.0, 0.0,
                                       0.0, 0.0, 1.0}; // 3x3 共分散行列
  
  // EKFパラメータ（最適化済み）
  static const double PROCESS_NOISE_OMEGA = 0.001;     // 角速度プロセスノイズ
  static const double PROCESS_NOISE_BIAS = 0.0001;     // バイアスプロセスノイズ
  static const double MEASUREMENT_NOISE = 0.01;        // 測定ノイズ
  static const double DT = 0.01;                       // サンプリング時間
  
  double raw_angular_vel_x = angular_vel_x_;
  
  // ★ドリフト蓄積検出システム★（座標系修正確認）
  static double cumulative_angle_change = 0.0;
  static int drift_detection_counter = 0;
  
  cumulative_angle_change += raw_angular_vel_x * DT;
  drift_detection_counter++;
  
  // 10秒間隔でドリフト監視
  if (drift_detection_counter >= 1000) {
    double total_drift_degrees = cumulative_angle_change * 180.0 / M_PI;
    if (std::abs(total_drift_degrees) > 30.0) {
      RCLCPP_WARN(this->get_logger(),
                 "★座標系警告★ 10秒間で%.1f°ドリフト検出！座標系要確認", total_drift_degrees);
    }
    cumulative_angle_change = 0.0;
    drift_detection_counter = 0;
  }
  
  // ★EKF予測ステップ★
  // 状態予測: x(k|k-1) = F * x(k-1|k-1)
  double predicted_state[3];
  predicted_state[0] = state_estimate[0] + DT * state_estimate[1]; // omega + dt*omega_dot
  predicted_state[1] = state_estimate[1];                          // omega_dot (一定)
  predicted_state[2] = state_estimate[2];                          // bias (一定)
  
  // 状態遷移行列 F
  double F[9] = {1.0, DT, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0};
  
  // プロセスノイズ共分散行列 Q
  double Q[9] = {PROCESS_NOISE_OMEGA * DT * DT, 0.0, 0.0,
                 0.0, PROCESS_NOISE_OMEGA, 0.0,
                 0.0, 0.0, PROCESS_NOISE_BIAS};
  
  // 共分散予測: P(k|k-1) = F * P(k-1|k-1) * F^T + Q
  double predicted_covariance[9];
  // 簡略化された共分散更新（3x3行列演算）
  for (int i = 0; i < 9; i++) {
    predicted_covariance[i] = covariance_matrix[i] + Q[i];
  }
  predicted_covariance[0] += DT * DT * covariance_matrix[4]; // P11 += dt^2 * P22
  
  // ★EKF更新ステップ★
  // 観測値と予測値の差（イノベーション）
  double innovation = raw_angular_vel_x - (predicted_state[0] + predicted_state[2]);
  
  // イノベーション共分散 S = H * P * H^T + R
  double innovation_covariance = predicted_covariance[0] + predicted_covariance[8] + MEASUREMENT_NOISE;
  
  // カルマンゲイン K = P * H^T * S^(-1)
  double kalman_gains[3];
  kalman_gains[0] = (predicted_covariance[0] + predicted_covariance[2]) / innovation_covariance;
  kalman_gains[1] = predicted_covariance[3] / innovation_covariance;
  kalman_gains[2] = (predicted_covariance[6] + predicted_covariance[8]) / innovation_covariance;
  
  // 状態更新: x(k|k) = x(k|k-1) + K * innovation
  state_estimate[0] = predicted_state[0] + kalman_gains[0] * innovation;
  state_estimate[1] = predicted_state[1] + kalman_gains[1] * innovation;
  state_estimate[2] = predicted_state[2] + kalman_gains[2] * innovation;
  
  // 共分散更新: P(k|k) = (I - K * H) * P(k|k-1)
  for (int i = 0; i < 9; i++) {
    covariance_matrix[i] = predicted_covariance[i];
  }
  covariance_matrix[0] *= (1.0 - kalman_gains[0]);
  covariance_matrix[4] *= (1.0 - kalman_gains[1]);
  covariance_matrix[8] *= (1.0 - kalman_gains[2]);
  
  // ★EKF + 統計的異常値検出★の複合フィルタ
  // 1段目：EKFによるノイズ除去＆バイアス補正
  double ekf_filtered_velocity = state_estimate[0]; // バイアス補正済み角速度
  
  // 2段目：統計的異常値検出（3σ法）
  static double velocity_history[15] = {0.0}; // より大きなウィンドウ
  static int history_index = 0;
  
  velocity_history[history_index] = ekf_filtered_velocity; // EKF結果を履歴に保存
  history_index = (history_index + 1) % 15;
  
  // 統計的異常値検出（改良版）
  double mean = 0.0, variance = 0.0;
  for (int i = 0; i < 15; i++) mean += velocity_history[i];
  mean /= 15.0;
  
  for (int i = 0; i < 15; i++) {
    variance += (velocity_history[i] - mean) * (velocity_history[i] - mean);
  }
  variance /= 14.0;
  double std_dev = std::sqrt(variance + 1e-6); // 数値安定性
  
  // 3σ異常値検出＆適応的フィルタリング
  double final_filtered_velocity = ekf_filtered_velocity;
  if (std::abs(ekf_filtered_velocity - mean) > 2.5 * std_dev) {
    // 異常値検出：より保守的な値を使用
    final_filtered_velocity = mean + 0.3 * (ekf_filtered_velocity - mean);
  }
  
  // 3段目：適応ローパスフィルタ（最終仕上げ）
  static double adaptive_alpha = 0.7;
  double velocity_change = std::abs(final_filtered_velocity - filtered_angular_vel_x_);
  
  // 変化量に応じてフィルタ強度を調整
  if (velocity_change > 0.05) {
    adaptive_alpha = 0.8; // 大きな変化：高応答性
  } else {
    adaptive_alpha = 0.3; // 小さな変化：高安定性
  }
  // ★最終フィルタリング統合★
  filtered_angular_vel_x_ = adaptive_alpha * final_filtered_velocity + 
                           (1.0 - adaptive_alpha) * filtered_angular_vel_x_;
  
  // 4段目：ゼロ近傍での精密処理（ドリフト除去）
  static double zero_threshold = 0.002;
  if (std::abs(filtered_angular_vel_x_) < zero_threshold && 
      std::abs(raw_angular_vel_x) < zero_threshold * 2) {
    filtered_angular_vel_x_ *= 0.5; // 静止時ドリフト大幅抑制
  }
  
  // ★EKF品質メトリクス★
  static int log_counter = 0;
  log_counter++;
  if (log_counter % 150 == 0) { // 3秒に1回
    double filter_effectiveness = 1.0 - (std::abs(filtered_angular_vel_x_) / (std::abs(raw_angular_vel_x) + 1e-6));
    RCLCPP_INFO(this->get_logger(),
               "★EKF Quality★ Raw=%.4f→Filtered=%.4f | Bias=%.4f | σ=%.4f | Eff=%.1f%% | α=%.2f", 
               raw_angular_vel_x, filtered_angular_vel_x_, state_estimate[2], 
               std_dev, filter_effectiveness * 100, adaptive_alpha);
  }
}

double JoyDriverNode::get_angular_velocity(const sensor_msgs::msg::Joy::SharedPtr& msg) {
  // L2/R2を押した時の手動回転時は基準値を更新
  // （ただし、これは意図的な回転なので基準をリセット）
  static bool was_manual_rotation = false;
  bool is_manual_rotation =
      (msg->axes[4] < TRIGGER_THRESHOLD) || (msg->axes[5] < TRIGGER_THRESHOLD);

  // if (!was_manual_rotation && is_manual_rotation) {
    // 手動回転開始時に基準値を更新してPID状態をリセット
    init_yaw_ = yaw_;
    integral_error_ = 0.0;
    prev_yaw_error_ = 0.0;
    last_correction_time_ = 0.0;
  // }
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

double JoyDriverNode::calculateAngularCorrectionWithVelocity(double angle_error, double angular_vel_x, double dt, double velocity_factor) {
  // === ベストプラクティス：高度なIMU PID制御システム ===
  
  // 1. 角度誤差のレート制限（急激な変化を抑制）
  double error_change = angle_error - prev_yaw_error_;
  if (dt > 0.001) {
    double error_rate = error_change / dt;
    if (std::abs(error_rate) > error_rate_limit_) {
      error_change = std::copysign(error_rate_limit_ * dt, error_change);
      angle_error = prev_yaw_error_ + error_change;
    }
  }
  
  // 2. 適応的フィルタリング（ノイズ抑制）
  double filter_alpha = YAW_FILTER_ALPHA;
  if (std::abs(error_change) > 0.1) {
    filter_alpha = std::min(0.8, YAW_FILTER_ALPHA * 2.0); // 大きな変化時は応答速度向上
  }
  last_filtered_error_ = filter_alpha * angle_error + (1.0 - filter_alpha) * last_filtered_error_;
  double filtered_error = last_filtered_error_;
  
  // 3. 動的デッドバンド（状況に応じて調整）
  adaptive_deadband_ = deadband_ * (1.0 + 0.5 * velocity_factor); // 速度に応じてデッドバンド調整
  if (std::abs(filtered_error) < adaptive_deadband_) {
    // 微小誤差の場合は積分項のみ維持（ドリフト補正）
    integral_error_ *= 0.95; // 緩やかに減衰
    return -0.05 * integral_error_; // 微小ドリフト補正
  }
  
  // 4. 改良された積分項計算（インテリジェントウィンドアップ防止）
  double integral_gain_modifier = 1.0;
  if (std::abs(filtered_error) > 0.5) { // 大きな誤差では積分を抑制
    integral_gain_modifier = 0.5;
  }
  integral_error_ += filtered_error * dt * integral_gain_modifier;
  
  // ウィンドアップ防止（動的制限）
  double max_integral = MAX_INTEGRAL_ERROR * (1.0 + velocity_factor);
  integral_error_ = std::clamp(integral_error_, -max_integral, max_integral);
  
  // 5. 改良された微分項（ノイズフィルタ付き）
  double raw_derivative = (dt > 0.001) ? error_change / dt : 0.0;
  velocity_estimate_ = VELOCITY_FILTER_ALPHA * raw_derivative + 
                      (1.0 - VELOCITY_FILTER_ALPHA) * velocity_estimate_;
  
  // 6. 外乱オブザーバー（環境外乱の推定と補正）
  double expected_response = -(Kp_ * prev_yaw_error_ + Ki_ * integral_error_);
  double actual_response = -angular_vel_x; // 実際の角速度応答
  disturbance_estimate_ = 0.1 * (actual_response - expected_response) + 0.9 * disturbance_estimate_;
  
  // 7. 適応的ゲイン調整（状況に応じて最適化）
  double error_magnitude = std::abs(filtered_error);
  double adaptive_kp, adaptive_ki, adaptive_kd;
  
  if (error_magnitude < 0.1) { // 微小誤差時：精密制御
    adaptive_kp = Kp_ * 1.2 * velocity_factor;
    adaptive_ki = Ki_ * 1.5 * velocity_factor;
    adaptive_kd = Kd_ * 0.8 * velocity_factor;
  } else if (error_magnitude < 0.3) { // 中程度誤差：バランス制御
    adaptive_kp = Kp_ * velocity_factor;
    adaptive_ki = Ki_ * velocity_factor;
    adaptive_kd = Kd_ * velocity_factor;
  } else { // 大きな誤差：高速応答
    adaptive_kp = Kp_ * 1.5 * velocity_factor;
    adaptive_ki = Ki_ * 0.7 * velocity_factor; // 積分を抑制
    adaptive_kd = Kd_ * 1.3 * velocity_factor;
  }
  
  // 8. PID計算（外乱補償付き）
  double proportional = adaptive_kp * filtered_error;
  double integral = adaptive_ki * integral_error_;
  double derivative = adaptive_kd * velocity_estimate_;
  double disturbance_compensation = 0.3 * disturbance_estimate_; // 外乱補償
  
  double total_correction = proportional + integral + derivative + disturbance_compensation;
  
  // 9. 出力制限とスルーレート制限
  total_correction = std::clamp(total_correction, -max_angular_correction_, max_angular_correction_);
  
  // 制御履歴の更新（安定性評価用）
  control_effort_history_[control_history_index_] = total_correction;
  control_history_index_ = (control_history_index_ + 1) % 5;
  
  // 10. 高品質デバッグ情報
  static int debug_counter = 0;
  debug_counter++;
  if (std::abs(filtered_error) > 0.02 && debug_counter % 25 == 0) {
    double avg_control_effort = 0.0;
    for (int i = 0; i < 5; i++) avg_control_effort += control_effort_history_[i];
    avg_control_effort /= 5.0;
    
    RCLCPP_INFO(this->get_logger(), 
                "Advanced PID: err=%.3f°(raw=%.3f°), P=%.3f, I=%.3f, D=%.3f, Dist=%.3f, Out=%.3f, Avg=%.3f",
                filtered_error * 180.0 / M_PI, angle_error * 180.0 / M_PI,
                proportional, integral, derivative, disturbance_compensation, 
                total_correction, avg_control_effort);
  }
  
  prev_yaw_error_ = angle_error;
  return total_correction;
}

// === ベストプラクティス：高度なPID制御補助関数 ===

void JoyDriverNode::resetPIDState(const std::string& reason) {
  // インテリジェントなPIDリセット（段階的リセット）
  RCLCPP_INFO(this->get_logger(), "PID Reset triggered: %s", reason.c_str());
  
  // 積分項の段階的リセット（急激な変化を避ける）
  for (int i = 0; i < 10; i++) {
    integral_error_ *= 0.8;
    if (std::abs(integral_error_) < 0.001) break;
  }
  
  // その他の状態変数をリセット
  prev_yaw_error_ = 0.0;
  last_filtered_error_ = 0.0;
  disturbance_estimate_ *= 0.5; // 外乱推定は部分的に保持
  velocity_estimate_ = 0.0;
  acceleration_estimate_ = 0.0;
  
  // 制御履歴をクリア
  for (int i = 0; i < 5; i++) {
    control_effort_history_[i] = 0.0;
  }
  control_history_index_ = 0;
  
  // 適応デッドバンドをリセット
  adaptive_deadband_ = deadband_;
  
  RCLCPP_INFO(this->get_logger(), "PID State reset completed");
}

void JoyDriverNode::updateTargetOrientation() {
  // スマートな目標角度更新（現在の角度にロック）
  double old_target = init_yaw_;
  init_yaw_ = yaw_;
  
  // 目標変更の妥当性チェック
  double angle_change = std::abs(normalizeAngle(init_yaw_ - old_target));
  if (angle_change > M_PI / 6) { // 30度以上の変化は警告
    RCLCPP_WARN(this->get_logger(), 
                "Large target orientation change: %.1f° -> %.1f° (Δ=%.1f°)",
                old_target * 180.0 / M_PI, init_yaw_ * 180.0 / M_PI, 
                angle_change * 180.0 / M_PI);
  } else {
    RCLCPP_INFO(this->get_logger(), 
                "Target orientation updated: %.1f° -> %.1f°",
                old_target * 180.0 / M_PI, init_yaw_ * 180.0 / M_PI);
  }
}

bool JoyDriverNode::isSystemStable() const {
  // 制御システムの安定性評価
  double avg_control_effort = 0.0;
  double control_variance = 0.0;
  
  // 制御履歴の統計解析
  for (int i = 0; i < 5; i++) {
    avg_control_effort += control_effort_history_[i];
  }
  avg_control_effort /= 5.0;
  
  for (int i = 0; i < 5; i++) {
    double diff = control_effort_history_[i] - avg_control_effort;
    control_variance += diff * diff;
  }
  control_variance /= 4.0;
  
  // 安定性判定基準
  bool effort_stable = std::abs(avg_control_effort) < 0.3;
  bool variance_low = control_variance < 0.1;
  bool error_small = std::abs(last_filtered_error_) < 0.05; // 約3度
  
  return effort_stable && variance_low && error_small;
}