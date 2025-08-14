#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

// include ROS2
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

// ROS messages
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include "custom_interfaces/msg/cmd_dpad.hpp"
#include "custom_interfaces/msg/upper_motor.hpp"

// ROS services
#include <std_srvs/srv/set_bool.hpp>

class JoyDriverNode : public rclcpp::Node {
 public:
  // Constructor
  JoyDriverNode();

  // gtestのためにpublicにしてある
  // Public enum
  enum class Mode { STOP, JOY, DPAD, LINETRACE };

  // Public helper functions
  static double applyDeadzone(double val, double threshold = 0.05);
  static double normalizeAngle(double angle);
  double calculatePIDCorrection(double error, double dt, double velocity_factor = 1.0);
  double calculateAngularCorrectionWithVelocity(double angle_error, double angular_vel_x, double dt, double velocity_factor = 1.0);
  std::string mode_to_string(Mode mode);
  double get_angular_velocity(const sensor_msgs::msg::Joy::SharedPtr& msg);
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void keyboard_callback(const std_msgs::msg::String::SharedPtr msg);
  
  // Advanced PID control functions (Best Practices)
  void resetPIDState(const std::string& reason = "manual");
  void updateTargetOrientation();
  bool isSystemStable() const;

 private:
  // default Mode
  Mode mode_ = Mode::STOP;

  // ros2パラメータを宣言、取得する関数
  void declare_parameters();
  void get_parameters();

  // Callback functions
  void rpy_callback(const geometry_msgs::msg::Vector3::SharedPtr msg);
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void ekf_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // Helper functions
  // double get_angular_velocity(const sensor_msgs::msg::Joy::SharedPtr& msg);  // moved to public
  // static double applyDeadzone(double val, double threshold = 0.05);  // moved to public
  // std::string mode_to_string(Mode mode);  // moved to public
  // void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);  // moved to public

  // ===== ROS2 components =====

  // ROS2 Subscription
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr rpy_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ekf_odom_subscription_;

  // ROS2 Publisher
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Publisher<custom_interfaces::msg::UpperMotor>::SharedPtr upper_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr linetrace_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_publisher_;

  // ROS2 Client
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr brake_client_;

  // ros2 QoS
  const rclcpp::QoS reliable_qos = rclcpp::QoS(1).reliable();
  const rclcpp::QoS best_effort_qos = rclcpp::QoS(10).best_effort();

  // ===== ROS2 components END =====

  // ===== ros2 params =====

  // joystick params
  double linear_x_scale_;
  double linear_y_scale_;
  double angular_scale_;
  int linear_x_axis_;
  int linear_y_axis_;
  int angular_axis_;
  bool ekf_fusion_enabled_;

  // PID
  double Kp_;
  double Ki_;
  double Kd_;
  double deadband_;
  double max_angular_correction_;

  // IMU補正用のフィルタとPID制御変数
  double filtered_yaw_ = 0.0;
  double prev_yaw_error_ = 0.0;
  double integral_error_ = 0.0;
  double last_correction_time_ = 0.0;
  
  // Advanced PID control variables (Best Practices)
  double error_rate_limit_ = 5.0;  // rad/s - 角度誤差変化率制限
  double last_filtered_error_ = 0.0;
  double disturbance_estimate_ = 0.0;  // 外乱推定値
  double adaptive_deadband_ = 0.02;   // 動的デッドバンド
  double control_effort_history_[5] = {0.0}; // 制御履歴（安定性評価用）
  int control_history_index_ = 0;
  double velocity_estimate_ = 0.0;    // 速度推定（カルマンフィルタ風）
  double acceleration_estimate_ = 0.0; // 加速度推定
  
  // ローパスフィルタ係数（0.0-1.0, 小さいほど平滑化が強い）
  static constexpr double YAW_FILTER_ALPHA = 0.3;  // より強い平滑化
  static constexpr double VELOCITY_FILTER_ALPHA = 0.4;  // 速度フィルタ
  
  // 積分項のウィンドアップ防止
  static constexpr double MAX_INTEGRAL_ERROR = 0.3;  // より保守的

  // ===== ros2 params END =====

  // Euler angles from IMU (raw data)
  double pitch_ = 0.0;
  double roll_ = 0.0;
  double yaw_ = 0.0;

  // EKF filtered pose data
  double ekf_yaw_ = 0.0;
  double ekf_x_ = 0.0;
  double ekf_y_ = 0.0;
  double ekf_angular_velocity_z_ = 0.0;
  bool ekf_data_received_ = false;

  // Angular velocities from IMU (rad/s)
  double angular_vel_x_ = 0.0;
  double angular_vel_y_ = 0.0;
  double angular_vel_z_ = 0.0;
  double filtered_angular_vel_x_ = 0.0;

  // 初期化時のyaw値
  // これはロボットの初期姿勢を基準にするための値
  // これを用いてDPADモードでの直線移動での回転方向のズレを補正する
  // L2, R2を押した時も値を更新する
  double init_yaw_ = 0.0;

  // ログ用の変数
  double last_yaw_log_ = 0.0;
  double last_yaw_log_time_ = 0.0;

  // member constants
  const double TRIGGER_THRESHOLD = 0.95;

  // other member variables (ex. flags)
  bool prev_linetrace_buttons_ = false;
  bool reverse_flag = false;
  bool prev_reverse_button = 0;

  bool prev_throwing_on = false;
  bool prev_ejection_on = false;
  bool prev_elevation_on = false;
};