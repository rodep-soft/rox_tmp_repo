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
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joy.hpp>
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

  // Public enum for testing
  enum class Mode { STOP, JOY, DPAD, LINETRACE };

  // Public helper functions for testing
  static double applyDeadzone(double val, double threshold = 0.05);
  static double normalizeAngle(double angle);
  double calculatePIDCorrection(double error, double dt, double velocity_factor = 1.0);
  double calculateAngularCorrectionWithVelocity(double angle_error, double angular_vel_x, double dt,
                                                double velocity_factor = 1.0);
  std::string mode_to_string(Mode mode, bool is_gear_down_param);
  double get_angular_velocity(const sensor_msgs::msg::Joy::SharedPtr& msg);
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void keyboard_callback(const std_msgs::msg::String::SharedPtr msg);

 private:
  // default Mode
  Mode mode_ = Mode::STOP;

  // ros2パラメータを宣言、取得する関数
  void declare_parameters();
  void get_parameters();

  // Callback functions
  void rpy_callback(const geometry_msgs::msg::Vector3::SharedPtr msg);
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

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

  // PID
  double Kp_;
  double Ki_;
  double Kd_;
  double deadband_;
  double max_angular_correction_;
  double vel_prediction_gain_;   // 角速度予測補正ゲイン
  double vel_damping_gain_;      // 角速度ダンピングゲイン

  // IMU補正用のフィルタとPID制御変数
  double filtered_yaw_ = 0.0;
  double prev_yaw_error_ = 0.0;
  double integral_error_ = 0.0;
  double last_correction_time_ = 0.0;

  // ローパスフィルタ係数（0.0-1.0, 小さいほど平滑化が強い）
  static constexpr double YAW_FILTER_ALPHA = 0.3;  // 角速度振動抑制のため強いフィルタ

  // 積分項のウィンドアップ防止
  static constexpr double MAX_INTEGRAL_ERROR = 0.5;

  // ===== ros2 params END =====

  // Euler
  double pitch_ = 0.0;
  double roll_ = 0.0;
  double yaw_ = 0.0;

  // Angular velocities from IMU (rad/s)
  double angular_vel_x_ = 0.0;
  double angular_vel_y_ = 0.0;
  double angular_vel_z_ = 0.0;
  double filtered_angular_vel_x_ = 0.0;  // yaw軸（x軸）のフィルタ済み角速度
  double filtered_angular_vel_z_ = 0.0;

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

  bool is_gear_down = false;
};