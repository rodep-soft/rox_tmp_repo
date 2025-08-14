#pragma once

#include <atomic>
#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <thread>
#include <vector>

#include "mecanum_wheel_controller/motor_controller.hpp"

class MecanumWheelControllerNode : public rclcpp::Node {
 public:
  MecanumWheelControllerNode();
  ~MecanumWheelControllerNode();

 private:
  // handle ros2 params
  void declare_parameters();
  void get_parameters();

  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void brake_service_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                              std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  void timer_send_velocity_callback();
  void stop_all_motors();
  void publish_wheel_odometry(double vx, double vy, double wz, 
                              int16_t rpm_sent_fl, int16_t rpm_sent_fr, 
                              int16_t rpm_sent_rl, int16_t rpm_sent_rr);
  void calculate_robot_velocity_from_feedback(double& vx, double& vy, double& wz);

  // ROS2 components
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr brake_service_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;

  MotorController motor_controller_;

  enum class Brake { NONE = 0, ENGAGE = 1 };

  Brake brake_ = Brake::NONE;

  // params
  double wheel_radius_;
  double wheel_base_x_;
  double wheel_base_y_;
  
  // Motor hardware correction factors
  double motor_correction_fl_;
  double motor_correction_fr_;
  double motor_correction_rl_;
  double motor_correction_rr_;

  std::atomic<double> vx_, vy_, wz_;
  
  // Odometry state
  double x_, y_, theta_;
  std::chrono::steady_clock::time_point last_odom_time_;
  
  // Motor feedback for odometry (future implementation)
  std::atomic<int16_t> actual_rpm_fl_{0}, actual_rpm_fr_{0}, actual_rpm_rl_{0}, actual_rpm_rr_{0};

  std::atomic<std::chrono::time_point<std::chrono::steady_clock>> last_subscription_time_;
  std::string serial_port_;
  int baud_rate_;
  std::vector<int> motor_ids_;
  int cmd_vel_timeout_ms_;

  rclcpp::TimerBase::SharedPtr timer_;

  // ros2 qos
  const rclcpp::QoS reliable_qos = rclcpp::QoS(1).reliable();
  const rclcpp::QoS best_effort_qos = rclcpp::QoS(10).best_effort();
};