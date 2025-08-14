#include <chrono>
#include <cmath>
#include <functional>
#include <geometry_msgs/msg/vector3.hpp>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <thread>
#include <vector>

#include "imu/imulib.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class IMUNode : public rclcpp::Node {
 public:
  IMUNode() : Node("imu_node"), imu_sensor_(12345) {
    if (!imu_sensor_.begin()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize BNO055 sensor!");
      rclcpp::shutdown();
    }

    // Publishers
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
    mag_publisher_ = this->create_publisher<sensor_msgs::msg::MagneticField>("/imu/mag", 10);
    rpy_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("/imu/rpy", 10);

    accel_and_gyro_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10), std::bind(&IMUNode::timer_accel_and_gyro_callback, this));

    mag_timer_ = this->create_wall_timer(std::chrono::milliseconds(50),
                                         std::bind(&IMUNode::timer_mag_callback, this));
  }

 private:
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr rpy_publisher_;
  rclcpp::TimerBase::SharedPtr accel_and_gyro_timer_;
  rclcpp::TimerBase::SharedPtr mag_timer_;

  BNO055 imu_sensor_;  // Example sensor ID

  void timer_accel_and_gyro_callback() {
    // --- オイラー角取得 ---

    auto rpy_msg = geometry_msgs::msg::Vector3();

    Vector<3> euler = imu_sensor_.getVector(VECTOR_EULER);

    rpy_msg.x = euler[0];
    rpy_msg.y = euler[1];
    rpy_msg.z = euler[2];

    // --- ここからImuメッセージ ---

    auto imu_accel_and_gyro_msg = sensor_msgs::msg::Imu();
    
    // Set header with timestamp and frame_id
    imu_accel_and_gyro_msg.header.stamp = this->get_clock()->now();
    imu_accel_and_gyro_msg.header.frame_id = "imu_link";

    Quaternion quat = imu_sensor_.getQuat();

    imu_accel_and_gyro_msg.orientation.x = quat.x();
    imu_accel_and_gyro_msg.orientation.y = quat.y();
    imu_accel_and_gyro_msg.orientation.z = quat.z();
    imu_accel_and_gyro_msg.orientation.w = quat.w();

    Vector<3> accel = imu_sensor_.getVector(VECTOR_ACCELEROMETER);

    imu_accel_and_gyro_msg.linear_acceleration.x = accel[0];
    imu_accel_and_gyro_msg.linear_acceleration.y = accel[1];
    imu_accel_and_gyro_msg.linear_acceleration.z = accel[2];

    Vector<3> gyro = imu_sensor_.getVector(VECTOR_GYROSCOPE);

    imu_accel_and_gyro_msg.angular_velocity.x = gyro[0];
    imu_accel_and_gyro_msg.angular_velocity.y = gyro[1];
    imu_accel_and_gyro_msg.angular_velocity.z = gyro[2];

    imu_publisher_->publish(imu_accel_and_gyro_msg);
    rpy_publisher_->publish(rpy_msg);
    
    // Debug output every 2 seconds
    static auto last_debug = std::chrono::steady_clock::now();
    auto current_time = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(current_time - last_debug).count() > 2.0) {
      double yaw_deg = euler[0] * 180.0 / M_PI;
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                          "IMU DATA: yaw=%.1f° gyro_z=%.3f accel(%.2f,%.2f,%.2f)",
                          yaw_deg, gyro[2], accel[0], accel[1], accel[2]);
      last_debug = current_time;
    }
  }

  void timer_mag_callback() {
    auto mag_msg = sensor_msgs::msg::MagneticField();
    
    // Set header with timestamp and frame_id
    mag_msg.header.stamp = this->get_clock()->now();
    mag_msg.header.frame_id = "imu_link";

    Vector<3> mag = imu_sensor_.getVector(VECTOR_MAGNETOMETER);

    mag_msg.magnetic_field.x = mag[0];
    mag_msg.magnetic_field.y = mag[1];
    mag_msg.magnetic_field.z = mag[2];

    mag_publisher_->publish(mag_msg);
  }

  //   void timer_callback() {
  //     auto imu_msg = sensor_msgs::msg::Imu();
  //     // imu_msg.orientation.x = imu_sensor_.getQuat().x;

  //     Quaternion quat = imu_sensor_.getQuat();

  //     imu_msg.orientation.x = quat.x();
  //     imu_msg.orientation.y = quat.y();
  //     imu_msg.orientation.z = quat.z();
  //     imu_msg.orientation.w = quat.w();

  //     Vector<3> accel = imu_sensor_.getVector(VECTOR_ACCELEROMETER);

  //     imu_msg.linear_acceleration.x = accel.x();
  //     imu_msg.linear_acceleration.y = accel.y();
  //     imu_msg.linear_acceleration.z = accel.z();

  //     Vector<3> gyro = imu_sensor_.getVector(VECTOR_GYROSCOPE);

  //     imu_msg.angular_velocity.x = gyro.x();
  //     imu_msg.angular_velocity.y = gyro.y();
  //     imu_msg.angular_velocity.z = gyro.z();

  //     // magnetic field data
  //     auto mag_msg = sensor_msgs::msg::MagneticField();

  //     Vector<3> mag = imu_sensor_.getVector(VECTOR_MAGNETOMETER);

  //     mag_msg.magnetic_field.x = mag.x();
  //     mag_msg.magnetic_field.y = mag.y();
  //     mag_msg.magnetic_field.z = mag.z();

  //     imu_publisher_->publish(imu_msg);
  //     mag_publisher_->publish(mag_msg);
  //   }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IMUNode>());
  rclcpp::shutdown();
  return 0;
}
