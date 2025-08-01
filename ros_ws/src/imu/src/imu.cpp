#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <thread>
#include <vector>

#include "imu/imulib.h"

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

    accel_and_gyro_timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
                                    std::bind(&IMUNode::timer_accel_and_gyro_callback, this));

    mag_timer_ = this->create_wall_timer(std::chrono::milliseconds(50),
                                    std::bind(&IMUNode::timer_mag_callback, this));
  }

 private:
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_publisher_;
  rclcpp::TimerBase::SharedPtr accel_and_gyro_timer_;
  rclcpp::TimerBase::SharedPtr mag_timer_;

  BNO055 imu_sensor_;  // Example sensor ID

  void timer_accel_and_gyro_callback() {
    auto imu_accel_and_gyro_msg = sensor_msgs::msg::Imu();

    // Quaternion quat = imu_sensor_.getQuat();

    // imu_accel_and_gyro_msg.orientation.x = quat.x();
    // imu_accel_and_gyro_msg.orientation.y = quat.y();
    // imu_accel_and_gyro_msg.orientation.z = quat.z();
    // imu_accel_and_gyro_msg.orientation.w = quat.w();

    Vector<3> euler = imu_sensor_.getVector(VECTOR_EULER);

    imu_accel_and_gyro_msg.orientation.x = euler[0];
    imu_accel_and_gyro_msg.orientation.y = euler[1];
    imu_accel_and_gyro_msg.orientation.z = euler[2];

    Vector<3> accel = imu_sensor_.getVector(VECTOR_ACCELEROMETER);

    imu_accel_and_gyro_msg.linear_acceleration.x = accel[0];
    imu_accel_and_gyro_msg.linear_acceleration.y = accel[1];
    imu_accel_and_gyro_msg.linear_acceleration.z = accel[2];

    Vector<3> gyro = imu_sensor_.getVector(VECTOR_GYROSCOPE);

    imu_accel_and_gyro_msg.angular_velocity.x = gyro[0];
    imu_accel_and_gyro_msg.angular_velocity.y = gyro[1];
    imu_accel_and_gyro_msg.angular_velocity.z = gyro[2];

    imu_publisher_->publish(imu_accel_and_gyro_msg);
  }

  void timer_mag_callback() {
    auto mag_msg = sensor_msgs::msg::MagneticField();

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
