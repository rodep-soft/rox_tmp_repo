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

class IMUNode : public rclcpp::Node {
 public:
  IMUNode() : Node("imu_node"), imu_sensor_(12345) {
    if (!imu_sensor_.begin()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize BNO055 sensor!");
      rclcpp::shutdown();
    }

    // BNO055座標系について:
    // センサーの物理的な取り付け方向によってX/Y/Z軸が決まる
    // 通常: X=前後, Y=左右, Z=上下（垂直軸）
    // ロボットで垂直回転（ヨー回転）を検出するには正しい軸の特定が必要
    RCLCPP_WARN(this->get_logger(), 
               "BNO055 IMU starting - Monitor logs to verify axis orientation");
    RCLCPP_WARN(this->get_logger(), 
               "Test: Rotate robot clockwise to see which axis shows positive values");

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

    // BNO055のオイラー角順序: euler[0]=Heading(Yaw), euler[1]=Roll, euler[2]=Pitch
    // 正しいRPY順序に並び替え
    rpy_msg.x = euler[1];  // Roll (元々はeuler[1])
    rpy_msg.y = euler[2];  // Pitch (元々はeuler[2])  
    rpy_msg.z = euler[0];  // Yaw/Heading (元々はeuler[0]) ← これが重要！

    // --- ここからImuメッセージ ---

    auto imu_accel_and_gyro_msg = sensor_msgs::msg::Imu();
    
    // Set header with current timestamp
    imu_accel_and_gyro_msg.header.stamp = this->now();
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

    // BNO055データの配列インデックス確認: gyro[0]=X, gyro[1]=Y, gyro[2]=Z（実装で確認済み）
    imu_accel_and_gyro_msg.angular_velocity.x = gyro[0];  // BNO055のX軸
    imu_accel_and_gyro_msg.angular_velocity.y = gyro[1];  // BNO055のY軸  
    imu_accel_and_gyro_msg.angular_velocity.z = gyro[2];  // BNO055のZ軸

    // === HARDWARE FAILURE DETECTION SYSTEM ===
    static int debug_counter = 0;
    static int hardware_failure_counter = 0;
    static bool z_axis_hardware_failure = false;
    debug_counter++;
    
    // Z軸ジャイロのハードウェア故障検出 (100 rad/s = 5730°/s以上は物理的に不可能)
    if (std::abs(gyro[2]) > 100.0) {
        hardware_failure_counter++;
        if (hardware_failure_counter > 10) {  // 10回連続で異常値なら故障判定
            if (!z_axis_hardware_failure) {
                RCLCPP_ERROR(this->get_logger(),
                           "HARDWARE FAILURE DETECTED: Z-axis gyroscope giving impossible values (%.1f rad/s = %.0f°/s)",
                           gyro[2], gyro[2] * 180.0 / M_PI);
                RCLCPP_ERROR(this->get_logger(),
                           "SWITCHING TO AXIS FALLBACK: Will force Z-axis to 0.0 and rely on X/Y axes");
                z_axis_hardware_failure = true;
            }
        }
    } else {
        hardware_failure_counter = 0;  // 正常値でカウンターリセット
    }
    
    // ハードウェア故障時はZ軸を強制的に0にする
    if (z_axis_hardware_failure) {
        imu_accel_and_gyro_msg.angular_velocity.z = 0.0;  // 故障軸を0に上書き
        // 静的カウンターで定期的に警告
        static int failure_warning_counter = 0;
        failure_warning_counter++;
        if (failure_warning_counter % 500 == 0) {  // 5秒ごと
            RCLCPP_WARN(this->get_logger(),
                       "Z-AXIS HARDWARE FAILURE: Raw=%.1f°/s, Publishing=0.0°/s (FORCED)",
                       gyro[2] * 180.0 / M_PI);
        }
    }
    
    double max_angular_vel = std::max({std::abs(gyro[0]), std::abs(gyro[1]), std::abs(gyro[2])});
    
    if (max_angular_vel > 0.5) {  // 0.5 rad/s = 約29°/s以上
      RCLCPP_WARN(this->get_logger(),
                 "BNO055 ROTATION: X=%.3f, Y=%.3f, Z=%.3f rad/s | Max=%.3f (%.1f°/s)",
                 gyro[0], gyro[1], gyro[2], max_angular_vel, max_angular_vel * 180.0 / M_PI);
      RCLCPP_WARN(this->get_logger(),
                 "AXIS TEST: Manually rotate robot clockwise - which axis shows POSITIVE values?");
    }
    
    // 定期的に全軸の値を表示（軸の対応関係確認用）
    if (debug_counter % 1000 == 0) {  // 10秒間隔
      RCLCPP_WARN(this->get_logger(),
                 "=== BNO055 COORDINATE SYSTEM ANALYSIS ===");
      RCLCPP_WARN(this->get_logger(),
                 "GYRO: X=%.3f, Y=%.3f, Z=%.3f rad/s", gyro[0], gyro[1], gyro[2]);
      RCLCPP_WARN(this->get_logger(),
                 "ACCEL: X=%.2f, Y=%.2f, Z=%.2f m/s² (gravity should be ~-9.8 on vertical axis)",
                 accel[0], accel[1], accel[2]);
      
      // 重力方向から垂直軸を特定
      double gravity_x = std::abs(accel[0]);
      double gravity_y = std::abs(accel[1]); 
      double gravity_z = std::abs(accel[2]);
      
      if (gravity_z > 8.0 && gravity_z > gravity_x && gravity_z > gravity_y) {
        RCLCPP_WARN(this->get_logger(), "ANALYSIS: Z-axis is VERTICAL (standard orientation)");
      } else if (gravity_x > 8.0 && gravity_x > gravity_y && gravity_x > gravity_z) {
        RCLCPP_WARN(this->get_logger(), "ANALYSIS: X-axis is VERTICAL (sensor rotated 90°)");
      } else if (gravity_y > 8.0 && gravity_y > gravity_x && gravity_y > gravity_z) {
        RCLCPP_WARN(this->get_logger(), "ANALYSIS: Y-axis is VERTICAL (sensor rotated 90°)");
      } else {
        RCLCPP_WARN(this->get_logger(), "ANALYSIS: Unable to determine vertical axis clearly");
      }
      
      RCLCPP_WARN(this->get_logger(),
                 "COORDINATE: gyro[0]=X_axis, gyro[1]=Y_axis, gyro[2]=Z_axis (verified in code)");
    }

    imu_publisher_->publish(imu_accel_and_gyro_msg);
    rpy_publisher_->publish(rpy_msg);
  }

  void timer_mag_callback() {
    auto mag_msg = sensor_msgs::msg::MagneticField();
    
    // Set header with current timestamp
    mag_msg.header.stamp = this->now();
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
