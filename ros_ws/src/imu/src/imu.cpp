#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include "imu/imulib.h"

#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <memory>
#include <functional>

class IMUNode : public rclcpp::Node {
    public:
        IMUNode(): Node("imu_node"), imu_sensor_(12345) {

            if (!imu_sensor_.begin()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to initialize BNO055 sensor!");
                rclcpp::shutdown();
            }
            
            // Publishers
            imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
            mag_publisher_ = this->create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", 10);
            
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&IMUNode::timer_callback, this)
            );

        }

    private:
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        BNO055 imu_sensor_; // Example sensor ID

        void timer_callback() {
            auto imu_msg = sensor_msgs::msg::Imu();
            // imu_msg.orientation.x = imu_sensor_.getQuat().x;

            Quaternion quat = imu_sensor_.getQuat();

            imu_msg.orientation.x = quat.x();
            imu_msg.orientation.y = quat.y();
            imu_msg.orientation.z = quat.z();
            imu_msg.orientation.w = quat.w();

            Vector<3> accel = imu_sensor_.getVector(VECTOR_ACCELEROMETER);

            imu_msg.linear_acceleration.x = accel.x();
            imu_msg.linear_acceleration.y = accel.y();
            imu_msg.linear_acceleration.z = accel.z();

            Vector<3> gyro = imu_sensor_.getVector(VECTOR_GYROSCOPE);

            imu_msg.angular_velocity.x = gyro.x();
            imu_msg.angular_velocity.y = gyro.y();
            imu_msg.angular_velocity.z = gyro.z();

            auto mag_msg = sensor_msgs::msg::MagneticField();

            Vector<3> mag = imu_sensor_.getVector(VECTOR_MAGNETOMETER);

            mag_msg.magnetic_field.x = mag.x();
            mag_msg.magnetic_field.y = mag.y();
            mag_msg.magnetic_field.z = mag.z();


            imu_publisher_->publish(imu_msg);
            mag_publisher_->publish(mag_msg);

        }

};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMUNode>());
    rclcpp::shutdown();
    return 0;
}