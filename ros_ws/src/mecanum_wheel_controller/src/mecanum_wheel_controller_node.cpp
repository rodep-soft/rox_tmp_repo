#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>
#include <memory>
#include <boost/asio.hpp>
#include <array>
#include <string>
#include <iostream>
#include <algorithm>

class MotorController {
    public:
            MotorController() : serial_port_(io_context_) {}

            void init_port(const std::string& port_name) {
                try {
                    serial_port_.open(port_name);

                    serial_port_.set_option(boost::asio::serial_port_base::baud_rate(115200));
                    serial_port_.set_option(boost::asio::serial_port_base::character_size(8));
                    serial_port_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
                    serial_port_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
                    serial_port_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
                } catch (const std::exception& e) {
                    throw std::runtime_error("Failed to initialize serial port: " + std::string(e.what()));
                }
            }

            void control_with_velocity(const uint8_t id = 0x01, const int16_t rpm = 0, bool brake = false) {
                send_data_[0] = id;
                send_data_[1] = 0x64;
                
                uint16_t val_u16 = static_cast<uint16_t>(rpm); // 符号付きを符号なしに変換

                send_data_[2] = static_cast<uint8_t>((val_u16 >> 8) & 0xFF); // High
                send_data_[3] = static_cast<uint8_t>(val_u16 & 0xFF); // Low

                send_data_[4] = 0x00;
                send_data_[5] = 0x00;
                send_data_[6] = 0x00;

                if (brake) {
                    send_data_[7] = 0xFF;
                } else {
                    send_data_[7] = 0x00;
                }

                send_data_[8] = 0x00;

                send_data_[9] = calc_crc8_maxim({send_data_.begin(), send_data_.begin() + 9});

                try {
                    boost::asio::write(serial_port_, boost::asio::buffer(send_data_, send_data_.size()));
                } catch (const std::exception& e) {
                    // エラーログを出力するが、例外は再投げしない（ロボット制御の継続性のため）
                    // RCLCPP_ERROR を使用（this->get_logger()は利用できないため、静的にログを出力）
                    std::cerr << "Failed to write to serial port: " << e.what() << std::endl;
                }
            }

    private:
        boost::asio::io_context io_context_;
        boost::asio::serial_port serial_port_;
        std::array<uint8_t, 10> send_data_;



        uint8_t calc_crc8_maxim(const std::array<uint8_t, 9>& data) {
            uint8_t crc = 0x00; // 初期値  (一般的なMaxim CRCの標準)

            const uint8_t reflected_polynomial = 0x8C; 

            // データバイトを一つずつ処理
            for (size_t i = 0; i < data.size(); i++) { // DATA[0]~DATA[8]まで、合計9バイト
                crc ^= data[i]; // 現在のバイトとCRCレジスタをXOR

                // 各バイトの8ビットを処理 (LSB First)
                for (uint8_t bit = 0; bit < 8; bit++) {
                    if (crc & 0x01) { // 最下位ビットが1の場合
                        crc = (crc >> 1) ^ reflected_polynomial; // 右シフトして多項式とXOR
                    } else {
                        crc >>= 1; // 最下位ビットが0の場合、単に右シフト
                    }
                }
            }
            return crc;
        }
};

class MecanumKinematicsNode : public rclcpp::Node
{
public:
    MecanumKinematicsNode()
        : Node("mecanum_kinematics_node")
    {
        // ROS 2 パラメータを宣言し、デフォルト値を設定
        this->declare_parameter<double>("wheel_radius", 0.05); // ホイール半径 (m)
        this->declare_parameter<double>("wheel_base_x", 0.2);  // lx: ロボット中心から左右ホイールまでの距離の半分 (m)
        this->declare_parameter<double>("wheel_base_y", 0.2);  // ly: ロボット中心から前後ホイールまでの距離の半分 (m)

        this->declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
  
        // パラメータを取得
        this->get_parameter("wheel_radius", wheel_radius_);
        this->get_parameter("wheel_base_x", wheel_base_x_);
        this->get_parameter("wheel_base_y", wheel_base_y_);
        
        // パラメータの妥当性チェック
        if (wheel_radius_ <= 0.0 || wheel_base_x_ <= 0.0 || wheel_base_y_ <= 0.0) {
            RCLCPP_ERROR(this->get_logger(), "Invalid robot parameters detected");
            throw std::invalid_argument("Robot parameters must be positive values");
        }
        
        std::string serial_port;
        this->get_parameter("serial_port", serial_port);
        
        // モーターコントローラーの初期化
        try {
            motor_controller_.init_port(serial_port);
            RCLCPP_INFO(this->get_logger(), "Serial port initialized: %s", serial_port.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize serial port: %s", e.what());
            // 必要に応じてここで例外を再投げするか、代替処理を行う
        }
        
        lxy_sum_ = wheel_base_x_ + wheel_base_y_;

        // commented out for debugging purposes
        // RCLCPP_INFO(this->get_logger(), "Mecanum Kinematics Node started with parameters:");
        // RCLCPP_INFO(this->get_logger(), "Wheel Radius (r): %.3f m", wheel_radius_);
        // RCLCPP_INFO(this->get_logger(), "Wheel Base X (lx): %.3f m", wheel_base_x_);
        // RCLCPP_INFO(this->get_logger(), "Wheel Base Y (ly): %.3f m", wheel_base_y_);

        // Twistメッセージを購読するためのSubscriptionを作成
        // トピック名 /cmd_vel はジョイスティックやナビゲーションで一般的に使われる
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&MecanumKinematicsNode::velocity_callback, this, std::placeholders::_1));

    }

private:
    MotorController motor_controller_;

    enum class MotorIndex : uint8_t {
        FRONT_RIGHT = 0,
        FRONT_LEFT = 1,
        BACK_RIGHT = 2,
        BACK_LEFT = 3
    };

    static constexpr size_t FR = static_cast<size_t>(MotorIndex::FRONT_RIGHT);
    static constexpr size_t FL = static_cast<size_t>(MotorIndex::FRONT_LEFT);
    static constexpr size_t BR = static_cast<size_t>(MotorIndex::BACK_RIGHT);
    static constexpr size_t BL = static_cast<size_t>(MotorIndex::BACK_LEFT);


    // Helper
    static constexpr uint8_t motor_id(MotorIndex index) {
        return static_cast<uint8_t>(index) + 1;
    }


    std::array<double, 4> wheel_vel_rad_per_s_;
    std::array<int16_t, 4> wheel_vel_rpm_;
    
    const double rad_s_to_rpm_factor = 60.0 / (2.0 * M_PI);

    int16_t rad_per_s_to_rpm(double rad_per_s) {
        const double rpm = rad_per_s * rad_s_to_rpm_factor;
        return static_cast<int16_t>(std::round(rpm));
    }



    void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Twistメッセージから速度を取得
        const double vx = msg->linear.x;
        const double vy = msg->linear.y;
        const double wz = msg->angular.z;


        // 逆運動学の計算 (rad/s)
        // 論文の式展開: ω = (1/r) * [vx ± vy ± (lx+ly)wz]
        wheel_vel_rad_per_s_[FR] = (1.0 / wheel_radius_) * (vx - vy - lxy_sum_ * wz);
        wheel_vel_rad_per_s_[FL] = (1.0 / wheel_radius_) * (vx + vy + lxy_sum_ * wz);
        wheel_vel_rad_per_s_[BR] = (1.0 / wheel_radius_) * (vx + vy - lxy_sum_ * wz);
        wheel_vel_rad_per_s_[BL] = (1.0 / wheel_radius_) * (vx - vy + lxy_sum_ * wz);

        // rad/s から RPM への変換
        for (size_t i = 0; i < wheel_vel_rpm_.size(); ++i) {
            wheel_vel_rpm_[i] = rad_per_s_to_rpm(wheel_vel_rad_per_s_[i]);
        }
        
        RCLCPP_DEBUG(this->get_logger(), "RPMs: [FR: %d, FL: %d, BR: %d, BL: %d]",
            wheel_vel_rpm_[0], wheel_vel_rpm_[1], wheel_vel_rpm_[2], wheel_vel_rpm_[3]);

        // 計算済みのRPM値を使用してモーター制御
        motor_controller_.control_with_velocity(motor_id(MotorIndex::FRONT_RIGHT), wheel_vel_rpm_[FR]);
        motor_controller_.control_with_velocity(motor_id(MotorIndex::FRONT_LEFT), wheel_vel_rpm_[FL]);
        motor_controller_.control_with_velocity(motor_id(MotorIndex::BACK_RIGHT), wheel_vel_rpm_[BR]);
        motor_controller_.control_with_velocity(motor_id(MotorIndex::BACK_LEFT), wheel_vel_rpm_[BL]);

    }
    

    // ROS 2 サブスクライバとパブリッシャ
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;

    // ロボットのパラメータ
    double wheel_radius_;
    double wheel_base_x_;
    double wheel_base_y_;
    double lxy_sum_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MecanumKinematicsNode>());
    rclcpp::shutdown();
    return 0;
}