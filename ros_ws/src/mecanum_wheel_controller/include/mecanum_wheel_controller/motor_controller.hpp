#pragma once

#include <atomic>
// #include <bit>
#include <boost/asio.hpp>
#include <cmath>
// #include <future>
#include <string>
#include <vector>
#include <memory>

// ros2
#include <rclcpp/rclcpp.hpp>

class MotorController {
    public:
        // Constructor
        MotorController(rclcpp::Logger logger);
        // Destructor
        ~MotorController();

        bool init_port(const std::string& port_name, int baud_rate);

        // Motor control functions
        void send_velocity_command(uint8_t motor_id, int16_t rpm, bool brake = false);
        void send_velocity_commands_sequential(const std::vector<std::pair<uint8_t, int16_t>>& commands, bool brake = false);

        // Helper functions
        void clear_serial_buffer();
        void start_async_read();
        void parse_buffer();
        void process_feedback_packet(const std::vector<uint8_t>& packet);
        void wait_for_feedback_response(uint8_t motor_id, int timeout_ms = 20);
        bool wait_for_motor_response(uint8_t motor_id, int timeout_ms = 50);
        bool validate_motor_response(const std::vector<uint8_t>& response, uint8_t expected_id);

        
    private:
        boost::asio::io_context io_context_;
        boost::asio::serial_port serial_port_;
        rclcpp::Logger logger_;

        std::vector<uint8_t> buffer_;
        std::array<uint8_t, 64> read_buf_;
        std::atomic<bool> reading_;
        std::thread io_thread_;

        std::chrono::stady_clock::time_point feedback_received_time_;
        std::atomic<uint8_t> last_motor_id_{0};
};