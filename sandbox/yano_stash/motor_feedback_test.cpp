#include <iostream>
#include <boost/asio.hpp>
#include <vector>
#include <optional>
#include <cstdint>
#include <iomanip>
#include <thread>
#include <chrono>

class MotorController {
  public:
    MotorController(const std::string& port_name) : port_(io_context_, port_name) {
      init();

    }

    void init() {
      port_.set_option(boost::asio::serial_port_base::baud_rate(115200)); 
      port_.set_option(boost::asio::serial_port_base::character_size(8));
      port_.set_option(boost::asio::serial_port_base::flow_control(
          boost::asio::serial_port_base::flow_control::none
      ));
      port_.set_option(boost::asio::serial_port_base::parity(
          boost::asio::serial_port_base::parity::none
      ));
      port_.set_option(boost::asio::serial_port_base::stop_bits(
          boost::asio::serial_port_base::stop_bits::one
      ));
    }

    void motor_control(bool brake = false) {
      std::vector<uint8_t> data;

      data.push_back(0x01);
      data.push_back(0x64);

      uint16_t val_u16 = static_cast<uint16_t>(100);  // 符号付きを符号なしに変換

      data.push_back(static_cast<uint8_t>((val_u16 >> 8) & 0xFF));  // Highバイト
      data.push_back(static_cast<uint8_t>(val_u16 & 0xFF));         // Lowバイト

      data.push_back(0x00);
      data.push_back(0x00);
      data.push_back(0x00);

      if (brake) {
        data.push_back(0xFF); 
      } else {
        data.push_back(0x00);
      }

      data.push_back(0x00);
      data.push_back(calc_crc8_maxim(data));

      boost::asio::write(port_, boost::asio::buffer(data, data.size()));

      std::this_thread::sleep_for(std::chrono::milliseconds(3)); // Wait for the command to be processed

      if (auto response = receive_feedback()) {
        const auto& feedback = *response;
        uint8_t id = feedback[0];
        uint8_t mode = feedback[1];
        int16_t current = static_cast<int16_t>((feedback[2] << 8) | feedback[3]);
        int16_t speed = static_cast<int16_t>((feedback[4] << 8) | feedback[5]);
        uint16_t position = static_cast<uint16_t>((feedback[6] << 8) | feedback[7]);
        uint8_t error = feedback[8];
        std::cout << "Motor ID: " << static_cast<int>(id) 
                  << ", Mode: " << static_cast<int>(mode) 
                  << ", Current: " << current 
                  << ", Speed: " << speed 
                  << ", Position: " << position 
                  << ", Error: " << static_cast<int>(error) 
                  << std::endl;

      } else {
        std::cerr << "No valid response received." << std::endl;
      }

    }

    std::optional<std::vector<uint8_t>> receive_feedback() {
      std::vector<uint8_t> feedback(10);
      boost::asio::read(port_, boost::asio::buffer(feedback, feedback.size()));


      try {
        uint8_t crc = calc_crc8_maxim(std::vector<uint8_t>(feedback.begin(), feedback.begin() + 9));
        if (crc != feedback[9]) {
          std::cerr << "CRC mismatch" << std::endl;
          return std::nullopt;
        } else {
          std::cout << "CRC match" << std::endl;
        }

        return feedback;
      } catch (const std::exception& e) {
        std::cerr << "Failed to read from serial port: " << e.what() << std::endl;
        return std::nullopt;
      }


    }

    uint8_t calc_crc8_maxim(const std::vector<uint8_t>& data) {
      uint8_t crc = 0x00;  // 初期値  (一般的なMaxim CRCの標準)

      const uint8_t reflected_polynomial = 0x8C;

      // データバイトを一つずつ処理
      for (size_t i = 0; i < data.size();
          i++) {        // DATA[0]~DATA[8]まで、合計9バイト
        crc ^= data[i];  // 現在のバイトとCRCレジスタをXOR

        // 各バイトの8ビットを処理 (LSB First)
        for (uint8_t bit = 0; bit < 8; bit++) {
          if (crc & 0x01) {                           // 最下位ビットが1の場合
            crc = (crc >> 1) ^ reflected_polynomial;  // 右シフトして多項式とXOR
          } else {
            crc >>= 1;  // 最下位ビットが0の場合、単に右シフト
          }
        }
      }
      return crc;
    }

  private:
    boost::asio::io_context io_context_;
    boost::asio::serial_port port_;
};


int main(int argc, char** argv) {
  MotorController motor_controller("/dev/ttyACM0");
  motor_controller.motor_control(true);
  std::this_thread::sleep_for(std::chrono::seconds(3));
  motor_controller.motor_control(false);
  std::this_thread::sleep_for(std::chrono::seconds(3));
  motor_controller.motor_control(true);
  return 0;
}
