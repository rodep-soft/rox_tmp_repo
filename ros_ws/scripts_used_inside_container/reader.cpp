// コンパイル方法
// g++ -std=c++17 -O2 -Wall reader.cpp -lboost_system -lpthread


#include <iostream>
#include <vector>
#include <boost/asio.hpp>
#include <thread>
#include <atomic>
#include <chrono>

// CRC8 Maxim計算関数
uint8_t calc_crc8_maxim(const std::vector<uint8_t>& data) {
  uint8_t crc = 0x00;
  const uint8_t reflected_polynomial = 0x8C;

  for (size_t i = 0; i < data.size(); i++) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (crc & 0x01) {
        crc = (crc >> 1) ^ reflected_polynomial;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

class MotorReceiver {
public:
  MotorReceiver(boost::asio::io_context& io, const std::string& port, unsigned int baud)
    : serial_port_(io, port), buffer_(), running_(true)
  {
    serial_port_.set_option(boost::asio::serial_port_base::baud_rate(baud));
    serial_port_.set_option(boost::asio::serial_port_base::character_size(8));
    serial_port_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serial_port_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    serial_port_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
  }

  void start() {
    do_read();
  }

  void stop() {
    running_ = false;
    serial_port_.close();
  }

private:
  void do_read() {
    if (!running_) return;

    serial_port_.async_read_some(boost::asio::buffer(read_buf_),
      [this](boost::system::error_code ec, std::size_t length) {
        if (!ec && length > 0) {
          buffer_.insert(buffer_.end(), read_buf_.begin(), read_buf_.begin() + length);
          parse_buffer();
          do_read();
        } else {
          if (ec) {
            std::cerr << "Read error: " << ec.message() << std::endl;
          }
        }
      });
  }

  void parse_buffer() {
    // 10バイトパケット単位でチェック
    while (buffer_.size() >= 10) {
      // パケット開始候補を探す(IDは1~4の範囲)
      size_t pos = 0;
      for (; pos <= buffer_.size() - 10; ++pos) {
        if (buffer_[pos] >= 1 && buffer_[pos] <= 4) {
          // CRC8チェック
          std::vector<uint8_t> packet_data(buffer_.begin() + pos, buffer_.begin() + pos + 9);
          uint8_t crc_calculated = calc_crc8_maxim(packet_data);
          uint8_t crc_received = buffer_[pos + 9];
          if (crc_calculated == crc_received) {
            break; // 正しいパケット発見
          }
        }
      }

      if (pos > 0) {
        // 不正な先頭バイトがあれば捨てる
        buffer_.erase(buffer_.begin(), buffer_.begin() + pos);
      }

      if (buffer_.size() < 10) break;

      // 10バイトパケット取り出し
      std::vector<uint8_t> packet(buffer_.begin(), buffer_.begin() + 10);

      // パースして表示
      uint8_t motor_id = packet[0];
      uint8_t mode_value = packet[1];
      // トルク電流(16bit符号付き)
      int16_t torque_current = (static_cast<int16_t>(packet[2]) << 8) | packet[3];
      // 速度(16bit符号付き)
      int16_t velocity = (static_cast<int16_t>(packet[4]) << 8) | packet[5];
      // 位置(16bit符号付き？ただしunsignedっぽい場合も)
      uint16_t position = (static_cast<uint16_t>(packet[6]) << 8) | packet[7];
      uint8_t error_code = packet[8];

      std::cout << "Motor ID: " << +motor_id
                << ", Mode: " << +mode_value
                << ", Torque Current: " << torque_current
                << ", Velocity: " << velocity
                << ", Position: " << position
                << ", Error: 0x" << std::hex << static_cast<int>(error_code) << std::dec
                << std::endl;

      // パケット消費
      buffer_.erase(buffer_.begin(), buffer_.begin() + 10);
    }
  }

  boost::asio::serial_port serial_port_;
  std::vector<uint8_t> buffer_;
  std::array<uint8_t, 64> read_buf_;
  std::atomic<bool> running_;
};

int main() {
  boost::asio::io_context io;
  std::string port_name = "/dev/ttyACM0";  // 環境に合わせて変えてね
  unsigned int baud_rate = 115200;

  try {
    MotorReceiver receiver(io, port_name, baud_rate);
    receiver.start();

    std::thread io_thread([&io]() {
      io.run();
    });

    std::cout << "Start receiving motor data... Ctrl+Cで停止\n";

    // Ctrl+Cまでメインスレッドを寝かせる
    std::this_thread::sleep_for(std::chrono::hours(24*365));

    receiver.stop();
    io_thread.join();

  } catch (std::exception& e) {
    std::cerr << "Exception: " << e.what() << std::endl;
  }

  return 0;
}
