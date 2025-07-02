#include "mecanum_wheel_controller/ddsm_ctrl.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <future>

DDSM_CTRL::DDSM_CTRL() 
    : packet_length(10),
      ddsm_type(TYPE_DDSM115), 
      get_info_flag(false),
      serial_port(nullptr)
{
    packet_move[0] = 0x01;
    packet_move[1] = 0x64;
    packet_move[2] = 0xff;
    packet_move[3] = 0xce;
    packet_move[4] = 0x00;
    packet_move[5] = 0x00;
    packet_move[6] = 0x00;
    packet_move[7] = 0x00;
    packet_move[8] = 0x00;
    packet_move[9] = 0xda;
    
    // Initialize data members
    speed_data = 0;
    temperature = 0;
    ddsm_mode = 0;
    ddsm_torque = 0;
    ddsm_u8 = 0;
    ddsm_pos = 0;
    fault_code = 0;
}

DDSM_CTRL::~DDSM_CTRL() {
    close();
}

bool DDSM_CTRL::init(const std::string& port_name, unsigned int baud_rate) {
    try {
        serial_port = std::make_unique<boost::asio::serial_port>(io_context, port_name);
        
        serial_port->set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
        serial_port->set_option(boost::asio::serial_port_base::character_size(8));
        serial_port->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        serial_port->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        serial_port->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
        
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error initializing serial port: " << e.what() << std::endl;
        return false;
    }
}

void DDSM_CTRL::close() {
    if (serial_port && serial_port->is_open()) {
        serial_port->close();
    }
    serial_port.reset();
}

// CRC-8/MAXIM
uint8_t DDSM_CTRL::crc8_update(uint8_t crc, uint8_t data) {
  uint8_t i;
  crc = crc ^ data;
  for (i = 0; i < 8; ++i) {
    if (crc & 0x01) {
      crc = (crc >> 1) ^ 0x8c;
    } else {
      crc >>= 1;
    }
  }
  return crc;
}

// config the type of ddsm.
int DDSM_CTRL::set_ddsm_type(int inputType) {
	if (inputType == 115 || inputType == TYPE_DDSM115) {
		ddsm_type = TYPE_DDSM115;
		return TYPE_DDSM115;
	}
	return -1;
}

// clear ddsm serial buffer.
void DDSM_CTRL::clear_ddsm_buffer() {
    if (!serial_port || !serial_port->is_open()) {
        return;
    }
    
    try {
        uint8_t dummy_buffer[256];
        boost::system::error_code ec;
        
        // Keep reading until no more data is available
        while (true) {
            size_t bytes_read = serial_port->read_some(boost::asio::buffer(dummy_buffer), ec);
            if (ec || bytes_read == 0) {
                break;
            }
        }
    } catch (const std::exception& e) {
        // Ignore errors during buffer clearing
    }
}

// feedback data from ddsm115.
int DDSM_CTRL::ddsm115_fb() {
    uint8_t data[10];
    
    if (!read_data(data, 10, TIMEOUT_MS)) {
        return -1;
    }

    uint8_t ddsm_id = data[0];

    // CRC-8/MAXIM
    uint8_t crc = 0;
    for (size_t i = 0; i < packet_length - 1; ++i) {
        crc = crc8_update(crc, data[i]);
    }
    if (crc != data[9]) {
        return -1;
    }

    ddsm_mode = data[1];

    ddsm_torque = (data[2] << 8) | data[3];
    if (ddsm_torque & 0x8000) {
        ddsm_torque = -(0x10000 - ddsm_torque);
    }

    speed_data = (data[4] << 8) | data[5];
    if (speed_data & 0x8000) {
        speed_data = -(0x10000 - speed_data);
    }

    if (get_info_flag) {
        get_info_flag = false;
        temperature = data[6];
        ddsm_u8 = data[7];
        fault_code = data[8];
    } else {
        ddsm_pos = (data[6] << 8) | data[7];
        fault_code = data[8];
    }
    return 1;
}


// check the ID of ddsm.
int DDSM_CTRL::ddsm_id_check() {
    packet_move[0] = 0xC8;
    packet_move[1] = 0x64;
    packet_move[2] = 0x00;
    packet_move[3] = 0x00;

    packet_move[4] = 0x00;
    packet_move[5] = 0x00;
    packet_move[6] = 0x00;
    packet_move[7] = 0x00;

    packet_move[8] = 0x00;
    packet_move[9] = 0xDE;
    
    if (!write_data(packet_move, packet_length)) {
        return -1;
    }

    uint8_t data[10];
    if (!read_data(data, 10, TIMEOUT_MS)) {
        return -1;
    }

    // CRC-8/MAXIM
    uint8_t crc = 0;
    for (size_t i = 0; i < packet_length - 1; ++i) {
        crc = crc8_update(crc, data[i]);
    }
    if (crc != data[9]) {
        return -1;
    }
    
    int feedback_type = data[1];
    uint8_t ID = data[0];
    return ID;
}

int DDSM_CTRL::ddsm_change_id(uint8_t id) {
    packet_move[0] = 0xAA;

    packet_move[1] = 0x55;
    packet_move[2] = 0x53;

    packet_move[3] = id;

    packet_move[4] = 0x00;
    packet_move[5] = 0x00;
    packet_move[6] = 0x00;
    packet_move[7] = 0x00;
    packet_move[8] = 0x00;

    // CRC-8/MAXIM
    uint8_t crc = 0;
    for (size_t i = 0; i < packet_length - 1; ++i) {
        crc = crc8_update(crc, packet_move[i]);
    }
    packet_move[9] = crc;

    for (int i = 0; i < 5; i++) {
        if (!write_data(packet_move, packet_length)) {
            return -1;
        }
        sleep_ms(TIME_BETWEEN_CMD);
    }

    ddsm_id_check();
    
    uint8_t data[10];
    if (!read_data(data, 10, TIMEOUT_MS)) {
        return -1;
    }

    // CRC-8/MAXIM
    crc = 0;
    for (size_t i = 0; i < packet_length - 1; ++i) {
        crc = crc8_update(crc, data[i]);
    }
    if (crc != data[9]) {
        return -1;
    }
    
    int feedback_type = data[1];
    uint8_t ID = data[0];

    if (id != ID) {
        return -1;
    } else {
        return ID;
    }
}

// change mode
// ddsm115:
// 1 - current loop
// 2 - speed loop
// 3 - position loop
void DDSM_CTRL::ddsm_change_mode(uint8_t id, uint8_t mode) {
    packet_move[0] = id;
    packet_move[1] = 0xA0;
    packet_move[2] = 0x00;
    packet_move[3] = 0x00;
    packet_move[4] = 0x00;
    packet_move[5] = 0x00;
    packet_move[6] = 0x00;
    packet_move[7] = 0x00;
    packet_move[8] = 0x00;
    packet_move[9] = mode;
    
    write_data(packet_move, packet_length);
}

// --- DDSM115 ---
// current loop, cmd: -32767 ~ 32767 -> -8 ~ 8 A (ddsm115 max current < 2.7A)
// speed loop, cmd: -200 ~ 200 rpm
// position loop, cmd: 0 ~ 32767 -> 0 ~ 360°

//    wherever the mode is set to position mode
//    the currently position is the 0 position and it moves to the goal position
//    at the direction as the shortest path.
void DDSM_CTRL::ddsm_ctrl(uint8_t id, int cmd, uint8_t acceleration_time) {
    ddsm_ctrl(id, cmd, acceleration_time, 0x00); // Default: no brake
}

void DDSM_CTRL::ddsm_ctrl(uint8_t id, int cmd, uint8_t acceleration_time, uint8_t brake) {
    packet_move[0] = id;
    packet_move[1] = 0x64;

    packet_move[2] = (cmd >> 8) & 0xFF;
    packet_move[3] = cmd & 0xFF;

    packet_move[4] = 0x00;
    packet_move[5] = 0x00;

    packet_move[6] = acceleration_time; // Acceleration time (0.1ms per rpm)
    packet_move[7] = brake;             // Brake (0xFF to brake, other values no brake)

    packet_move[8] = 0x00;              // Reserved

    // CRC-8/MAXIM
    uint8_t crc = 0;
    for (size_t i = 0; i < packet_length - 1; ++i) {
        crc = crc8_update(crc, packet_move[i]);
    }
    packet_move[9] = crc;

    write_data(packet_move, packet_length);

    ddsm115_fb();
}

void DDSM_CTRL::ddsm_get_info(uint8_t id) {  
    packet_move[0] = id;
    get_info_flag = true;

    packet_move[1] = 0x74;

    packet_move[2] = 0x00;
    packet_move[3] = 0x00;

    packet_move[4] = 0x00;
    packet_move[5] = 0x00;
    packet_move[6] = 0x00;
    packet_move[7] = 0x00;

    packet_move[8] = 0x00;
    // CRC-8/MAXIM
    uint8_t crc = 0;
    for (size_t i = 0; i < packet_length - 1; ++i) {
        crc = crc8_update(crc, packet_move[i]);
    }
    packet_move[9] = crc;
    
    write_data(packet_move, packet_length);
    
    ddsm115_fb();
}

void DDSM_CTRL::ddsm_stop(uint8_t id) {
    ddsm_ctrl(id, 0, 0);
    ddsm115_fb();
}

// Helper functions for serial communication
bool DDSM_CTRL::write_data(const uint8_t* data, size_t length) {
    if (!serial_port || !serial_port->is_open()) {
        return false;
    }
    
    try {
        boost::asio::write(*serial_port, boost::asio::buffer(data, length));
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error writing data: " << e.what() << std::endl;
        return false;
    }
}

bool DDSM_CTRL::read_data(uint8_t* buffer, size_t length, int timeout_ms) {
    if (!serial_port || !serial_port->is_open()) {
        return false;
    }
    
    auto start_time = get_current_time();
    
    try {
        // Use async read with timeout
        boost::system::error_code ec;
        std::future<size_t> read_result = std::async(std::launch::async, [&]() {
            return boost::asio::read(*serial_port, boost::asio::buffer(buffer, length), ec);
        });
        
        if (read_result.wait_for(std::chrono::milliseconds(timeout_ms)) == std::future_status::timeout) {
            return false;
        }
        
        size_t bytes_read = read_result.get();
        return !ec && bytes_read == length;
    } catch (const std::exception& e) {
        std::cerr << "Error reading data: " << e.what() << std::endl;
        return false;
    }
}

std::chrono::steady_clock::time_point DDSM_CTRL::get_current_time() {
    return std::chrono::steady_clock::now();
}

bool DDSM_CTRL::is_timeout(const std::chrono::steady_clock::time_point& start_time, int timeout_ms) {
    auto current_time = get_current_time();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time);
    return elapsed.count() >= timeout_ms;
}

void DDSM_CTRL::sleep_ms(int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}