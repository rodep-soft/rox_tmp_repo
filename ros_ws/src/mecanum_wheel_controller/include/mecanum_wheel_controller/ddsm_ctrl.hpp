#ifndef _DDSM_CTRL_H
#define _DDSM_CTRL_H

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <cstdint>
#include <string>
#include <chrono>
#include <vector>
#include <memory>

#define DDSM_BAUDRATE 115200

#define TYPE_DDSM115  1
#define TIME_BETWEEN_CMD 4
#define TIMEOUT_MS 4

// DDSM115 Control Modes
#define DDSM115_CURRENT_MODE  1
#define DDSM115_SPEED_MODE    2
#define DDSM115_POSITION_MODE 3


class DDSM_CTRL {
public:
    DDSM_CTRL();
    ~DDSM_CTRL();
    
    // Initialize serial connection
    bool init(const std::string& port_name, unsigned int baud_rate = DDSM_BAUDRATE);
    void close();
    
    // Core functions
    void clear_ddsm_buffer();
    uint8_t crc8_update(uint8_t crc, uint8_t data);
    int set_ddsm_type(int inputType);
    int ddsm_id_check();
    int ddsm_change_id(uint8_t id);
    void ddsm_change_mode(uint8_t id, uint8_t mode);
    void ddsm_ctrl(uint8_t id, int cmd, uint8_t acceleration_time);
    void ddsm_ctrl(uint8_t id, int cmd, uint8_t acceleration_time, uint8_t brake);
    void ddsm_get_info(uint8_t id);
    void ddsm_stop(uint8_t id);
    int ddsm115_fb();

private:
    const size_t packet_length;   
    uint8_t packet_move[10];
    bool get_info_flag;
    
    // Boost.ASIO components
    boost::asio::io_context io_context;
    std::unique_ptr<boost::asio::serial_port> serial_port;
    
    // Helper functions
    bool write_data(const uint8_t* data, size_t length);
    bool read_data(uint8_t* buffer, size_t length, int timeout_ms = TIMEOUT_MS);
    std::chrono::steady_clock::time_point get_current_time();
    bool is_timeout(const std::chrono::steady_clock::time_point& start_time, int timeout_ms);
    void sleep_ms(int ms);

public:
    // Data members - sensor readings (DDSM115)
    int speed_data;      // Current speed
    int temperature;     // Temperature (from info request)
    int ddsm_mode;       // Current control mode
    int ddsm_torque;     // Torque output
    int ddsm_u8;         // Additional status data (from info request)
    int ddsm_pos;        // Current position
    int fault_code;      // Error/fault code
};

#endif