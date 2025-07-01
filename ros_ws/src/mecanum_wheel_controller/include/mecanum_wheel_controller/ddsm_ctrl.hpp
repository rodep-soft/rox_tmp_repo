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
#define TYPE_DDSM210  2
#define TIME_BETWEEN_CMD 4
#define TIMEOUT_MS 4


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
    void ddsm_ctrl(uint8_t id, int cmd, uint8_t act);
    void ddsm_get_info(uint8_t id);
    void ddsm_stop(uint8_t id);
    int ddsm210_fb();
    int ddsm115_fb();

private:
    const size_t packet_length;   
    uint8_t packet_move[10];
    uint8_t ddsm_type;
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
    // Data members - sensor readings
    int speed_data;  // 115 210
    int current;     // 210
    int acceleration_time; // 210
    int temperature; // 115[info] 210

    int ddsm_mode;   // 115
    int ddsm_torque; // 115
    int ddsm_u8;     // 115[info]

    int32_t mileage; // 210[info]
    int ddsm_pos;    // 115 210[info]

    int fault_code;  // 115 210
};

#endif