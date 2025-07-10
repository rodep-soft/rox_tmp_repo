#include <gpiod.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <memory>

constexpr const char* CHIP_NAME = "gpiochip0"; // BCM0–27 = gpiochip4 on RPi 5の場合は変更が必要　<- ???
constexpr int TRIG_LINE = 6;  // BCM6
constexpr int ECHO_LINE = 5;  // BCM5

constexpr int CYCLE_MS = 500;
constexpr int TIMEOUT_MS = 100;
constexpr double SOUND_SPEED_CM_PER_US = 0.0343; // 343 m/s = 0.0343 cm/μs
constexpr int MAX_DISTANCE_CM = 400; // HC-SR04の最大測定距離

class UltrasonicSensor {
private:
    gpiod_chip* chip_;
    gpiod_line* trig_;
    gpiod_line* echo_;
    bool initialized_;

public:
    UltrasonicSensor() : chip_(nullptr), trig_(nullptr), echo_(nullptr), initialized_(false) {}
    
    ~UltrasonicSensor() {
        cleanup();
    }

    bool initialize() {
        // GPIO チップを開く
        chip_ = gpiod_chip_open_by_name(CHIP_NAME);
        if (!chip_) {
            std::cerr << "Error: Failed to open GPIO chip " << CHIP_NAME << std::endl;
            return false;
        }

        // GPIO ラインを取得
        trig_ = gpiod_chip_get_line(chip_, TRIG_LINE);
        echo_ = gpiod_chip_get_line(chip_, ECHO_LINE);
        
        if (!trig_ || !echo_) {
            std::cerr << "Error: Failed to get GPIO lines" << std::endl;
            cleanup();
            return false;
        }

        // GPIO ラインを設定
        if (gpiod_line_request_output(trig_, "hcsr04_trig", 0) < 0) {
            std::cerr << "Error: Failed to request trigger line as output" << std::endl;
            cleanup();
            return false;
        }
        
        if (gpiod_line_request_input(echo_, "hcsr04_echo") < 0) {
            std::cerr << "Error: Failed to request echo line as input" << std::endl;
            cleanup();
            return false;
        }

        initialized_ = true;
        return true;
    }

    long measureDistance() {
        if (!initialized_) {
            std::cerr << "Error: Sensor not initialized" << std::endl;
            return -1;
        }

        // トリガーパルスを送信（10μs）
        if (gpiod_line_set_value(trig_, 1) < 0) {
            std::cerr << "Error: Failed to set trigger high" << std::endl;
            return -1;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(10));
        
        if (gpiod_line_set_value(trig_, 0) < 0) {
            std::cerr << "Error: Failed to set trigger low" << std::endl;
            return -1;
        }

        // エコー信号の立ち上がりエッジを待機
        auto timeout = std::chrono::milliseconds(TIMEOUT_MS);
        auto start_wait = std::chrono::high_resolution_clock::now();
        
        while (gpiod_line_get_value(echo_) == 0) {
            if (std::chrono::high_resolution_clock::now() - start_wait > timeout) {
                std::cerr << "Timeout: No echo rising edge detected" << std::endl;
                return -1;
            }
        }
        auto echo_start = std::chrono::high_resolution_clock::now();

        // エコー信号の立ち下がりエッジを待機
        while (gpiod_line_get_value(echo_) == 1) {
            if (std::chrono::high_resolution_clock::now() - echo_start > timeout) {
                std::cerr << "Timeout: Echo signal too long" << std::endl;
                return -1;
            }
        }
        auto echo_end = std::chrono::high_resolution_clock::now();

        // 距離を計算
        auto duration_us = std::chrono::duration_cast<std::chrono::microseconds>(echo_end - echo_start).count();
        double distance_cm = duration_us * SOUND_SPEED_CM_PER_US / 2.0;
        
        // 範囲チェック
        if (distance_cm > MAX_DISTANCE_CM) {
            std::cerr << "Warning: Distance exceeds sensor range" << std::endl;
            return -1;
        }

        return static_cast<long>(distance_cm);
    }

private:
    void cleanup() {
        if (chip_) {
            gpiod_chip_close(chip_);
            chip_ = nullptr;
        }
        trig_ = nullptr;
        echo_ = nullptr;
        initialized_ = false;
    }
};

int main() {
    std::cout << "HC-SR04 Ultrasonic Sensor Distance Measurement" << std::endl;
    std::cout << "===============================================" << std::endl;
    
    UltrasonicSensor sensor;
    if (!sensor.initialize()) {
        std::cerr << "Failed to initialize ultrasonic sensor" << std::endl;
        return 1;
    }

    std::cout << "Sensor initialized successfully. Starting measurements..." << std::endl;
    std::cout << "Press Ctrl+C to stop." << std::endl << std::endl;

    int measurement_count = 0;
    while (true) {
        auto loop_start = std::chrono::steady_clock::now();

        long distance = sensor.measureDistance();
        measurement_count++;
        
        if (distance == -1) {
            std::cout << "[" << measurement_count << "] Measurement failed" << std::endl;
        } else {
            std::cout << "[" << measurement_count << "] Distance: " << distance << " cm" << std::endl;
        }

        // 一定周期を維持
        auto elapsed = std::chrono::steady_clock::now() - loop_start;
        auto sleep_time = std::chrono::milliseconds(CYCLE_MS) - elapsed;
        if (sleep_time > std::chrono::milliseconds(0)) {
            std::this_thread::sleep_for(sleep_time);
        }
    }

    return 0;
}

