# DDSM_CTRL API Reference

## Overview

The `DDSM_CTRL` class provides a C++ interface for controlling DDSM (Digital DC Servo Motor) motors through serial communication. It supports both DDSM115 and DDSM210 motor types with various control modes including current loop, speed loop, and position loop control.

## Table of Contents
- [Class Definition](#class-definition)
- [Constructor & Destructor](#constructor--destructor)
- [Initialization & Connection](#initialization--connection)
- [Motor Control Methods](#motor-control-methods)
- [Configuration Methods](#configuration-methods)
- [Feedback & Monitoring](#feedback--monitoring)
- [Data Members](#data-members)
- [Constants](#constants)
- [Usage Examples](#usage-examples)

---

## Class Definition

```cpp
class DDSM_CTRL {
public:
    // Constructor & Destructor
    DDSM_CTRL();
    ~DDSM_CTRL();
    
    // Initialization & Connection
    bool init(const std::string& port_name, unsigned int baud_rate = DDSM_BAUDRATE);
    void close();
    
    // Configuration
    int set_ddsm_type(int inputType);
    void ddsm_change_mode(uint8_t id, uint8_t mode);
    int ddsm_change_id(uint8_t id);
    
    // Motor Control
    void ddsm_ctrl(uint8_t id, int cmd, uint8_t act);
    void ddsm_stop(uint8_t id);
    
    // Information & Feedback
    int ddsm_id_check();
    void ddsm_get_info(uint8_t id);
    int ddsm210_fb();
    int ddsm115_fb();
    
    // Utility
    void clear_ddsm_buffer();
    uint8_t crc8_update(uint8_t crc, uint8_t data);
    
    // Data members (sensor readings)
    int speed_data;         // Current speed (115/210)
    int current;            // Current consumption (210 only)
    int acceleration_time;  // Acceleration time (210 only) 
    int temperature;        // Temperature (115 info/210)
    int ddsm_mode;          // Current mode (115 only)
    int ddsm_torque;        // Torque (115 only)
    int ddsm_u8;            // Additional data (115 info only)
    int32_t mileage;        // Mileage counter (210 info only)
    int ddsm_pos;           // Position (115/210 info)
    int fault_code;         // Fault/error code (115/210)
};
```

---

## Constructor & Destructor

### `DDSM_CTRL()`
**Description:** Default constructor that initializes the DDSM controller object.

**Parameters:** None

**Behavior:**
- Initializes packet data structures
- Sets default motor type to `TYPE_DDSM115`
- Initializes all data members to zero
- Sets up default packet format

---

### `~DDSM_CTRL()`
**Description:** Destructor that cleans up resources and closes serial connection.

**Parameters:** None

**Behavior:**
- Automatically calls `close()` to terminate serial connection
- Cleans up allocated resources

---

## Initialization & Connection

### `bool init(const std::string& port_name, unsigned int baud_rate = DDSM_BAUDRATE)`
**Description:** Initializes serial communication with the DDSM motor controller.

**Parameters:**
- `port_name`: Serial port path (e.g., "/dev/ttyUSB0", "/dev/ttyACM0")
- `baud_rate`: Communication baud rate (default: 115200)

**Returns:**
- `true`: Initialization successful
- `false`: Initialization failed

**Example:**
```cpp
DDSM_CTRL motor;
if (motor.init("/dev/ttyUSB0", 115200)) {
    std::cout << "Motor initialized successfully" << std::endl;
} else {
    std::cerr << "Failed to initialize motor" << std::endl;
}
```

---

### `void close()`
**Description:** Closes the serial connection and releases resources.

**Parameters:** None

**Returns:** None

**Behavior:**
- Closes the serial port if open
- Resets internal serial port pointer

---

## Motor Control Methods

### `void ddsm_ctrl(uint8_t id, int cmd, uint8_t act)`
**Description:** Main motor control function that sends commands to a specific motor.

**Parameters:**
- `id`: Motor ID (1-255)
- `cmd`: Command value (range depends on mode and motor type)
- `act`: Action/enable flag (1 = enable, 0 = disable)

**Command Ranges by Mode:**

#### DDSM115:
- **Current loop:** cmd = -32767 to 32767 → -8A to 8A (max 2.7A)
- **Speed loop:** cmd = -200 to 200 rpm
- **Position loop:** cmd = 0 to 32767 → 0° to 360°

#### DDSM210:
- **Open loop:** cmd = -32767 to 32767
- **Speed loop:** cmd = -2100 to 2100 → -210 to 210 rpm  
- **Position loop:** cmd = 0 to 32767 → 0° to 360°

**Example:**
```cpp
// Speed control: rotate at 100 rpm
motor.ddsm_ctrl(1, 100, 1);  // Motor ID 1, 100 rpm, enable

// Position control: move to 180 degrees
motor.ddsm_ctrl(1, 16384, 1);  // Motor ID 1, 180°, enable
```

---

### `void ddsm_stop(uint8_t id)`
**Description:** Stops a specific motor by sending zero command.

**Parameters:**
- `id`: Motor ID to stop

**Returns:** None

**Example:**
```cpp
motor.ddsm_stop(1);  // Stop motor with ID 1
```

---

## Configuration Methods

### `int set_ddsm_type(int inputType)`
**Description:** Sets the motor type for proper command formatting.

**Parameters:**
- `inputType`: Motor type (115, 210, TYPE_DDSM115, or TYPE_DDSM210)

**Returns:**
- `TYPE_DDSM115` (1): Successfully set to DDSM115
- `TYPE_DDSM210` (2): Successfully set to DDSM210  
- `-1`: Invalid input type

**Example:**
```cpp
if (motor.set_ddsm_type(210) == TYPE_DDSM210) {
    std::cout << "Motor type set to DDSM210" << std::endl;
}
```

---

### `void ddsm_change_mode(uint8_t id, uint8_t mode)`
**Description:** Changes the control mode of a specific motor.

**Parameters:**
- `id`: Motor ID
- `mode`: Control mode

**Control Modes:**

#### DDSM115:
- `1`: Current loop control
- `2`: Speed loop control  
- `3`: Position loop control

#### DDSM210:
- `0`: Open loop control
- `2`: Speed loop control
- `3`: Position loop control

**Example:**
```cpp
// Set motor 1 to speed control mode
motor.ddsm_change_mode(1, 2);
```

---

### `int ddsm_change_id(uint8_t id)`
**Description:** Changes the ID of a connected motor (must be only one motor connected).

**Parameters:**
- `id`: New motor ID to assign

**Returns:**
- Motor ID: Successfully changed and verified
- `-1`: Failed to change ID

**Example:**
```cpp
int new_id = motor.ddsm_change_id(5);
if (new_id == 5) {
    std::cout << "Motor ID successfully changed to 5" << std::endl;
}
```

---

## Feedback & Monitoring

### `int ddsm_id_check()`
**Description:** Checks and returns the ID of a connected motor.

**Parameters:** None

**Returns:**
- Motor ID (1-255): Valid motor detected
- `-1`: No motor detected or communication error

**Example:**
```cpp
int motor_id = motor.ddsm_id_check();
if (motor_id > 0) {
    std::cout << "Detected motor with ID: " << motor_id << std::endl;
}
```

---

### `void ddsm_get_info(uint8_t id)`
**Description:** Requests detailed information from a specific motor.

**Parameters:**
- `id`: Motor ID to query

**Returns:** None

**Behavior:**
- Sends info request packet
- Automatically processes response and updates data members
- For DDSM115: Updates temperature, ddsm_u8, fault_code
- For DDSM210: Updates mileage, position, fault_code

**Example:**
```cpp
motor.ddsm_get_info(1);
std::cout << "Motor temperature: " << motor.temperature << "°C" << std::endl;
std::cout << "Fault code: " << motor.fault_code << std::endl;
```

---

### `int ddsm210_fb()`
**Description:** Processes feedback data from DDSM210 motors.

**Parameters:** None

**Returns:**
- `1`: Successfully processed feedback
- `-1`: Communication error or invalid data

**Data Updated:**
- Speed, current, acceleration_time, temperature, fault_code (mode 0x64)
- Mileage, position, fault_code (mode 0x74)

---

### `int ddsm115_fb()`
**Description:** Processes feedback data from DDSM115 motors.

**Parameters:** None

**Returns:**
- `1`: Successfully processed feedback
- `-1`: Communication error or invalid data

**Data Updated:**
- Mode, torque, speed, position, fault_code
- Temperature, ddsm_u8 (when info flag is set)

---

## Data Members

### Speed & Motion
- `int speed_data`: Current motor speed in RPM (both DDSM115/210)
- `int ddsm_pos`: Current position (both DDSM115/210)
- `int32_t mileage`: Total distance/rotation counter (DDSM210 info only)

### Current & Power  
- `int current`: Current consumption in mA (DDSM210 only)
- `int ddsm_torque`: Torque output (DDSM115 only)

### Status & Health
- `int temperature`: Motor temperature in °C
- `int fault_code`: Error/fault code (0 = no fault)
- `int ddsm_mode`: Current control mode (DDSM115 only)

### Timing & Control
- `int acceleration_time`: Acceleration time setting (DDSM210 only)
- `int ddsm_u8`: Additional status data (DDSM115 info only)

---

## Constants

```cpp
#define DDSM_BAUDRATE 115200    // Default baud rate
#define TYPE_DDSM115  1         // DDSM115 motor type identifier
#define TYPE_DDSM210  2         // DDSM210 motor type identifier  
#define TIME_BETWEEN_CMD 4      // Milliseconds between commands
#define TIMEOUT_MS 4            // Communication timeout in milliseconds
```

---

## Usage Examples

### Basic Motor Control Setup

```cpp
#include "ddsm_ctrl.hpp"

int main() {
    DDSM_CTRL motor;
    
    // Initialize connection
    if (!motor.init("/dev/ttyUSB0", 115200)) {
        std::cerr << "Failed to initialize motor" << std::endl;
        return -1;
    }
    
    // Set motor type
    motor.set_ddsm_type(TYPE_DDSM210);
    
    // Check motor ID
    int motor_id = motor.ddsm_id_check();
    if (motor_id < 0) {
        std::cerr << "No motor detected" << std::endl;
        return -1;
    }
    
    std::cout << "Motor ID: " << motor_id << std::endl;
    
    // Set to speed mode
    motor.ddsm_change_mode(motor_id, 2);
    
    return 0;
}
```

### Speed Control Example

```cpp
void speed_control_demo(DDSM_CTRL& motor, uint8_t id) {
    // Set speed mode
    motor.ddsm_change_mode(id, 2);
    
    // Accelerate to 100 RPM
    for (int speed = 0; speed <= 100; speed += 10) {
        motor.ddsm_ctrl(id, speed, 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // Read feedback
        if (motor.ddsm210_fb() == 1) {
            std::cout << "Target: " << speed 
                      << " RPM, Actual: " << motor.speed_data << " RPM" << std::endl;
        }
    }
    
    // Stop motor
    motor.ddsm_stop(id);
}
```

### Position Control Example

```cpp
void position_control_demo(DDSM_CTRL& motor, uint8_t id) {
    // Set position mode
    motor.ddsm_change_mode(id, 3);
    
    // Move to different positions
    int positions[] = {0, 8192, 16384, 24576, 32767}; // 0°, 90°, 180°, 270°, 360°
    
    for (int pos : positions) {
        motor.ddsm_ctrl(id, pos, 1);
        std::cout << "Moving to position: " << pos << std::endl;
        
        // Wait for movement to complete
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        // Get current position
        motor.ddsm_get_info(id);
        std::cout << "Current position: " << motor.ddsm_pos << std::endl;
    }
}
```

### Multi-Motor Control Example

```cpp
void multi_motor_control() {
    std::vector<DDSM_CTRL> motors(4);
    std::vector<uint8_t> motor_ids = {1, 2, 3, 4};
    
    // Initialize all motors
    for (size_t i = 0; i < motors.size(); i++) {
        if (!motors[i].init("/dev/ttyUSB0", 115200)) {
            std::cerr << "Failed to initialize motor " << i << std::endl;
            continue;
        }
        motors[i].set_ddsm_type(TYPE_DDSM210);
        motors[i].ddsm_change_mode(motor_ids[i], 2); // Speed mode
    }
    
    // Synchronized speed control
    std::vector<int> speeds = {100, -100, 100, -100}; // Opposite directions
    
    for (size_t i = 0; i < motors.size(); i++) {
        motors[i].ddsm_ctrl(motor_ids[i], speeds[i], 1);
    }
    
    // Monitor all motors
    for (int count = 0; count < 10; count++) {
        for (size_t i = 0; i < motors.size(); i++) {
            if (motors[i].ddsm210_fb() == 1) {
                std::cout << "Motor " << motor_ids[i] 
                          << " Speed: " << motors[i].speed_data << " RPM" << std::endl;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // Stop all motors
    for (size_t i = 0; i < motors.size(); i++) {
        motors[i].ddsm_stop(motor_ids[i]);
    }
}
```

### Error Handling Example

```cpp
void error_handling_demo(DDSM_CTRL& motor, uint8_t id) {
    motor.ddsm_get_info(id);
    
    if (motor.fault_code != 0) {
        std::cout << "Motor fault detected: " << motor.fault_code << std::endl;
        
        // Common fault codes (motor-specific, check manual)
        switch (motor.fault_code) {
            case 1:
                std::cout << "Overcurrent fault" << std::endl;
                break;
            case 2:
                std::cout << "Overvoltage fault" << std::endl;
                break;
            case 3:
                std::cout << "Undervoltage fault" << std::endl;
                break;
            case 4:
                std::cout << "Overtemperature fault" << std::endl;
                break;
            default:
                std::cout << "Unknown fault code" << std::endl;
        }
        
        // Stop motor on fault
        motor.ddsm_stop(id);
    }
}
```

---

## Best Practices

1. **Always initialize before use:**
   ```cpp
   if (!motor.init(port_name, baud_rate)) {
       // Handle initialization error
   }
   ```

2. **Set motor type explicitly:**
   ```cpp
   motor.set_ddsm_type(TYPE_DDSM210);  // or TYPE_DDSM115
   ```

3. **Check motor ID before control:**
   ```cpp
   int id = motor.ddsm_id_check();
   if (id < 0) {
       // Handle no motor detected
   }
   ```

4. **Monitor fault codes regularly:**
   ```cpp
   motor.ddsm_get_info(id);
   if (motor.fault_code != 0) {
       motor.ddsm_stop(id);
       // Handle fault condition
   }
   ```

5. **Use appropriate delays between commands:**
   ```cpp
   motor.ddsm_ctrl(id, cmd1, 1);
   std::this_thread::sleep_for(std::chrono::milliseconds(TIME_BETWEEN_CMD));
   motor.ddsm_ctrl(id, cmd2, 1);
   ```

6. **Always stop motors on program exit:**
   ```cpp
   // In destructor or cleanup function
   for (auto& motor_id : motor_ids) {
       motor.ddsm_stop(motor_id);
   }
   ```

---

## Dependencies

- **Boost.Asio**: For serial communication
- **C++17**: Required language standard
- **Linux/Unix**: Serial port access (/dev/ttyUSB*, /dev/ttyACM*)

## Thread Safety

The `DDSM_CTRL` class is **not thread-safe**. If using multiple threads:
- Use separate instances for each thread, or
- Implement external synchronization (mutex/lock)

---

*Generated from DDSM_CTRL implementation - Version 1.0*
