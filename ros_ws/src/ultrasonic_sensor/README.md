# Ultrasonic Sensor Package

ROS 2 package for HC-SR04 ultrasonic sensor distance measurement using GPIO.

## Prerequisites

This package requires the libgpiod library for GPIO access on Linux systems (especially Raspberry Pi).

### Installing libgpiod

On Ubuntu/Debian systems:
```bash
sudo apt update
sudo apt install libgpiod-dev libgpiod2
```

On Raspberry Pi OS:
```bash
sudo apt update
sudo apt install libgpiod-dev libgpiod2
```

## Hardware Setup

- Connect HC-SR04 ultrasonic sensor to Raspberry Pi GPIO pins
- Default configuration:
  - TRIG pin: BCM GPIO 6
  - ECHO pin: BCM GPIO 5
  - VCC: 5V
  - GND: Ground

## Building

```bash
cd ~/ros_ws
colcon build --packages-select ultrasonic_sensor
```

## Running

```bash
source ~/ros_ws/install/setup.bash
ros2 run ultrasonic_sensor ultrasonic_sensor
```

## Configuration

The GPIO pins can be modified in `src/ultrasonic_sensor.cpp`:
- `TRIG_LINE`: Trigger pin (default: 6)
- `ECHO_LINE`: Echo pin (default: 5)
- `CHIP_NAME`: GPIO chip name (default: "gpiochip0")

Note: On Raspberry Pi 5, you may need to change to "gpiochip4" for BCM pins.

## License

MIT License
