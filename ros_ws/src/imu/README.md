# IMU Package

This package reads data from a BNO055 IMU sensor and publishes it as ROS 2 topics.

## Overview

This package provides a ROS 2 node that interfaces with a BNO055 9-DOF IMU sensor over I2C. It publishes orientation (quaternion), angular velocity, linear acceleration, and magnetometer data.

## Features

- Publishes `sensor_msgs/Imu` messages containing orientation, angular velocity, and linear acceleration.
- Publishes `sensor_msgs/MagneticField` messages.
- Based on a C++ library for the BNO055 sensor.
- Configurable I2C address and device path.

## Hardware Requirements

- **Sensor**: BNO055 9-DOF IMU
- **Connection**: I2C

## Building

```bash
cd /path/to/ros_ws
colcon build --packages-select imu
source install/setup.bash
```

## Usage

To run the IMU node:

```bash
ros2 run imu imu_node
```

To view the published data:

```bash
# IMU data (orientation, angular velocity, linear acceleration)
ros2 topic echo /imu/data

# Magnetometer data
ros2 topic echo /imu/mag
```

## Published Topics

- **`/imu/data`** (`sensor_msgs/msg/Imu`) - Publishes the IMU data.
- **`/imu/mag`** (`sensor_msgs/msg/MagneticField`) - Publishes the magnetometer data.
