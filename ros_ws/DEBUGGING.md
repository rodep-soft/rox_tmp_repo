# Smart Debugging Guide

This document explains how to debug the ROS2 mecanum wheel system without requiring rebuilds.

## Problem with Previous Approach

Previously, debugging required commenting/uncommenting `RCLCPP_INFO` statements and rebuilding with `colcon build`, which was time-consuming and inefficient.

## New Smart Debugging Approach

### 1. Using ROS2 Log Levels

The code now uses `RCLCPP_DEBUG` for debug messages that can be enabled/disabled at runtime:

```bash
# Enable DEBUG level logging
ros2 run joy_driver joy_driver_node --ros-args --log-level DEBUG

# Or change log level at runtime
ros2 service call /joy_driver_node/set_logger_level rcl_interfaces/srv/SetLoggerLevel "{name: 'joy_driver_node', level: 'DEBUG'}"
```

### 2. Runtime Debug Parameters

Each node now has a `debug_output` parameter that can be toggled without rebuilding:

```bash
# Enable debug output
ros2 param set /joy_driver_node debug_output true
ros2 param set /mecanum_wheel_controller_node debug_output true

# Disable debug output
ros2 param set /joy_driver_node debug_output false
ros2 param set /mecanum_wheel_controller_node debug_output false

# View current parameter values
ros2 param get /joy_driver_node debug_output
```

### 3. Using ros2 topic for Monitoring

Instead of adding debug prints, monitor topics directly:

```bash
# Monitor joystick input
ros2 topic echo /joy

# Monitor velocity commands
ros2 topic echo /cmd_vel

# Monitor custom dpad commands
ros2 topic echo /cmd_dpad

# Monitor with specific rates
ros2 topic hz /cmd_vel
ros2 topic bw /cmd_vel
```

### 4. Launch with Debug Configuration

Use the new launch file with debug options:

```bash
# Launch with debug enabled
ros2 launch mecanum_system.launch.py debug:=true log_level:=debug

# Launch with normal operation
ros2 launch mecanum_system.launch.py debug:=false log_level:=info

# Launch with custom config
ros2 launch mecanum_system.launch.py config_file:=custom_config.yaml
```

### 5. Configuration File Updates

The `mecanum.yaml` configuration file now includes debug parameters:

```yaml
/**:
  ros__parameters:
    # Debug control - no rebuild needed to change!
    debug_output: false  # Set to true to enable debug output
    
    # Other parameters...
    linear_x_scale: 1.0
    # ...
```

## Quick Debug Commands Reference

```bash
# Start system with debug
ros2 launch mecanum_system.launch.py debug:=true

# Monitor all topics
ros2 topic list
ros2 topic echo /cmd_vel

# Change debug at runtime
ros2 param set /joy_driver_node debug_output true

# View all parameters
ros2 param list
ros2 param dump /joy_driver_node

# Change log levels
ros2 service call /joy_driver_node/set_logger_level rcl_interfaces/srv/SetLoggerLevel "{name: 'joy_driver_node', level: 'DEBUG'}"

# View node info
ros2 node info /joy_driver_node
ros2 node info /mecanum_wheel_controller_node
```

## Benefits

1. **No Rebuild Required**: Debug output can be toggled at runtime
2. **Real-time Monitoring**: Use `ros2 topic echo` to see data flow
3. **Flexible Configuration**: Multiple debug levels and parameters
4. **Professional Debugging**: Follows ROS2 best practices
5. **Time Saving**: No more waiting for `colcon build`

## Migration from Old Approach

**Old way (requires rebuild):**
```cpp
// Commenting/uncommenting for debug
// RCLCPP_INFO(this->get_logger(), "Debug message");
```

**New way (runtime control):**
```cpp
// Always available via log level
RCLCPP_DEBUG(this->get_logger(), "Debug message");

// Optional INFO level via parameter
if (debug_output_) {
  RCLCPP_INFO(this->get_logger(), "Debug message");
}
```

This approach eliminates the need for frequent rebuilds and provides much more flexible debugging capabilities.