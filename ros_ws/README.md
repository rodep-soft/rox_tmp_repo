# ROS 2 Mecanum Wheel Control System

A complete ROS 2 system for controlling mecanum wheel robots using joystick input. This system provides a full control pipeline from joystick input to individual wheel velocity commands.

## System Overview

The system consists of three main components:

1. **Joy Node** - Publishes joystick input data
2. **Joy Driver** - Converts joystick input to robot velocity commands (`cmd_vel`)
3. **Mecanum Wheel Controller** - Converts velocity commands to individual wheel velocities

```
[Joystick] → [Joy Node] → [Joy Driver] → [Mecanum Controller] → [Motors]
            /joy topic    cmd_vel topic   wheel velocities
```

## Features

- ✅ Full omnidirectional control (forward/backward, left/right strafe, rotation)
- ✅ Configurable velocity scaling and joystick axis mapping
- ✅ Mecanum wheel kinematics implementation
- ✅ Parameter-based configuration
- ✅ Launch file for easy startup
- ✅ Detailed logging and debugging output

## Prerequisites

- ROS 2 (tested on ROS 2 Humble)
- A USB joystick/gamepad
- `joy` package: `sudo apt install ros-humble-joy`

## Package Structure

```
ros_ws/
├── src/
│   ├── joy_driver/              # Joystick to cmd_vel converter
│   │   ├── src/
│   │   │   └── joy_driver_node.cpp
│   │   ├── package.xml
│   │   └── CMakeLists.txt
│   └── mecanum_wheel_controller/ # Mecanum wheel kinematics
│       ├── src/
│       │   └── mecanum_wheel_controller_node.cpp
│       ├── package.xml
│       └── CMakeLists.txt
├── launch/
│   └── launch.py               # Complete system launch file
└── README.md
```

## Building the System

```bash
cd ros_ws
colcon build
source install/setup.bash
```

## Configuration

### Joystick Axis Mapping

The default axis mapping assumes a standard Xbox/PlayStation controller:

- **Axis 0**: Left stick horizontal (strafe left/right)
- **Axis 1**: Left stick vertical (forward/backward)
- **Axis 3**: Right stick horizontal (rotation)

### Parameters

#### Joy Driver Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `linear_x_scale` | 1.0 | Max forward/backward speed (m/s) |
| `linear_y_scale` | 1.0 | Max left/right strafe speed (m/s) |
| `angular_scale` | 1.0 | Max rotation speed (rad/s) |
| `linear_x_axis` | 1 | Joystick axis for forward/backward |
| `linear_y_axis` | 0 | Joystick axis for left/right strafe |
| `angular_axis` | 3 | Joystick axis for rotation |

#### Mecanum Controller Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `wheel_radius` | 0.05 | Wheel radius in meters |
| `wheel_base_x` | 0.3 | Distance between front and rear wheels (m) |
| `wheel_base_y` | 0.3 | Distance between left and right wheels (m) |

## Usage

### Quick Start

1. **Connect your joystick** to the computer
2. **Launch the complete system**:
   ```bash
   cd ros_ws
   source install/setup.bash
   ros2 launch launch/launch.py
   ```
3. **Use the joystick** to control the robot:
   - Left stick: Forward/backward and strafe left/right
   - Right stick: Rotation

### Finding Your Joystick Axes

If your controller uses different axis mappings:

1. Run the system and observe the terminal output
2. Move each joystick axis individually
3. Note which axis numbers change in the logs
4. Update the `launch.py` file with the correct axis numbers

### Custom Configuration

Edit `launch/launch.py` to customize parameters:

```python
# Example: Slower, more precise control
'linear_x_scale': 0.5,    # Slower forward/backward
'linear_y_scale': 0.5,    # Slower strafe
'angular_scale': 0.8,     # Slower rotation

# Example: Different controller mapping
'linear_x_axis': 1,       # Left stick vertical
'linear_y_axis': 0,       # Left stick horizontal  
'angular_axis': 2,        # Right stick horizontal (PS controller)
```

### Running Individual Components

You can also run components separately for testing:

```bash
# Run joy node only
ros2 run joy joy_node

# Run joy driver only
ros2 run joy_driver joy_driver_node

# Run mecanum controller only
ros2 run mecanum_wheel_controller mecanum_wheel_controller_node
```

## Mecanum Wheel Kinematics

The system implements standard mecanum wheel kinematics for a 4-wheel configuration:

```
    FL ↗  ↖ FR
       |⬆|
    BL ↖  ↗ BR
```

### Kinematic Equations

```cpp
FL = (vx - vy - (lx + ly) * wz) / r
FR = (vx + vy + (lx + ly) * wz) / r
BL = (vx + vy - (lx + ly) * wz) / r  
BR = (vx - vy + (lx + ly) * wz) / r
```

Where:
- `vx`, `vy`, `wz`: Linear X, Linear Y, Angular Z velocities
- `lx`, `ly`: Half distances between wheels
- `r`: Wheel radius
- `FL`, `FR`, `BL`, `BR`: Individual wheel velocities (rad/s)

## Topics

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/joy` | `sensor_msgs/Joy` | Raw joystick input |
| `/cmd_vel` | `geometry_msgs/Twist` | Robot velocity commands |

## Debugging

### Check Topics

```bash
# List active topics
ros2 topic list

# Monitor joystick input
ros2 topic echo /joy

# Monitor velocity commands
ros2 topic echo /cmd_vel
```

### Check Nodes

```bash
# List running nodes
ros2 node list

# Check node info
ros2 node info /joy_driver_node
ros2 node info /mecanum_wheel_controller_node
```

## Extending the System

### Adding Motor Control

To connect to actual motors, modify the `mecanum_wheel_controller_node.cpp`:

1. Add your motor driver dependencies to `package.xml` and `CMakeLists.txt`
2. Implement motor communication in the `publish_motor_commands()` function
3. Examples of motor interfaces:
   - PWM signals
   - Serial communication
   - CAN bus
   - ROS 2 motor controller topics

### Example Motor Integration

```cpp
// Add to constructor
motor_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("motor_commands", 10);

// Implement motor publishing
void publish_motor_commands(double fl, double fr, double bl, double br) {
    auto motor_msg = std_msgs::msg::Float64MultiArray();
    motor_msg.data = {fl, fr, bl, br};
    motor_pub_->publish(motor_msg);
}
```

## Troubleshooting

### Joystick Not Detected

```bash
# Check if joystick is connected
ls /dev/input/js*

# Test joystick directly
sudo apt install jstest-gtk
jstest /dev/input/js0
```

### Wrong Axis Mapping

1. Run the system and watch the terminal output
2. Move joystick axes one at a time
3. Update the axis parameters in `launch.py`

### No Movement

1. Check if all nodes are running: `ros2 node list`
2. Check topic communication: `ros2 topic echo /cmd_vel`
3. Verify joystick input: `ros2 topic echo /joy`
4. Check parameter values are reasonable (not too small)

## License

[Add your license here]

## Contributors

[Add contributors here]

## Support

For issues and questions:
1. Check the troubleshooting section above
2. Review ROS 2 logs: `ros2 run rqt_console rqt_console`
3. Open an issue in the repository
