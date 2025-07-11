# ROS 2 Mecanum Wheel Control System

A complete ROS 2 system for controlling mecanum wheel robots using joystick input. This system provides a full control pipeline from joystick input to individual wheel velocity commands with serial communication to motor controllers.

## Docker Environment

This project includes a complete Docker setup for easy deployment and development:

- **Base Image**: `osrf/ros:humble-desktop`
- **Included Packages**: ROS 2 Humble, joy package, development tools
- **Serial Support**: Full device access for motor communication
- **Network**: Host networking for easy ROS 2 communication

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

- Docker and Docker Compose
- A USB joystick/gamepad (for joystick control)
- ROS 2 (tested on ROS 2 Humble) - included in Docker container
- Serial connection to motor controller (for actual robot control)

## Package Structure

```
ros_ws/
├── config/
│   └── mechanum.yaml           # Robot configuration parameters
├── src/
│   ├── joy_driver/              # Joystick to cmd_vel converter
│   │   ├── src/
│   │   │   └── joy_driver_node.cpp
│   │   ├── package.xml
│   │   └── CMakeLists.txt
│   ├── mecanum_wheel_controller/ # Mecanum wheel kinematics
│   │   ├── src/
│   │   │   └── mecanum_wheel_controller_node.cpp
│   │   ├── config/
│   │   │   └── mechanum.yaml    # Local config copy
│   │   ├── package.xml
│   │   └── CMakeLists.txt
│   ├── color_sensor/            # Color sensor package
│   │   ├── src/
│   │   │   └── color_sensor.py
│   │   ├── package.xml
│   │   └── setup.py
│   ├── imu/                     # IMU sensor package
│   │   ├── src/
│   │   │   ├── imu.cpp
│   │   │   └── imulib.cpp
│   │   ├── package.xml
│   │   └── CMakeLists.txt
│   └── ultrasonic_sensor/       # Ultrasonic sensor package
│       ├── src/
│       │   └── ultrasonic_sensor.cpp
│       ├── package.xml
│       └── CMakeLists.txt
├── launch/
│   ├── launch.py               # Complete system with joystick
│   └── launch_without_joy.py   # System without joystick (direct cmd_vel)
└── README.md
```

## Building the System

### Using Docker (Recommended)

1. **Build and start the Docker container**:
   ```bash
   cd /home/tatsv/jordan/rox_repo
   docker-compose up -d
   ```

2. **Access the container**:
   ```bash
   docker exec -it ros2_rox_container bash
   ```

3. **Build the ROS 2 workspace**:
   ```bash
   cd /ros_ws
   colcon build
   source install/setup.bash
   ```

### Native Installation

```bash
cd ros_ws
colcon build
source install/setup.bash
```

## Configuration

### Robot Configuration File

The main robot parameters are configured in `config/mechanum.yaml`:

```yaml
/mechanum_wheel_controller:
  ros__parameters:
    wheel_radius: 0.1
    wheel_base_x: 0.5
    wheel_base_y: 0.5
    serial_port: "/dev/ttyACM1"
    baud_rate: 115200
    motor_ids: [1, 2, 3, 4]  # FL, FR, BL, BR
```

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
| `wheel_radius` | 0.1 | Wheel radius in meters |
| `wheel_base_x` | 0.5 | Distance between front and rear wheels (m) |
| `wheel_base_y` | 0.5 | Distance between left and right wheels (m) |
| `serial_port` | "/dev/ttyACM1" | Serial port for motor communication |
| `baud_rate` | 115200 | Serial communication baud rate |
| `motor_ids` | [1, 2, 3, 4] | Motor IDs: [FL, FR, BL, BR] |

## Usage

### Quick Start with Joystick Control

1. **Start the Docker container**:
   ```bash
   docker-compose up -d
   docker exec -it ros2_rox_container bash
   ```

2. **Connect your joystick** to the computer and ensure it's accessible in the container

3. **Launch the complete system with joystick**:
   ```bash
   cd /ros_ws
   source install/setup.bash
   ros2 launch launch/launch.py
   ```

4. **Use the joystick** to control the robot:
   - Left stick: Forward/backward and strafe left/right
   - Right stick: Rotation

### Quick Start without Joystick

For testing or manual control without a physical joystick:

1. **Launch the system without joystick**:
   ```bash
   cd /ros_ws
   source install/setup.bash
   ros2 launch launch/launch_without_joy.py
   ```

2. **Manually publish velocity commands**:
   ```bash
   # Example: Move forward
   ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"
   
   # Example: Strafe right
   ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: -0.5, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"
   
   # Example: Rotate clockwise
   ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.5}"
   ```

### Finding Your Joystick Axes

If your controller uses different axis mappings:

1. Run the system and observe the terminal output
2. Move each joystick axis individually
3. Note which axis numbers change in the logs
4. Update the `launch.py` file with the correct axis numbers

### Custom Configuration

Edit `config/mechanum.yaml` to customize robot parameters:

```yaml
# Example: Smaller robot configuration
/mechanum_wheel_controller:
  ros__parameters:
    wheel_radius: 0.05      # Smaller wheels
    wheel_base_x: 0.3       # Shorter wheelbase
    wheel_base_y: 0.3       # Narrower track width
    serial_port: "/dev/ttyUSB0"  # Different serial port
    baud_rate: 9600         # Lower baud rate
```

Edit `launch/launch.py` to customize joystick parameters:

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
# Run joy node only (inside container)
ros2 run joy joy_node

# Run joy driver only
ros2 run joy_driver joy_driver_node

# Run mecanum controller only
ros2 run mecanum_wheel_controller mecanum_wheel_controller_node
```

### Serial Motor Communication

The mecanum wheel controller communicates with motors via serial connection. Ensure:

1. **Serial device permissions** (inside container):
   ```bash
   # Check if serial device exists
   ls -la /dev/ttyACM* /dev/ttyUSB*
   
   # If permission denied, add user to dialout group (on host system)
   sudo usermod -a -G dialout $USER
   ```

2. **Motor controller protocol**: The system sends wheel velocities as comma-separated values:
   ```
   FL_velocity,FR_velocity,BL_velocity,BR_velocity\n
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

The system is already configured for serial motor communication. To modify motor control:

1. **Update motor protocol** in `mecanum_wheel_controller_node.cpp`
2. **Configure serial parameters** in `config/mechanum.yaml`
3. **Test motor communication**:
   ```bash
   # Monitor serial output (on host system)
   sudo cat /dev/ttyACM1
   
   # Send test commands
   echo "10.0,10.0,10.0,10.0" > /dev/ttyACM1
   ```

### Example Alternative Motor Interfaces

```cpp
// For CAN bus communication
#include <linux/can.h>
#include <linux/can/raw.h>

// For I2C communication  
#include <linux/i2c-dev.h>

// For SPI communication
#include <linux/spi/spidev.h>

// For ROS 2 motor controller topics
motor_pub_ = this->create_publisher<control_msgs::msg::JointJog>("joint_commands", 10);
```

## Troubleshooting

### Docker Issues

```bash
# Rebuild container if needed
docker-compose down
docker-compose build --no-cache
docker-compose up -d

# Check container logs
docker logs ros2_rox_container

# Access container shell
docker exec -it ros2_rox_container bash
```

### Joystick Not Detected

```bash
# Check if joystick is connected (on host)
ls /dev/input/js*

# Test joystick directly (on host)
sudo apt install jstest-gtk
jstest /dev/input/js0

# Ensure device is accessible in container
docker exec -it ros2_rox_container ls /dev/input/
```

### Serial Communication Issues

```bash
# Check serial devices (in container)
ls -la /dev/ttyACM* /dev/ttyUSB*

# Test serial communication (on host)
sudo minicom -D /dev/ttyACM1 -b 115200

# Check permissions (on host)
sudo chmod 666 /dev/ttyACM1
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
5. Verify serial connection: `ros2 topic echo /motor_commands`

## License

[Add your license here]

## Contributors

[Add contributors here]

## Support

For issues and questions:
1. Check the troubleshooting section above
2. Review ROS 2 logs: `ros2 run rqt_console rqt_console`
3. Open an issue in the repository
