#!/bin/bash

# Smart Debugging Demo Script for Mecanum Wheel System
# This script demonstrates how to debug without rebuilding

echo "=== Smart Debugging Demo ==="
echo "This script shows commands for debugging without colcon build"
echo ""

echo "1. Launch system with debug enabled:"
echo "   ros2 launch mecanum_system.launch.py debug:=true log_level:=debug"
echo ""

echo "2. Monitor topics in real-time (in separate terminals):"
echo "   ros2 topic echo /joy"
echo "   ros2 topic echo /cmd_vel"
echo "   ros2 topic echo /cmd_dpad"
echo ""

echo "3. Enable/disable debug output at runtime:"
echo "   ros2 param set /joy_driver_node debug_output true"
echo "   ros2 param set /mecanum_wheel_controller_node debug_output true"
echo ""

echo "4. Change log levels at runtime:"
echo "   ros2 service call /joy_driver_node/set_logger_level rcl_interfaces/srv/SetLoggerLevel \"{name: 'joy_driver_node', level: 'DEBUG'}\""
echo ""

echo "5. View system information:"
echo "   ros2 node list"
echo "   ros2 topic list"
echo "   ros2 param list"
echo ""

echo "6. Performance monitoring:"
echo "   ros2 topic hz /cmd_vel"
echo "   ros2 topic bw /cmd_vel"
echo ""

echo "=== NO MORE COLCON BUILD FOR DEBUG! ==="
echo ""

# If running this script directly, you can uncomment the following to execute commands:
# echo "Would you like to run some of these commands? (y/n)"
# read -r response
# if [[ "$response" == "y" || "$response" == "Y" ]]; then
#     echo "Listing ROS2 nodes..."
#     ros2 node list
#     echo ""
#     echo "Listing ROS2 topics..."
#     ros2 topic list
# fi