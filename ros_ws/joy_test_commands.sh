#!/bin/bash

# Joy driver test commands

# Basic movements
alias joy_stop='ros2 topic pub --once /joy sensor_msgs/msg/Joy "{axes: [0.0, 0.0, 0.0, 0.0, 1.0, 1.0], buttons: [0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0]}"'
alias joy_forward='ros2 topic pub --once /joy sensor_msgs/msg/Joy "{axes: [0.0, 0.5, 0.0, 0.0, 1.0, 1.0], buttons: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]}"'
alias joy_backward='ros2 topic pub --once /joy sensor_msgs/msg/Joy "{axes: [0.0, -0.5, 0.0, 0.0, 1.0, 1.0], buttons: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]}"'
alias joy_left='ros2 topic pub --once /joy sensor_msgs/msg/Joy "{axes: [-0.5, 0.0, 0.0, 0.0, 1.0, 1.0], buttons: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]}"'
alias joy_right='ros2 topic pub --once /joy sensor_msgs/msg/Joy "{axes: [0.5, 0.0, 0.0, 0.0, 1.0, 1.0], buttons: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]}"'

# Rotation
alias joy_rotate_left='ros2 topic pub --once /joy sensor_msgs/msg/Joy "{axes: [0.0, 0.0, 0.5, 0.0, 1.0, 1.0], buttons: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]}"'
alias joy_rotate_right='ros2 topic pub --once /joy sensor_msgs/msg/Joy "{axes: [0.0, 0.0, -0.5, 0.0, 1.0, 1.0], buttons: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]}"'

# L2/R2 manual rotation
alias joy_l2_rotate='ros2 topic pub --once /joy sensor_msgs/msg/Joy "{axes: [0.0, 0.0, 0.0, 0.0, -1.0, 1.0], buttons: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]}"'
alias joy_r2_rotate='ros2 topic pub --once /joy sensor_msgs/msg/Joy "{axes: [0.0, 0.0, 0.0, 0.0, 1.0, -1.0], buttons: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]}"'

# Combined movements
alias joy_forward_right='ros2 topic pub --once /joy sensor_msgs/msg/Joy "{axes: [0.5, 0.5, 0.0, 0.0, 1.0, 1.0], buttons: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]}"'

# Mode switches
alias joy_mode_joy='ros2 topic pub --once /joy sensor_msgs/msg/Joy "{axes: [0.0, 0.0, 0.0, 0.0, 1.0, 1.0], buttons: [0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0]}"'
alias joy_mode_dpad='ros2 topic pub --once /joy sensor_msgs/msg/Joy "{axes: [0.0, 0.0, 0.0, 0.0, 1.0, 1.0], buttons: [0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0]}"'
alias joy_mode_stop='ros2 topic pub --once /joy sensor_msgs/msg/Joy "{axes: [0.0, 0.0, 0.0, 0.0, 1.0, 1.0], buttons: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]}"'

echo "Joy driver test commands loaded!"
echo "Commands: joy_stop, joy_forward, joy_backward, joy_left, joy_right"
echo "          joy_rotate_left, joy_rotate_right, joy_l2_rotate, joy_r2_rotate"
echo "          joy_mode_joy, joy_mode_dpad, joy_mode_stop"
