#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # パラメータファイルのパス
    config_file_path = '/home/rodep/git_workspace/rox_tmp_repo/ros_ws/config/config.yaml'
    
    # Joy Driver Node
    joy_driver_node = Node(
        package='joy_driver',
        executable='joy_driver_test',
        name='joy_driver_node',
        output='screen',
        parameters=[config_file_path]
    )
    
    # Mecanum Wheel Controller Node (with wheel odometry)
    mecanum_controller_node = Node(
        package='mecanum_wheel_controller',
        executable='mecanum_wheel_controller_node',
        name='mecanum_wheel_controller_node',
        output='screen',
        parameters=[config_file_path]
    )
    
    # IMU Node
    imu_node = Node(
        package='imu',
        executable='imu_node',
        name='imu_node',
        output='screen',
        parameters=[config_file_path]
    )
    
    # Madgwick Filter Node
    madgwick_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick_node',
        output='screen',
        parameters=[{
            'use_mag': False,
            'world_frame': 'enu',
            'publish_tf': False,
            'publish_debug_topics': True
        }],
        remappings=[
            ('/imu/data_raw', '/imu/data_raw'),
            ('/imu/data', '/imu/data')
        ]
    )
    
    return LaunchDescription([
        joy_driver_node,
        mecanum_controller_node,
        imu_node,
        madgwick_node
    ])
