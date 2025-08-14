#!/usr/bin/env python3

import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # IMU Filter Madgwick Node
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            namespace='',
            parameters=[{
                'use_mag': False,
                'publish_tf': False,
                'world_frame': 'enu',
                'fixed_frame': 'base_link',
                'gain': 0.1,
                'zeta': 0.0,
                'mag_bias_x': 0.0,
                'mag_bias_y': 0.0,
                'mag_bias_z': 0.0,
                'orientation_stddev': 0.0,
                'angular_velocity_stddev': 0.0,
                'linear_acceleration_stddev': 0.0,
                'magnetic_field_stddev': 0.0
            }],
            remappings=[
                ('imu/data_raw', '/imu'),
                ('imu/data', '/imu/filtered')
            ]
        ),
        
        # Joy Driver Node (using filtered IMU)
        Node(
            package='joy_driver',
            executable='joy_driver_node',
            name='joy_driver_node',
            parameters=['/home/rodep/git_workspace/rox_tmp_repo/ros_ws/config/config.yaml'],
            remappings=[
                ('imu', '/imu/filtered')
            ]
        ),
        
        # Other existing nodes...
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node'
        ),
        
        Node(
            package='mecanum_wheel_controller',
            executable='mecanum_wheel_controller_node',
            name='mecanum_wheel_controller_node',
            parameters=['/home/rodep/git_workspace/rox_tmp_repo/ros_ws/config/config.yaml']
        )
    ])
