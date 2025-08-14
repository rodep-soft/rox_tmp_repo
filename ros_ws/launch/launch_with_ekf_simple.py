#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():
    # EKF設定ファイルのパス
    ekf_config_file_path = '/home/rodep/git_workspace/rox_tmp_repo/ros_ws/config/ekf.yaml'
    
    # Base system launch (raw IMU + wheel odom)
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            '/home/rodep/git_workspace/rox_tmp_repo/ros_ws/launch/launch_raw_imu.py'
        ])
    )
    
    # EKF Localization Node (robot_localization)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file_path],
        remappings=[
            ('/odometry/filtered', '/odom/filtered'),
            ('/accel/filtered', '/accel/filtered')
        ]
    )
    
    return LaunchDescription([
        base_launch,
        ekf_node
    ])
