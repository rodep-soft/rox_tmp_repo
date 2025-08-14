#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # パラメータファイルのパス
    config_file_path = '/home/rodep/git_workspace/rox_tmp_repo/ros_ws/config/ekf.yaml'
    
    # Madgwick launch file のパス
    madgwick_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('launch'),
                'launch_with_madgwick.py'
            ])
        ])
    )
    
    # EKF Localization Node
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[config_file_path],
        remappings=[
            ('/odometry/filtered', '/odom'),
            ('/accel/filtered', '/accel/filtered')
        ]
    )
    
    return LaunchDescription([
        madgwick_launch,
        ekf_node
    ])
