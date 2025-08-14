from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Basic nodes that are always launched
    nodes = [
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            namespace="",
            output='screen'
        ),

        # IMU Filter Madgwick (always enabled)
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_madgwick',
            output='screen',
            parameters=[{
                'use_mag': False,
                'publish_tf': False,
                'world_frame': 'enu',
                'fixed_frame': 'base_link',
                'gain': 0.1,
                'zeta': 0.0
            }],
            remappings=[
                ('imu/data_raw', '/imu'),
                ('imu/data', '/imu/filtered')
            ]
        ),

        Node(
            package="joy_driver",
            executable="joy_driver_node",
            name="joy_driver_node",
            output='screen',
            parameters=["config/config.yaml"]
        ),

        Node(
            package="mecanum_wheel_controller",
            executable="mecanum_wheel_controller_node",
            name="mecanum_wheel_controller_node",
            output='screen',
            parameters=["config/config.yaml"]
        ),

        Node(
            package='color_sensor',
            executable='color_publisher',
            name='color_sensor_publisher_node',
            output='screen'
        ),

        Node(
            package='imu',
            executable='imu_node',
            name='imu_node',
            output='screen'
        ),

        Node(
            package="led_control",
            executable="led_control_node",
            name="led_control_node",
            output="screen"
        ),

        Node(
            package="lifting_motor",
            executable="lifting_motor",
            name="lifting_motor_node",
            parameters=[]
        )
    ]
    
    return LaunchDescription(nodes)
