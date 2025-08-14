from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            namespace="",
            output='screen'
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
        ),

        # robot_localization EKF node for sensor fusion
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=['config/ekf.yaml'],
            remappings=[
                ('odometry/filtered', '/odom/filtered'),
                ('/diagnostics', '/diagnostics')
            ]
        )
    ])
