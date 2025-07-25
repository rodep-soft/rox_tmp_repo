from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            namespace="joy",
        ),

        Node(
            package="joy_driver",
            executable="joy_driver_node",
            name="joy_driver_node",
            parameters=["config/config.yaml"]
        ),

        Node(
            package="mecanum_wheel_controller",
            executable="mecanum_wheel_controller_node",
            name="mecanum_wheel_controller_node",
            parameters=["config/config.yaml"]
        ),

        Node(
            package="lifting_motor",
            executable="lifting_motor",
            name="lifting_motor_node",
            parameters=[]
        )
    ])
