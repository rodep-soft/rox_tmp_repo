from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("mecanum_wheel_controller"),
                    "config",
                    "mecanum.yaml",
                ]
            ),
            description="Path to the configuration file.",
        )
    ]

    # Get launch configurations
    config_file = LaunchConfiguration("config_file")

    # Nodes to launch
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        namespace="joy",
    )

    joy_driver_node = Node(
        package="joy_driver",
        executable="joy_driver_node",
        name="joy_driver_node",
        parameters=[config_file],
    )

    mecanum_wheel_controller_node = Node(
        package="mecanum_wheel_controller",
        executable="mecanum_wheel_controller_node",
        name="mecanum_wheel_controller_node",
        parameters=[config_file],
    )

    return LaunchDescription(
        declared_arguments + [joy_node, joy_driver_node, mecanum_wheel_controller_node]
    )
