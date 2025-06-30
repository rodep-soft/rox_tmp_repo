from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Launching the joy node
        Node(
            package='joy',
            namespace='joy',
            executable='joy_node',
            name='joy_node'
        ),
        # Launching the joy driver node
        Node(
            package='joy_driver',
            executable='joy_driver_node',
            name='joy_driver_node'
        )
    ])