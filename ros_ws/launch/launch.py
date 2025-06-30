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
        # Launching the joy driver node with parameters
        Node(
            package='joy_driver',
            executable='joy_driver_node',
            name='joy_driver_node',
            parameters=[{
                # Velocity scaling parameters
                'linear_x_scale': 1.0,    # Max forward/backward speed (m/s)
                'linear_y_scale': 1.0,    # Max left/right strafe speed (m/s)  
                'angular_scale': 1.0,     # Max rotation speed (rad/s)
                
                # Joystick axis mapping (adjust these based on your controller)
                # This is just an example;
                'linear_x_axis': 1,       # Left stick vertical (forward/backward)
                'linear_y_axis': 0,       # Left stick horizontal (left/right strafe)
                'angular_axis': 3,        # Right stick horizontal (rotation)
            }]
        )
    ])