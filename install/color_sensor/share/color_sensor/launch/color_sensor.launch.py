from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    node = Node(
        package='color_sensor',
        executable='color_publisher',
        name='color_publisher_node'
    )
    
    ld.add_action(node)
    return ld
