from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    Launch file for the mecanum wheel system with configurable debug output.
    
    This launch file demonstrates smart debugging practices:
    1. Use log levels (DEBUG, INFO, WARN, ERROR) instead of commenting/uncommenting code
    2. Configure debug output via parameters without rebuilding
    3. Use ros2 topic echo for runtime monitoring
    """
    
    # Declare launch arguments for debug control
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug output for detailed logging'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Set ROS log level (debug, info, warn, error)'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='mecanum.yaml',
        description='Configuration file name in config directory'
    )
    
    # Get configuration file path
    config_path = os.path.join(
        get_package_share_directory('mecanum_wheel_controller'),  # This may need adjustment
        'config',
        LaunchConfiguration('config_file')
    )
    
    # Alternative: Use relative path from workspace
    workspace_config_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'config',
        'mecanum.yaml'
    )
    
    # Joy driver node
    joy_driver_node = Node(
        package='joy_driver',
        executable='joy_driver_node',
        name='joy_driver_node',
        parameters=[
            workspace_config_path,
            {'debug_output': LaunchConfiguration('debug')}
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        output='screen'
    )
    
    # Mecanum wheel controller node
    mecanum_controller_node = Node(
        package='mecanum_wheel_controller',
        executable='mecanum_wheel_controller_node',
        name='mecanum_wheel_controller_node',
        parameters=[
            workspace_config_path,
            {'debug_output': LaunchConfiguration('debug')}
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        output='screen'
    )
    
    # Joy node (from ros-humble-joy package)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device_id': 0,
            'deadzone': 0.05,
            'autorepeat_rate': 20.0
        }],
        output='screen'
    )
    
    # Add informational messages
    debug_info = LogInfo(
        msg=[
            TextSubstitution(text='=== Smart Debugging Tips ===\n'),
            TextSubstitution(text='1. Monitor topics in real-time:\n'),
            TextSubstitution(text='   ros2 topic echo /cmd_vel\n'),
            TextSubstitution(text='   ros2 topic echo /joy\n'),
            TextSubstitution(text='2. Change log levels at runtime:\n'),
            TextSubstitution(text='   ros2 service call /joy_driver_node/set_logger_level rcl_interfaces/srv/SetLoggerLevel "{{name: \'joy_driver_node\', level: \'DEBUG\'}}"\n'),
            TextSubstitution(text='3. Change debug parameters at runtime:\n'),
            TextSubstitution(text='   ros2 param set /joy_driver_node debug_output true\n'),
            TextSubstitution(text='4. View all parameters:\n'),
            TextSubstitution(text='   ros2 param list\n'),
            TextSubstitution(text='5. No rebuild needed for debug changes!\n'),
            TextSubstitution(text='==============================')
        ]
    )
    
    return LaunchDescription([
        debug_arg,
        log_level_arg,
        config_file_arg,
        debug_info,
        joy_node,
        joy_driver_node,
        mecanum_controller_node,
    ])