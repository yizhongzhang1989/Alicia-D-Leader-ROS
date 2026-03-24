from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 声明参数
    debug_mode_arg = DeclareLaunchArgument(
        'debug_mode', default_value='false', description='Enable debug mode')
    port_arg = DeclareLaunchArgument(
        'port', default_value='', description='Serial port device name (e.g. ttyACM0), empty for auto-detect')
    baudrate_arg = DeclareLaunchArgument(
        'baudrate', default_value='1000000', description='Serial baudrate')
    query_rate_arg = DeclareLaunchArgument(
        'query_rate', default_value='200.0', description='Joint state query rate (Hz)')
    joint_config_arg = DeclareLaunchArgument(
        'joint_config', default_value='',
        description='Path to joint_config.yaml. Empty uses default from package.')

    # Single driver node
    driver_node = Node(
        package='alicia_duo_leader_driver',
        executable='alicia_driver_node.py',
        name='alicia_driver_node',
        output='screen',
        parameters=[{
            'debug_mode': LaunchConfiguration('debug_mode'),
            'baudrate': LaunchConfiguration('baudrate'),
            'port': LaunchConfiguration('port'),
            'query_rate': LaunchConfiguration('query_rate'),
            'joint_config': LaunchConfiguration('joint_config'),
        }],
    )

    return LaunchDescription([
        debug_mode_arg,
        port_arg,
        baudrate_arg,
        query_rate_arg,
        joint_config_arg,
        driver_node,
    ])
