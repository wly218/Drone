from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from ros_gz_bridge.actions import RosGzBridge
# 妈的，为什么无法解析导入
def generate_launch_description():

    bridge_name = LaunchConfiguration('bridge_name')
    config_file = LaunchConfiguration('config_file')

    declare_bridge_name_cmd = DeclareLaunchArgument(
        'bridge_name', 
        default_value='ros_gz_bridge',
        description='Name of ros_gz_bridge node'
    )

    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file', 
        default_value='/home/wang/Drone/src/bridge/config/bridge.yaml',
        description='YAML config file'
    )

    # Create the launch description and populate
    ld = LaunchDescription([
        RosGzBridge(
            bridge_name=LaunchConfiguration('bridge_name'),
            config_file=LaunchConfiguration('config_file'),
        ),
    ])

    # Declare the launch options
    ld.add_action(declare_bridge_name_cmd)
    ld.add_action(declare_config_file_cmd)

    return ld