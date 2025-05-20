import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():


    drone_dir = get_package_share_directory('drone_cartographer')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    rviz_config_dir = os.path.join(drone_dir, 'rviz')+"/cartographer.rviz"

    sdf_file  =  os.path.join(drone_dir,'description','drone_model.sdf')
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

     # 哎，还是不够了解啊，知其然而不知其所以然
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[sdf_file],
        output=['screen']
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(drone_dir, 'configuration', 'bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    return LaunchDescription([
        bridge,
        joint_state_publisher,
        robot_state_publisher,
        rviz_node
    ])