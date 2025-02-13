from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import LogInfo, DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.actions import Node

# This file launches the gazebo simulation world

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('world', default_value='maze',
                          description='Gazebo World')
]

def generate_launch_description():

    # Directories
    pkg_ros_gz_sim = get_package_share_directory(
        'ros_gz_sim')
    pkg_rushbott_gz_bringup = get_package_share_directory(
        'rushbott_gz_bringup')
    
    # Paths
    gz_sim_launch = PathJoinSubstitution(
        [pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py']
    )         

    # Set Gazebo resource path
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=PathJoinSubstitution([pkg_rushbott_gz_bringup, 'worlds'])
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments=[
            ('gz_args', [
                LaunchConfiguration('world'),
                '.sdf '
                '-v1 '
                '-r '
                ]),
            ('on_exit_shutdown', 'True')
        ]
    )

    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
             "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"
        ]
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gz_bridge_node)
    ld.add_action(gz_resource_path)
    ld.add_action(gz_sim)
    return ld
