import os

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

# This file launches the gazebo simulation world

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('world', default_value='depot',
                          description='Ignition World')
]

def generate_launch_description():

    # Directories
    pkg_ros_gz_sim = get_package_share_directory(
        'ros_gz_sim')
    
    # Paths
    gz_sim_launch = PathJoinSubstitution(
        [pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py']
    )         


    gz_sim = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_sim_launch),
            launch_arguments={
                'gz_args':[
                    '-v4 '
                    ],
                'on_exit_shutdown': 'True'
            }.items()           
    )
    
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gz_sim)
    return ld
