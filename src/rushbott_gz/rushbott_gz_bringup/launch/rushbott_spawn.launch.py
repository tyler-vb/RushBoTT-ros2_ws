from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('robot_name', default_value='RushBoTT',
                          description='Robot name'),
]

robot_name = LaunchConfiguration('robot_name')

def generate_launch_description():

    # Directories
    pkg_rushbott_common_bringup = get_package_share_directory(
        'rushbott_common_bringup')

    # Paths
    rushbott_nodes_launch = PathJoinSubstitution(
        [pkg_rushbott_common_bringup, 'launch', 'rushbott_nodes.launch.py'])
    
    spawn_robot_group_action = GroupAction([
        # Robot Nodes
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rushbott_nodes_launch)),

        # Spawn RushBoTT
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', robot_name,
                        '-topic', 'robot_description',
                        ],
            output='screen'
        )
    ])

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(spawn_robot_group_action)
    return ld