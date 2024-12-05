from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

# This launch file generates the robot description topic, starts Rviz2, and spawns the robot in a gazebo world

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('use_rviz', default_value='true',
                          choices=['true', 'false'], description='Start rviz.'),
]

robot_name = 'RushBoTT'

def generate_launch_description():

    # Directories
    pkg_rushbott_common_bringup = get_package_share_directory(
        'rushbott_common_bringup')

    # Paths
    robot_description_launch = PathJoinSubstitution(
        [pkg_rushbott_common_bringup, 'launch', 'robot_description.launch.py'])
    rviz2_launch = PathJoinSubstitution(
        [pkg_rushbott_common_bringup, 'launch', 'rviz2.launch.py'])
    
    spawn_robot_group_action = GroupAction([
        # Rviz
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rviz2_launch),
            condition=IfCondition(LaunchConfiguration('use_rviz'))),

        # Robot Description
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_description_launch)),

        
        # Spawn RushBoTT
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', robot_name,
                        '-topic', 'robot_description'],
           output='screen'
        )

    ])

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(spawn_robot_group_action)
    return ld