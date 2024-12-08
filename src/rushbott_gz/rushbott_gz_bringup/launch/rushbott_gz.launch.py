from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

ARGUMENTS = [
    DeclareLaunchArgument('use_rviz', default_value='true',
                          choices=['true', 'false'],
                          description='Start rviz.'),
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('world', default_value='depot',
                          description='Ignition World'),
]

def generate_launch_description():

    # Directories
    pkg_rushbott_gz_bringup = get_package_share_directory(
        'rushbott_gz_bringup')
    
    # Paths
    rushbott_spawn_launch = PathJoinSubstitution(
        [pkg_rushbott_gz_bringup, 'launch', 'rushbott_spawn.launch.py'])
    sim_launch = PathJoinSubstitution(
        [pkg_rushbott_gz_bringup, 'launch', 'sim.launch.py'])
    
    # Start sim
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch),
        launch_arguments=[
            ('world', LaunchConfiguration('world'))
        ]
    )

    # Spawn robot
    rushbott_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rushbott_spawn_launch),
        launch_arguments=[
        ('use_rviz', LaunchConfiguration('use_rviz'))
        ]
    )

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(sim)
    ld.add_action(rushbott_spawn)
    return ld