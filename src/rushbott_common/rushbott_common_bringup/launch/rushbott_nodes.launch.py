from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='use_sim_time')                 
]

def generate_launch_description(): 
    # Directories
    pkg_rushbott_common_bringup = get_package_share_directory('rushbott_common_bringup')
    pkg_rushbott_control = get_package_share_directory('rushbott_control')
    pkg_rushbott_moveit_config = get_package_share_directory('rushbott_moveit_config')

    # Paths
    robot_description_launch = PathJoinSubstitution(
        [pkg_rushbott_common_bringup, 'launch', 'robot_description.launch.py'])
    control_launch = PathJoinSubstitution(
        [pkg_rushbott_control, 'launch', 'control.launch.py'])
    teleop_launch = PathJoinSubstitution(
        [pkg_rushbott_control, 'launch', 'teleop.launch.py'])
    moveit2_launch = PathJoinSubstitution(
        [pkg_rushbott_moveit_config, 'launch', 'servo.launch.py'])

    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_description_launch)
    )
    controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(control_launch)
    )
    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(teleop_launch)
    )
    moveit2_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit2_launch)
    )
    
    # Rviz
    rviz_config = PathJoinSubstitution([pkg_rushbott_common_bringup, 'rviz', 'rushbott.rviz']) 

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d', rviz_config],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    # Create launch description and add actions
    ld = LaunchDescription()
    ld.add_action(robot_description)
    ld.add_action(controllers)
    ld.add_action(teleop)
    ld.add_action(moveit2_interface)
    ld.add_action(rviz)

    return ld