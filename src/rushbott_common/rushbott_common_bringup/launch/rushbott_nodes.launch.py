from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():

    # Directories
    pkg_rushbott_common_bringup = get_package_share_directory('rushbott_common_bringup')
    pkg_rushbott_control = get_package_share_directory('rushbott_control')

    # Paths
    control_launch = PathJoinSubstitution(
        [pkg_rushbott_control, 'launch', 'control.launch.py']
    )

    # Controller
    diffdrive_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(control_launch)
    )

    # Create launch description and add actions
    ld = LaunchDescription()
    ld.add_action(diffdrive_controller)

    return ld