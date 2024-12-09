from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

ARGUMENTS = [

]

def generate_launch_description():

    pkg_rushbott_control = get_package_share_directory("rushbott_control")

    control_params_file = PathJoinSubstitution(
        [pkg_rushbott_control,  "config", "control.yaml"]
    )

    # Define nodes
    diffdrive_controller = Node(
        package="controller_manager",
        executable="spawner",
    )

    ld = LaunchDescription(ARGUMENTS)

    ld.add_action(diffdrive_controller)

    return ld
