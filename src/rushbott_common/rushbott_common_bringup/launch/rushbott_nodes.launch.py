from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():

    # Directories
    pkg_rushbott_control = get_package_share_directory('rushbott_control')

    # Paths
    control_launch = PathJoinSubstitution(
        [pkg_rushbott_control, 'launch', 'control.launch.py']
    )

    # Controllers
    controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(control_launch)
    )

    # Teleop
    teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        emulate_tty=True,
        prefix='xterm -hold -e',
        parameters=[{
            'stamped': True,
        }],
        remappings=[(
            '/cmd_vel', '/rover_controller/cmd_vel'
        )]
    )   

    # Create launch description and add actions
    ld = LaunchDescription()
    ld.add_action(teleop)
    ld.add_action(controllers)

    return ld