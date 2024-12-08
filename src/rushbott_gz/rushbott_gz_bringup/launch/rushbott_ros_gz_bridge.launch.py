from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='use_sim_time'),
]

def generate_launch_description():
    robot_name = LaunchConfiguration('robot_name')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # cmd_vel bridge
    cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='cmd_vel_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        arguments=[
            ['/cmd_vel' + '@geometry_msgs/msg/TwistStamped' + '[ignition.msgs.Twist'],
            ['/model/', robot_name, '/cmd_vel' +
             '@geometry_msgs/msg/TwistStamped' +
             ']ignition.msgs.Twist']
        ],
        remappings=[
            (['/cmd_vel'], 'cmd_vel'),
            (['/model/', robot_name, '/cmd_vel'],
             'diffdrive_controller/cmd_vel')
        ])

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(cmd_vel_bridge)
    return ld

