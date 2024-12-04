from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    pkg_rushbott_common_bringup = get_package_share_directory('rushbott_common_bringup')

    # Rviz
    rviz_config = PathJoinSubstitution([pkg_rushbott_common_bringup, 'rviz', 'rushbott.rviz'])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '--display-config', rviz_config,
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    )

    # Define LaunchDescription variable
    ld = LaunchDescription()
    # Add nodes to LaunchDescription
    ld.add_action(rviz)

    return ld