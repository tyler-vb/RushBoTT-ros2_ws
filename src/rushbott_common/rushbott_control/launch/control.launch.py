from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():

    ackermannSteering_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            'ackermannSteering_controller',
            '--controller-manager-timeout',
            '30'
        ],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager-timeout',
            '30'
        ],
        output='screen',
    )

    ackermannSteering_controller_callback = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[ackermannSteering_controller_spawner],
        )
    )

    ld = LaunchDescription()

    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(ackermannSteering_controller_callback)

    return ld
