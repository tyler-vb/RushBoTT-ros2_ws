from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    pkg_rushbott_rover = get_package_share_directory('rushbott_rover')

    rover_controller_launch = PathJoinSubstitution(
        [pkg_rushbott_rover, 'launch', 'rover_controller.launch.py']
    )

    # diff_drive_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=[
    #         'diff_drive_controller',
    #         '--controller-manager-timeout',
    #         '30'
    #     ],
    #     output='screen',
    # )

    rover_joint_velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            'rover_joint_velocity_controller',
            '--controller-manager-timeout',
            '30'
        ],
        output='screen',
    )

    rover_joint_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            'rover_joint_position_controller',
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

    rover_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rover_controller_launch)
    )

    # diff_drive_controller_callback = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=joint_state_broadcaster_spawner,
    #         on_exit=[diff_drive_controller_spawner],
    #     )
    # )

    rover_joint_controllers_callback = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                rover_joint_velocity_controller_spawner,
                rover_joint_position_controller_spawner
            ]
        )
    )

    rover_controllers_callback = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=rover_controller
        )
    )

    ld = LaunchDescription()

    ld.add_action(joint_state_broadcaster_spawner)
    # ld.add_action(diff_drive_controller_callback)
    ld.add_action(rover_joint_controllers_callback)
    ld.add_action(rover_controllers_callback)

    return ld
