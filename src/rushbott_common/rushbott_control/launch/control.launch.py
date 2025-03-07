from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():

    pkg_rushbott_control = get_package_share_directory('rushbott_control')

    control_config = PathJoinSubstitution(
        [pkg_rushbott_control, 'config', 'control.yaml'])      

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[control_config],
        output="screen",
        condition=IfCondition(
            PythonExpression([
                "'",
                LaunchConfiguration("use_sim_time"),
                "' == 'false'"
            ])
        )
    )

    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            # 'rover_controller',
            'arm_controller',
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

    controller_callback = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[controller_spawner]
        )
    )

    ld = LaunchDescription()

    ld.add_action(control_node)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(controller_callback)
    return ld
