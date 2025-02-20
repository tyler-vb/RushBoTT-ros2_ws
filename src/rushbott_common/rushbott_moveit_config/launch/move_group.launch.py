from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder(robot_name='rushbott', package_name='rushbott_moveit_config')
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner", "stomp"],
            default_planning_pipeline="pilz_industrial_motion_planner"
        )
        .to_moveit_configs()
    )

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        arguments=["--ros-args", "--log-level", "info"],
        parameters=[moveit_config.to_dict(),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'start_state': {'content': 'config/initial_positions.yaml'}},
        ]
    )
    
    ld = LaunchDescription()
    ld.add_action(move_group_node)

    return ld
