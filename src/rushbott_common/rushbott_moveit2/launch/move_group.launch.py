from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    pkg_rushbott_common_bringup = get_package_share_directory('rushbott_common_bringup')

    moveit_config = (
        MoveItConfigsBuilder(robot_name='rushbott', package_name='rushbott_moveit2')
        .robot_description_semantic(file_path="config/rushbott.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_scene_monitor(
            publish_robot_description=False,
            publish_robot_description_semantic=True
        )
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner", "stomp"],
            default_planning_pipeline="pilz_industrial_motion_planner"
        )
        .pilz_cartesian_limits(file_path='config/pilz_cartesian_limits.yaml')
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

    # Rviz
    rviz_config = PathJoinSubstitution([pkg_rushbott_common_bringup, 'rviz', 'rushbott.rviz']) 
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d', rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
    )
    
    ld = LaunchDescription()
    ld.add_action(move_group_node)
    ld.add_action(rviz)

    return ld
