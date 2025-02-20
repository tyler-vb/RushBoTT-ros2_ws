from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch_param_builder import ParameterBuilder

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
    servo_params = {
        "moveit_servo": ParameterBuilder("rushbott_moveit_config")
        .yaml("config/servo_parameters.yaml")
        .to_dict()
    }

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        output="screen"
    )
    
    ld = LaunchDescription()
    ld.add_action(servo_node)

    return ld