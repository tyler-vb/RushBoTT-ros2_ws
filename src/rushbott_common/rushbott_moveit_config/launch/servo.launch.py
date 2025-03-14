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
            default_planning_pipeline="stomp"
        )
        .to_moveit_configs()
    )

    # Start the actual move_group node/action server
    servo_params = {
        "moveit_servo": ParameterBuilder("rushbott_moveit_config")
        .yaml("config/servo_parameters.yaml")
        .to_dict()
    }

    filter_update_period = {"update_period": 0.02}
    planning_group_name = {"planning_group_name": "arm"}

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        parameters=[
            servo_params,
            filter_update_period,
            planning_group_name,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        output="screen"
    )
    
    ld = LaunchDescription()
    ld.add_action(servo_node)

    return ld