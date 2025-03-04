from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description(): 
    
    # Teleop
    rover_teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        output='screen',
        emulate_tty=True,
        prefix='xterm -hold -e',
        parameters=[{
            'stamped': True,
        }],
        remappings=[(
            '/cmd_vel', '/rover_controller/cmd_vel'
        )])   
    
    arm_teleop = Node(
        package='rushbott_moveit_config',
        executable='servo_keyboard_input',
        output='screen',
        emulate_tty=True,
        prefix='xterm -hold -e',
        )   

    # Create launch description and add actions
    ld = LaunchDescription()
    ld.add_action(rover_teleop)
    ld.add_action(arm_teleop)

    return ld