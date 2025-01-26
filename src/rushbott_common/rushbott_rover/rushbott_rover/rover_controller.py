# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from typing import List
import math

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

from .drive_module import DriveModule
from .steering_controller import SteeringController

class RoverController(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        # Declare all parameters
        self.declare_parameter("robot_base_frame", "base_footprint")
        self.declare_parameter("twist_topic", "cmd_vel")

        self.declare_parameter("position_controller_name", "position_controller")
        self.declare_parameter("velocity_controller_name", "velocity_controller")
        self.declare_parameter("publish_rate", 50.0)

        self.declare_parameter("steering_joints", ["joint1", "joint2"])
        self.declare_parameter("drive_joints", ["joint1", "joint2"])

        self.declare_parameter("wheel_radius", 0.5)
        self.declare_parameter("track_width", 40.0)
        self.declare_parameter("wheelbase", 40.0)

        self.declare_parameter("drive_velocity_limit", 10.0)
        self.declare_parameter("steering_angle_limit", 10.0)

        self.get_logger().info(f'Initializing rover controller ...')

        self.last_velocity_command: Twist = None
         # keep last position message to avoid inf value in steering angle data
        self.last_position_msg: Float64MultiArray = None

        self.robot_base_link = self.get_parameter("robot_base_frame").value

        # publish the module steering angle
        position_controller_name = self.get_parameter("position_controller_name").value
        steering_angle_publish_topic = "/" + position_controller_name + "/" + "commands"
        self.drive_module_steering_angle_publisher = self.create_publisher(
            Float64MultiArray,
            steering_angle_publish_topic,
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                durability=DurabilityPolicy.VOLATILE,
                depth=10))

        self.get_logger().info(
            f'Publishing steering angle changes on topic "{steering_angle_publish_topic}"'
        )

        # publish the module drive velocity
        velocity_controller_name = self.get_parameter("velocity_controller_name").value
        velocity_publish_topic = "/" + velocity_controller_name + "/" + "commands"
        self.drive_module_velocity_publisher = self.create_publisher(
            Float64MultiArray,
            velocity_publish_topic,
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                durability=DurabilityPolicy.VOLATILE,
                depth=10))

        self.get_logger().info(
            f'Publishing drive velocity changes on topic "{velocity_publish_topic}"'
        )

        # Create the controller that will determine the correct drive commands for the different drive modules
        # Create the controller before we subscribe to state changes so that the first change that comes in gets
        # registered
        self.drive_velocity_limit = self.get_parameter("drive_velocity_limit").value
        self.steeing_angle_limit = self.get_parameter("steering_angle_limit").value
        self.wheel_radius = self.get_parameter("wheel_radius").value
        self.drive_modules = self.get_drive_modules()
        self.controller = SteeringController(
            self.drive_modules,
            self.drive_velocity_limit,
            self.steeing_angle_limit,
            self.wheel_radius,
            self.write_log)
        self.get_logger().info(f'Loading controller...')

        # Create the timer that is used to ensure that we publish movement data regularly
        self.cycle_time_in_hertz = self.get_parameter("publish_rate").value
        self.get_logger().info(
            f'Publishing changes at fequency: "{self.cycle_time_in_hertz}" Hz'
        )
        self.timer = self.create_timer(
            1.0 / self.cycle_time_in_hertz,
            self.timer_callback,
            callback_group=None,
            clock=self.get_clock())
        self.i = 0

        # Finally listen to the cmd_vel topic for movement commands. We could have a message incoming
        # at any point after we register so we set this subscription up last.
        twist_topic = self.get_parameter("twist_topic").value
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            twist_topic,
            self.cmd_vel_callback,
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                durability=DurabilityPolicy.VOLATILE,
                depth=10))
        self.get_logger().info(
            f'Listening for movement commands on topic "{twist_topic}"'
        )

    def cmd_vel_callback(self, msg: Twist):
        if msg == None:
            return

        # If this twist message is the same as last time, then we don't need to do anything
        if self.last_velocity_command is not None:
            if msg.linear.x == self.last_velocity_command.linear.x and \
                msg.angular.z == self.last_velocity_command.angular.z:

                # The last command was the same as the current command. So just ignore it and move on.
                self.get_logger().info(
                    f'Received a Twist message that is the same as the last message. Taking no action. Message was: "{msg}"'
                )

                return

        self.get_logger().info(
            f'Received a Twist message that is different from the last command. Processing message: "{msg}"'
        )

        self.controller.update_drive_module_states(
                body_v = msg.linear.x,
                body_w = msg.angular.z
            )

        self.last_velocity_command = msg

    def get_drive_modules(self) -> List[DriveModule]:
        # Get the drive module information from the URDF and turn it into a list of drive modules.
        #
        # For now we don't read the URDF and just hard-code the drive modules
        track_width = self.get_parameter("track_width").value
        wheelbase = self.get_parameter("wheelbase").value
        steering_joints: List[str] = self.get_parameter("steering_joints").value + ["None"]
        drive_joints: List[str] = self.get_parameter("drive_joints").value + ["None"]
        
        # store the steering joints
        drive_modules: List[DriveModule] = []

        drive_module_name = "left_front"
        left_front = DriveModule(
            name=drive_module_name,
            steering_index=next((steering_joints.index(x) for x in steering_joints if drive_module_name in x), -1),
            drive_index=next((drive_joints.index(x) for x in drive_joints if drive_module_name in x), -1),
            x_position=0.5*wheelbase,
            y_position=0.5*track_width
        )
        drive_modules.append(left_front)

        self.get_logger().info(
            f'Configured drive module: "{left_front.name}" ' +
            f'with steering link: "{steering_joints[left_front.steering_index]}" ' +
            f'and drive link: "{drive_joints[left_front.drive_index]}" ' +
            f'and position: ["{left_front.x_position}", "{left_front.y_position}"]'
        )

        drive_module_name = "right_front"
        right_front = DriveModule(
            name=drive_module_name,
            steering_index=next((steering_joints.index(x) for x in steering_joints if drive_module_name in x), -1),
            drive_index=next((drive_joints.index(x) for x in drive_joints if drive_module_name in x), -1),
            x_position=0.5*wheelbase,
            y_position=-0.5*track_width
        )
        drive_modules.append(right_front)

        self.get_logger().info(
            f'Configured drive module: "{right_front.name}" ' +
            f'with steering link: "{steering_joints[right_front.steering_index]}" ' +
            f'and drive link: "{drive_joints[right_front.drive_index]}" ' +
            f'and position: ["{right_front.x_position}", "{right_front.y_position}"]'
        )

        drive_module_name = "left_middle"
        left_middle = DriveModule(
            name=drive_module_name,
            steering_index=next((steering_joints.index(x) for x in steering_joints if drive_module_name in x), -1),
            drive_index=next((drive_joints.index(x) for x in drive_joints if drive_module_name in x), -1),
            x_position=0,
            y_position=0.5*track_width
        )
        drive_modules.append(left_middle)

        self.get_logger().info(
            f'Configured drive module: "{left_middle.name}" ' +
            f'with steering link: "{steering_joints[left_middle.steering_index]}" ' +
            f'and drive link: "{drive_joints[left_middle.drive_index]}" ' +
            f'and position: ["{left_middle.x_position}", "{left_middle.y_position}"]'
        )

        drive_module_name = "right_middle"
        right_middle = DriveModule(
            name=drive_module_name,
            steering_index=next((steering_joints.index(x) for x in steering_joints if drive_module_name in x), -1),
            drive_index=next((drive_joints.index(x) for x in drive_joints if drive_module_name in x), -1),
            x_position=0,
            y_position=-0.5*track_width
        )
        drive_modules.append(right_middle)

        self.get_logger().info(
            f'Configured drive module: "{right_middle.name}" ' +
            f'with steering link: "{steering_joints[right_middle.steering_index]}" ' +
            f'and drive link: "{drive_joints[right_middle.drive_index]}" ' +
            f'and position: ["{right_middle.x_position}", "{right_middle.y_position}"]'
        )

        drive_module_name = "left_rear"
        left_rear = DriveModule(
            name=drive_module_name,
            steering_index=next((steering_joints.index(x) for x in steering_joints if drive_module_name in x), -1),
            drive_index=next((drive_joints.index(x) for x in drive_joints if drive_module_name in x), -1),
            x_position=-0.5*wheelbase,
            y_position=0.5*track_width       
        )
        drive_modules.append(left_rear)

        self.get_logger().info(
            f'Configured drive module: "{left_rear.name}" ' +
            f'with steering link: "{steering_joints[left_rear.steering_index]}" ' +
            f'and drive link: "{drive_joints[left_rear.drive_index]}" ' +
            f'and position: ["{left_rear.x_position}", "{left_rear.y_position}"]'
        )

        drive_module_name = "right_rear"
        right_rear = DriveModule(
            name=drive_module_name,
            steering_index=next((steering_joints.index(x) for x in steering_joints if drive_module_name in x), -1),
            drive_index=next((drive_joints.index(x) for x in drive_joints if drive_module_name in x), -1),
            x_position=-0.5*wheelbase,
            y_position=-0.5*track_width
        )
        drive_modules.append(right_rear)

        self.get_logger().info(
            f'Configured drive module: "{right_rear.name}" ' +
            f'with steering link: "{steering_joints[right_rear.steering_index]}" ' +
            f'and drive link: "{drive_joints[right_rear.drive_index]}" ' +
            f'and position: ["{right_rear.x_position}", "{right_rear.y_position}"]'
        )
        return drive_modules

    def timer_callback(self):
        drive_module_states = self.controller.get_desired_states()

        # Only publish movement commands if drive modules have states
        if len(drive_module_states) == 0:
            return

        velocity_msg = Float64MultiArray()
        drive_velocity_values = [a for a in drive_module_states[0][:-1]]
        velocity_msg.data = drive_velocity_values

        position_msg = Float64MultiArray()
        steering_angle_values = [a for a in drive_module_states[1][:-1]]
        position_msg.data = steering_angle_values

        # if there are some inf values in data publish last position instead (or update last position message)
        if (any(math.isinf(x) for x in position_msg.data)) and not (self.last_position_msg is None):
            position_msg = self.last_position_msg
        else:
            self.last_position_msg = position_msg

        # Publish the next steering angle and the next velocity sets. Note that
        # The velocity is published (very) shortly after the position data, which means
        # that the velocity could lag in very tight update loops.
        self.drive_module_steering_angle_publisher.publish(position_msg)
        self.drive_module_velocity_publisher.publish(velocity_msg)

    def write_log(self, text: str):
        self.get_logger().info(text)


def main(args=None):
    rclpy.init(args=args)
    node = RoverController('rover_controller')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()

    except(KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()  # Shut down the executor

if __name__ == "__main__":
   main()