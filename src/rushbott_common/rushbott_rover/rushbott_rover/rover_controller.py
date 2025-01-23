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
from rclpy.clock import Clock, Time
from rclpy.duration import Duration as TimeDuration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from tf2_geometry_msgs import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import tf2_ros

from builtin_interfaces.msg import Duration as MsgDuration
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .geometry import Point
from .drive_module import DriveModule
from .steering_controller import SteeringController
from .states import BodyMotion

class RoverController(Node):
    def __init__(self):
        super().__init__("publisher_velocity_controller")
        # Declare all parameters
        self.declare_parameter("robot_base_frame", "base_footprint")
        self.declare_parameter("twist_topic", "cmd_vel")

        self.declare_parameter("position_controller_name", "position_controller")
        self.declare_parameter("velocity_controller_name", "velocity_controller")
        self.declare_parameter("cycle_fequency", 50)

        self.declare_parameter("steering_joints", ["joint1", "joint2"])
        self.declare_parameter("drive_joints", ["joint1", "joint2"])

        self.get_logger().info(f'Initializing rover controller ...')

        self.last_velocity_command: Twist = None

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

        # keep last position message to avoid inf value in steering angle data
        self.last_position_msg: Float64MultiArray = None

        # Create the controller that will determine the correct drive commands for the different drive modules
        # Create the controller before we subscribe to state changes so that the first change that comes in gets
        # registered
        self.get_logger().info(f'Storing drive module information...')
        self.drive_modules = self.get_drive_modules()
        self.controller = SteeringController(self.drive_modules, self.write_log)

        # Create the timer that is used to ensure that we publish movement data regularly
        self.cycle_time_in_hertz = self.get_parameter("cycle_fequency").value
        self.get_logger().info(
            f'Publishing changes at fequency: "{self.cycle_time_in_hertz}" Hz'
        )

        self.timer = self.create_timer(
            1.0 / self.cycle_time_in_hertz,
            self.timer_callback,
            callback_group=None,
            clock=self.get_clock())
        self.i = 0

        # Listen for state changes in the drive modules
        joint_state_topic = "joint_states"
        self.state_change_subscription = self.create_subscription(
            JointState,
            joint_state_topic,
            self.joint_states_callback,
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                durability=DurabilityPolicy.VOLATILE,
                depth=10)
        )

        self.get_logger().info(
            f'Listening for drive module state changes on "{joint_state_topic}"'
        )

        # Initialize the drive modules
        self.last_drive_module_state = self.initialize_drive_module_states(self.drive_modules)

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

        # When we get a stream of command it is possible that each command is slightly different (looking at you ROS2 nav)
        # This means we reset the starting time of the change profile each time, which starts the process all over
        # Because we don't take the current steering velocity / drive acceleration into account we assume that we
        # start from rest. That is wrong. We should be starting from a place where we have the current
        # steering velocity / drive acceleration.

        self.store_time_and_update_controller_time()

        # this is where i left off, I think I need to make this get saved in the controller so when you try to get information
        # from the controller after a tick you are able to.
        self.controller.get_drive_module_states(
            BodyMotion(
                msg.linear.x,
                msg.angular.z
            )
        )

        self.last_velocity_command = msg
        self.last_velocity_command_received_at = self.last_recorded_time

    def get_drive_modules(self) -> List[DriveModule]:
        # Get the drive module information from the URDF and turn it into a list of drive modules.
        #
        # For now we don't read the URDF and just hard-code the drive modules
        track_width = 0.35
        wheelbase = 0.30
        

        # store the steering joints
        steering_joint_names = self.get_parameter("steering_joints").value
        steering_joints = []
        for name in steering_joint_names:
            steering_joints.append(name)
            self.get_logger().info(
                f'Discovered steering joint: "{name}"'
            )

        # store the drive joints
        drive_joint_names = self.get_parameter("drive_joints").value
        drive_joints = []
        for name in drive_joint_names:
            drive_joints.append(name)
            self.get_logger().info(
                f'Discovered drive joint: "{name}"'
            )

        drive_modules: List[DriveModule] = []
        drive_module_name = "left_front"
        left_front = DriveModule(
            name=drive_module_name,
            steering_link=next((x for x in steering_joints if drive_module_name in x), "joint_steering_{}".format(drive_module_name)),
            drive_link=next((x for x in drive_joints if drive_module_name in x), "joint_drive_{}".format(drive_module_name)),
            xy_position=Point(0.5 * wheelbase, 0.5 * track_width, 0.0),
        )
        drive_modules.append(left_front)

        self.get_logger().info(
            f'Configured drive module: "{left_front.name}" ' +
            f'with steering link: "{left_front.steering_link_name}" ' +
            f'and drive link: "{left_front.driving_link_name}" ' +
            f'and position: ["{left_front.xy_position.x}", "{left_front.xy_position.y}"]'
        )

        drive_module_name = "right_front"
        right_front = DriveModule(
            name=drive_module_name,
            steering_link=next((x for x in steering_joints if drive_module_name in x), "joint_steering_{}".format(drive_module_name)),
            drive_link=next((x for x in drive_joints if drive_module_name in x), "joint_drive_{}".format(drive_module_name)),
            xy_position=Point(0.5 * wheelbase, -0.5 * track_width, 0.0),
        )
        drive_modules.append(right_front)

        self.get_logger().info(
            f'Configured drive module: "{right_front.name}" ' +
            f'with steering link: "{right_front.steering_link_name}" ' +
            f'and drive link: "{right_front.driving_link_name}" ' +
            f'and position: ["{right_front.xy_position.x}", "{right_front.xy_position.y}"]'
        )

        drive_module_name = "left_middle"
        left_middle = DriveModule(
            name=drive_module_name,
            drive_link=next((x for x in drive_joints if drive_module_name in x), "joint_drive_{}".format(drive_module_name)),
            xy_position=Point(0, 0.5 * track_width, 0.0),
        )
        drive_modules.append(left_middle)

        self.get_logger().info(
            f'Configured drive module: "{left_middle.name}" ' +
            f'with drive link: "{left_middle.driving_link_name}" ' +
            f'and position: ["{left_middle.xy_position.x}", "{left_middle.xy_position.y}"]'
        )

        drive_module_name = "right_middle"
        right_middle = DriveModule(
            name=drive_module_name,
            drive_link=next((x for x in drive_joints if drive_module_name in x), "joint_drive_{}".format(drive_module_name)),
            xy_position=Point(0, -0.5 * track_width, 0.0),
        )
        drive_modules.append(right_middle)

        self.get_logger().info(
            f'Configured drive module: "{right_middle.name}" ' +
            f'with drive link: "{right_middle.driving_link_name}" ' +
            f'and position: ["{right_middle.xy_position.x}", "{right_middle.xy_position.y}"]'
        )

        drive_module_name = "left_rear"
        left_rear = DriveModule(
            name=drive_module_name,
            steering_link=next((x for x in steering_joints if drive_module_name in x), "joint_steering_{}".format(drive_module_name)),
            drive_link=next((x for x in drive_joints if drive_module_name in x), "joint_drive_{}".format(drive_module_name)),
            xy_position=Point(-0.5 * wheelbase, 0.5 * track_width, 0.0),
            
        )
        drive_modules.append(left_rear)

        self.get_logger().info(
            f'Configured drive module: "{left_rear.name}" ' +
            f'with steering link: "{left_rear.steering_link_name}" ' +
            f'and drive link: "{left_rear.driving_link_name}" ' +
            f'and position: ["{left_rear.xy_position.x}", "{left_rear.xy_position.y}"]'
        )

        drive_module_name = "right_rear"
        right_rear = DriveModule(
            name=drive_module_name,
            steering_link=next((x for x in steering_joints if drive_module_name in x), "joint_steering_{}".format(drive_module_name)),
            drive_link=next((x for x in drive_joints if drive_module_name in x), "joint_drive_{}".format(drive_module_name)),
            xy_position=Point(-0.5 * wheelbase, -0.5 * track_width, 0.0),
        )
        drive_modules.append(right_rear)

        self.get_logger().info(
            f'Configured drive module: "{right_rear.name}" ' +
            f'with steering link: "{right_rear.steering_link_name}" ' +
            f'and drive link: "{right_rear.driving_link_name}" ' +
            f'and position: ["{right_rear.xy_position.x}", "{right_rear.xy_position.y}"]'
        )

        return drive_modules

    def store_time_and_update_controller_time(self):
        time: Time = self.get_clock().now()
        seconds = time.nanoseconds * 1e-9
        self.controller.on_tick(seconds)
        self.last_recorded_time = time

    def timer_callback(self):
        self.store_time_and_update_controller_time()

        drive_module_states = self.controller.get_drive_module_states()

        # Only publish movement commands if there is a trajectory
        if len(drive_module_states) == 0:
            return

        position_msg = Float64MultiArray()
        steering_angle_values = [a.steering_angle_in_radians for a in drive_module_states]
        position_msg.data = steering_angle_values

        velocity_msg = Float64MultiArray()
        drive_velocity_values = [a.drive_velocity_in_radians_per_second for a in drive_module_states]
        velocity_msg.data = drive_velocity_values

        # if there are some inf values in data publish last position instead (or update last position message)
        if (any(math.isinf(x) for x in position_msg.data)) and not (self.last_position_msg is None):
            position_msg = self.last_position_msg
        else:
            self.last_position_msg = position_msg

        # Publish the next steering angle and the next velocity sets. Note that
        # The velocity is published (very) shortly after the position data, which means
        # that the velocity could lag in very tight update loops.
        #self.get_logger().info(f'Publishing steering angle data: "{position_msg}"')
        self.drive_module_steering_angle_publisher.publish(position_msg)

        #self.get_logger().info(f'Publishing velocity angle data: "{velocity_msg}"')
        self.drive_module_velocity_publisher.publish(velocity_msg)

        self.last_control_update_send_at = self.last_recorded_time

    def write_log(self, text: str):
        self.get_logger().info(text)


def main(args=None):
    rclpy.init(args=args)

    pub = RoverController()

    rclpy.spin(pub)
    pub.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()