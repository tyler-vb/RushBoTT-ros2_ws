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

class DriveModuleDesiredValues(object):

    def __init__(
        self,
        name: str,
        steering_angle_in_radians: float,
        drive_velocity_in_meters_per_second: float,
        ):
        self.name = name
        self.steering_angle_in_radians = steering_angle_in_radians
        self.drive_velocity_in_radians_per_second = drive_velocity_in_meters_per_second

class DriveModule(object):

    def __init__(
        self,
        name: str,
        steering_link: str,
        drive_link: str,
        x_position: float,
        y_position: float,
        wheel_radius: float,
        steering_motor_maximum_position: float,
        drive_motor_maximum_velocity: float):

        self.name = name

        self.steering_link_name = steering_link
        self.driving_link_name = drive_link

        # Assume a vertical steering axis that goes through the center of the wheel (i.e. no steering offset)
        self.x_position = x_position
        self.y_position = y_position
        self.wheel_radius = wheel_radius

        self.steering_motor_maximum_position = steering_motor_maximum_position
        self.drive_motor_maximum_velocity = drive_motor_maximum_velocity

    # Motors
    # Wheel
    # Sensors