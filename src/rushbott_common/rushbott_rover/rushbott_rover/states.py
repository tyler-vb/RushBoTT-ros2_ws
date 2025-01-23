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

from .geometry import Vector3

class BodyMotion(object):

    def __init__(
        self,
        linear_x_velocity_in_meters_per_second: float,
        angular_z_velocity_in_radians_per_second: float
        ):
        self.linear_velocity = Vector3(linear_x_velocity_in_meters_per_second, 0.0, 0.0)
        self.angular_velocity = Vector3(0.0, 0.0, angular_z_velocity_in_radians_per_second)

class DriveModuleDesiredValues(object):

    def __init__(
        self,
        name: str,
        steering_angle_in_radians: float,
        drive_velocity_in_meters_per_second: float,
        ):
        self.name = name
        self.steering_angle_in_radians = steering_angle_in_radians
        self.drive_velocity_in_meters_per_second = drive_velocity_in_meters_per_second