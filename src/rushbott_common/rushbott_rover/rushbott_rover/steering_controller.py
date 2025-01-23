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

import math
from typing import Callable, List

# local
from .drive_module import DriveModule
from .geometry import Point
from .states import DriveModuleDesiredValues, BodyMotion

class SteeringController():

    def __init__(
            self,
            drive_modules: List[DriveModule],
            motion_limit: BodyMotion,
            logger: Callable[[str], None]):
            
        self.motion_limit = motion_limit
        self.modules = drive_modules
        self.logger = logger
        
        self.COT_limit = Point(0,0,0)
        self.get_angle_limits()


    def get_angle_limits(self):
        for module in self.modules:
            self.COT_limit = max(abs((self.COT_limit.x-module.xy_position.x)/math.tan(module.steering_motor_maximum_position) + module.xy_position.y), self.COT_limit)

    def update_current_motion(self, motion: BodyMotion):
        self.motion = motion

    def update_drive_module_states(self, desired_motion: BodyMotion):

        v = desired_motion.linear_velocity
        w = desired_motion.angular_velocity

        center_of_turning = Point(0, v/w, 0)

        states = []

        for module in self.modules:

            steering_angle = math.atan(center_of_turning.x - module.xy_position.x, center_of_turning.y - module.xy_position.y)


        
            distance_from_COT = math.sqrt((center_of_turning.y - module.xy_position.y)**2 + (center_of_turning.x - module.xy_position.x)**2)
            
            drive_velocity = distance_from_COT*v/module.wheel_radius
            steering_angle = math.atan(center_of_turning.x - module.xy_position.x, center_of_turning.y - module.xy_position.y)

            if not math.isclose(drive_velocity, 0.0, abs_)

            states.append(DriveModuleDesiredValues(
                module.name,
                self.clamp(drive_velocity, module.drive_motor_maximum_velocity),
                self.clamp(steering_angle, module.steering_motor_maximum_position)
            ))

            return states
        
    def clamp(self, value, limit):
        return max(-limit, min(value, limit))
