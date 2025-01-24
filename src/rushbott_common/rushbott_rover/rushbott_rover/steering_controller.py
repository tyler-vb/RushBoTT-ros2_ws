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
from .drive_module import DriveModule, DriveModuleDesiredValues

class SteeringController():

    def __init__(
            self,
            drive_modules: List[DriveModule],
            logger: Callable[[str], None]):

        self.modules = drive_modules
        self.logger = logger

        self.module_desired_states: List[DriveModuleDesiredValues] = None

        self.COT_limit = self.get_COT_limit()

    def get_COT_limit(self):
        for module in self.modules:
            limit = abs(max(module.x_position/math.tan(module.steering_motor_maximum_position) + module.y_position, self.COT_limit))
        return limit
    
    def get_drive_module_states(self) -> List[DriveModuleDesiredValues]:
        if self.module_desired_states is None:
            return []
        return self.module_desired_states

    def update_drive_module_states(self, body_v, body_w):
        states = []

        scale = 1.0

        if math.isclose(body_v, 0):
            states = [
                DriveModuleDesiredValues(
                    module.name,
                    0,
                    0
                ) for module in self.modules
            ]

        elif math.isclose(body_w, 0):
            for module in self.modules:
                scale = min(module.drive_motor_maximum_velocity / abs(drive_velocity), scale)
                state = DriveModuleDesiredValues(
                    module.name,
                    0,
                    drive_velocity)
                
                states.append(state)

        else:
            center_of_turning = body_v/body_w
            if abs(center_of_turning) >= self.COT_limit:
                center_of_turning = self.COT_limit
                body_w = body_v/center_of_turning

            for module in self.modules:

                distance_from_COT = math.sqrt((center_of_turning - module.y_position)**2 - module.x_position**2)

                drive_velocity = distance_from_COT*body_w/module.wheel_radius
                steering_angle = math.atan(-module.x_position, center_of_turning - module.y_position)

                scale = min(module.drive_motor_maximum_velocity / abs(drive_velocity), scale)

                state = DriveModuleDesiredValues(
                    module.name,
                    steering_angle,
                    drive_velocity)
                
                states.append(state)

        for state in states: 
            state.drive_velocity_in_radians_per_second *= scale

        self.module_desired_states = states