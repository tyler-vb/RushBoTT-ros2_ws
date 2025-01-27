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

class SteeringController():

    def __init__(
            self,
            drive_modules: List[DriveModule],
            vel_limit: float,
            angle_limit: float,
            wheel_radius: float,
            logger: Callable[[str], None]):

        self.modules = drive_modules
        self.vel_limit = vel_limit
        self.angle_limit = angle_limit
        self.wheel_radius = wheel_radius
        self.logger = logger

        self.body_state: List[float] = None
        self.desired_states: List[List[float]] = None

        self.num_joints = self.get_num_joints()
        self.COT_limit = self.get_COT_limit()

    def get_num_joints(self):
        num_drive = 0
        num_steer = 0
        for module in self.modules:
            if module.drive_index != -1:
                num_drive += 1
            if module.steering_index != -1:
                num_steer += 1
        
        return [num_drive, num_steer]
    def get_COT_limit(self):
        limit = 0
        for module in self.modules:
            limit = max(abs(module.x_position/math.tan(self.angle_limit) + module.y_position), limit)
            limit = math.ceil(limit)
        self.logger(f"Calculated COT limit: {limit}")
        return limit
    
    def get_desired_states(self) -> List[List[float]]:
        if self.desired_states is None:
            return []
        return self.desired_states

    def update_drive_module_states(self, body_v, body_w):
        
        drive_velocities = [0] * (self.num_joints[0] + 1)
        steering_angles = [0] * (self.num_joints[1] + 1)

        scale = 1.0

        for module in self.modules:

            drive_velocity = 0
            steering_angle = 0

            if math.isclose(body_v, 0):
                pass

            elif math.isclose(body_w, 0):
                drive_velocity = body_v/self.wheel_radius
                
                scale = min(self.vel_limit / abs(drive_velocity), scale)

            else:
                center_of_turning = body_v/body_w

                if abs(center_of_turning) >= self.COT_limit:
                    center_of_turning = math.copysign(self.COT_limit, center_of_turning)
                    body_w = body_v/center_of_turning

                distance_from_COT = math.sqrt((center_of_turning-module.y_position)**2+(-module.x_position)**2)

                drive_velocity = math.copysign(distance_from_COT*body_w/self.wheel_radius, body_v)
                steering_angle = math.atan(-module.x_position/(center_of_turning - module.y_position))

                scale = min(self.vel_limit/abs(drive_velocity), scale)

            drive_velocities[module.drive_index] = drive_velocity   
            steering_angles[module.steering_index] = steering_angle

        drive_velocities = [scale * x for x in drive_velocities]

        self.desired_states = [drive_velocities, steering_angles]

        self.logger(f"drive velocities: {drive_velocities}")
        self.logger(f"steering angles: {steering_angles}")