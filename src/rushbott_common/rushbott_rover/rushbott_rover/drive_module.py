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

class DriveModule(object):

    def __init__(
        self,
        name: str,
        steering_index: int,
        drive_index: int,
        x_position: float,
        y_position: float):

        self.name = name
        self.steering_index = steering_index
        self.drive_index = drive_index

        # Assume a vertical steering axis that goes through the center of the wheel (i.e. no steering offset)
        self.x_position = x_position
        self.y_position = y_position
        
    # Motors
    # Wheel
    # Sensors