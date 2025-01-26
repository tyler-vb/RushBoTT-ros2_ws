
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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='use_sim_time'),
]

def generate_launch_description():

    #Directories
    pkg_rushbott_rover = get_package_share_directory('rushbott_rover')

    # Paths
    config = PathJoinSubstitution(
        [pkg_rushbott_rover, 'config', 'rover.yaml']
    )

    rover_controller = Node(
        package="rushbott_rover",
        executable="rover_controller",
        name="rover_controller",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            config
        ],
        output="both",
    )

    ld = LaunchDescription(ARGUMENTS)
    # Add nodes to LaunchDescription
    ld.add_action(rover_controller)
    return ld
