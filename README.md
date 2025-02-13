# RushBoTT-ros2_ws

Only working on windows right now :'(

This repository uses docker+devcontainers to work. To get started:
1) Get WSL2 (Ubuntu) on your system if you don't already (https://learn.microsoft.com/en-us/windows/wsl/install)
2) Download Docker Desktop on Windows (https://docs.docker.com/desktop/setup/install/windows-install/)
3) Download VSCode, again on Windows (https://code.visualstudio.com/)
4) Install the Remote Development VSCode extension
5) Clone this repository into your WSL directory of choice.
```
git clone https://github.com/tyler-vb/RushBoTT-ros2_ws.git
```
6) With the repository opened in VS Code, press ctrl+shift+p and open "WSL: Connect to WSL" using the command prompt (you can also click on the little blue "><" icon in the bottom right)
7) Press ctrl+shift+p again (or "><") and open "Dev Containers: Reopen in Container" using the command prompt
8) Choose the devcontainer.json file you'd like to use to launch your container (you can choose between NVIDIA or Intel rendering, this only works with a linux booted machine (no wsl ;-;). If you don't have a GPU/Linux your choice doesn't matter)

Congrats! You have successfully opened this repository in a docker container.
You will know you were successful if you see "Dev Container: RushBoTT ROS2 Intel/NVIDIA" in the bottom left of your VS Code window.

-----

Here are some ROS2 terminal commands to run once you're in the container:

```
# ROS INITIALIZING (Run this when you start the container)
rosdep install --from-paths src --ignore-src -r -i -y --rosdistro jazzy
colcon build --symlink-install --continue-on-error --cmake-args -DBUILD_TESTING=ON

-----LAUNCH ROBOT DESCRIPTION-----
. ~/RushBoTT-ros2_ws/install/setup.sh
ros2 launch rushbott_common_bringup robot_description.launch.py

-----LAUNCH RVIZ2-----
. ~/RushBoTT-ros2_ws/install/setup.sh
ros2 launch rushbott_common_bringup rviz2.launch.py

-----LAUNCH JOINT STATE PUBLISHER GUI-----
. ~/RushBoTT-ros2_ws/install/setup.sh
ros2 run joint_state_publisher_gui joint_state_publisher_gui 

# START GAZEBO SIM
. ~/RushBoTT-ros2_ws/install/setup.sh
ros2 launch rushbott_gz_bringup sim.launch.py

# SPAWN ROBOT
. ~/RushBoTT-ros2_ws/install/setup.sh
ros2 launch rushbott_gz_bringup rushbott_spawn.launch.py

# LAUNCH GAZEBO AND RVIZ2 W/ RUSHBOTT (Run this one to play w/ robot, use teleop terminal to command robot)
. ~/RushBoTT-ros2_ws/install/setup.sh
ros2 launch rushbott_gz_bringup rushbott_gz.launch.py
```

sudo chmod 777 /dev/dri/renderD128
sudo add-apt-repository ppa:kisak/kisak-mesa
sudo apt update

. ~/RushBoTT-ros2_ws/install/setup.sh
ros2 launch rushbott_control control.launch.py