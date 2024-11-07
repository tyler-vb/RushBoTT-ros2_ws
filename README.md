# RushBoTT-ros2_ws
Hello, idk how to write a readme but here's a quick guide to get yourself situated (for windows)

1) Download Docker Desktop: https://docs.docker.com/desktop/setup/install/windows-install/
2) Get WSL on your system if you haven't already
3) Clone this repository into WSL. One way you can do this is by opening the directory where you'd like to clone this repository in WSL, then typing
   ```
   git clone https://github.com/tyler-vb/RushBoTT-ros2_ws.git
   ```
4) Install VS Code, then install the "Remote Explorer" extension
5) In VS Code, press ctrl+shift+p and open "WSL: Connect to WSL" using the command prompt
6) Open the cloned repository in VS Code WSL
7) Once again, press ctrl+shift+p and open "Dev Containers: Reopen in Container" using the command prompt

Congrats! You have successfully opened this repository in the RushBoTT ROS2 Container.
You will know you were successful if you see "Dev Container: RushBoTT ROS2 Container" in the bottom left of your VS Code window.

If you are interested in adding any new packages to the container, I HIGHLY recommend watching this video beforehand:
https://www.youtube.com/watch?v=RbP5cARP-SM&list=PLunhqkrRNRhaqt0UfFxxC_oj7jscss2qe&index=3&ab_channel=ArticulatedRobotics
