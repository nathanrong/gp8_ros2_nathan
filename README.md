# Setup Steps #

1. Downloaded Ubuntu 24.04.5 LTS and created a bootable USB with Rufus using:

   * GPT
   * UEFI

   Note: use online tutorials -> https://ubuntu.com/tutorials/create-a-usb-stick-on-windows#1-overview
   Rufus Link: https://rufus.ie/en/
   Ubuntu Link: https://releases.ubuntu.com/24.04/?_gl=1*dkh0jw*_gcl_au*MTM3NjAwOTE4NC4xNzg5NzQ2NTU0Li0uLS4xNzg5NzQ2NTUzLjE2NjkyMDk1Mi4xNzg5NzQ2NTU0LjE3ODk3NDc5NTA.

2. Disabled Windows Device Encryption/BitLocker so Ubuntu could resize the Windows partition.

3. Installed Ubuntu alongside Windows, allocating approximately 200 GB to Ubuntu.

4. Installed Git:

   ```bash
   sudo apt update
   sudo apt install git -y
   ```

5. Created a ROS workspace and cloned the GP8 repository:

   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   git clone https://github.com/19522514/gp8_ros2.git
   ```

    Note: following tutorial -> https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html?utm_source=chatgpt.com

6. Installed ROS 2 Jazzy using the official ROS apt-source method.

7. Installed ROS 2 Jazzy Desktop and Jazzy MoveIt package:

   ```bash
   sudo apt install ros-jazzy-desktop -y
   sudo apt install ros-jazzy-moveit-py
   ```

8. Added ROS Jazzy to the shell startup file:

   ```bash
   echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

9. Installed Colcon:

   ```bash
   sudo apt install python3-colcon-common-extensions -y
   ```

10. Initialized and updated `rosdep`, then installed workspace dependencies.

11. Fixed `gp8_system_tests/CMakeLists.txt` by changing:

```cmake
DIRECTORY scripts src
```

to:

```cmake
DIRECTORY scripts
```

12. Fixed `gp8_gazebo/CMakeLists.txt` by changing:

```cmake
DIRECTORY config launch models worlds
```

to:

```cmake
DIRECTORY config launch worlds
```

13. Installed Gazebo message and ROS-Gazebo bridge dependencies:

```bash
sudo apt install ros-jazzy-gz-msgs-vendor ros-jazzy-ros-gz-bridge -y
```

14. Installed the MoveIt STOMP planner:

```bash
sudo apt install ros-jazzy-moveit-planners-stomp -y
```

15. Built the workspace:

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

16. Verified that the GP8 packages were available.

17. Successfully launched the complete system using:

```bash
cd ~/ros2_ws/src/gp8_ros2/gp8_bringup
source ~/ros2_ws/install/setup.bash
bash gp8_gazebo_and_moveit.sh
```

18. Successfully planned and executed GP8 motion through MoveIt in Gazebo.
