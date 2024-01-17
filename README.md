# Teleop-Twist-Controller2

[![License: CC BY-NC-SA 4.0](https://img.shields.io/badge/License-CC_BY--NC--SA_4.0-lightgrey.svg)](https://creativecommons.org/licenses/by-nc-sa/4.0/)
[![Ubuntu:Focal](https://img.shields.io/badge/Ubuntu-Focal-brightgreen)](https://releases.ubuntu.com/focal/)
[![ROS:Foxy](https://img.shields.io/badge/ROS-Foxy-blue)](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)

"teleop_twist_controller" is a joystick control package utilized for the movement of ROS2 robots.

## Requirements

- Joystick
   ```
   sudo apt-get install joystick jstest-gtk
   ```
- Pygame
   ```
   python3 -m pip install -U pygame==2.5.2 --user
   ```
- Xbox Controller
   ```
   sudo apt-add-repository -y ppa:rael-gc/ubuntu-xboxdrv
   ```
   ```
   sudo apt-get update
   ```
   ```
   sudo apt-get install xboxdrv
   ```
   ```
   sudo sh -c "echo blacklist xpad >> /etc/modprobe.d/blacklist.conf"
   ```
   ```
   sudo modprobe xpad
   ```

## Install and Build

1. Navigating to the "src" directory within your workspace :
   ```
   cd ~/dev_ws/src
   ```
2. Clone teleop_twist_controller2 package for github :
   ```
   git clone https://github.com/wenjiu2001/Teleop-Twist-Controller2.git teleop_twist_controller2
   ```
3. Build teleop_twist_controller2 package :
   ```
   cd ~/dev_ws && colcon build --symlink-install
   ```
4. Package environment setup :
   ```
   source ./install/setup.bash
   ```

## How to Use

Adjustments can be made by modifying the following parameters:

| Parameter name | Data Type | detail                                                       |
| -------------- | ------- | ------------------------------------------------------------ |
| device_number| int | Set the controller to be used. <br/>default: `1` |
| speed        | float | Set the maximum and minimum linear velocities. <br/>default: `0.26` |
| turn         | float | Set the maximum and minimum angular velocities. <br/>default: `1.82` |
| cmd_vel_topic| string | Set the name of the topic to be published. <br/>default: `/cmd_vel` |
| repeat_rate  | float | Set the frequency of continuous `cmd_vel_topic` updates. <br/>default: `0.0` |

- Activate joystick control :
   ```
   ros2 launch teleop_twist_controller2 teleop_twist_controller2.launch.py
   ```
   
## References

- Install Gazebo using Ubuntu packages (https://classic.gazebosim.org/tutorials?tut=install_ubuntu)
- TurtleBot3 Simulation (https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)
- teleop_twist_keyboard (https://wiki.ros.org/teleop_twist_keyboard)
- pygame (https://www.pygame.org/news)
