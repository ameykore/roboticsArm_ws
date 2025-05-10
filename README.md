# Robotics Arm
## Overview
This repository contains ROS 2 packages for simulating and controlling the myCobot robotic arm using ROS 2 Control and MoveIt 2.

## Installation
- Install ROS 2 Jazzy(I tested on jazzy, you can use other version)
- To see the URDF file in RViz first.
    ```
    sudo apt-get install ros-${ROS_DISTRO}-urdf-tutorial
    ```
## To run
- Clone this repository
see output in rvis by
```
ros2 launch urdf_tutorial display.launch.py model:=/{pathtothisrepo}/mycobot_description/urdf/mycobot_280_urdf.xacro
```


## Todo
- Add Gazebo support
- RViz visualization for robot state and motion planning
- Pick and place task implementation using the MoveIt Task Constructor (MTC)
- 3D perception and object segmentation using point cloud da