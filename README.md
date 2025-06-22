# Robotics Arm
## Overview
Directory is under development

## Installation
- Install ROS2-humble, current developement is going on ROS2-humble. The reason is MoveIt assistent is not compatible with ROS2-humble for now. 
- Although the package does use Panda xacro, the main point is to build any robot with URDF in moveit and gazebo. 

## To run
- If you want o run rviz demo file.
```bash
ros2 launch panda_moveit_config demo.launch.py
```

- If you want to modify and run rviz demo file.
```bash
ros2 launch panda_moveit_config moveit.launch.py
```

## Current Work in progress
- Could not contact service /controller_manager/list_controllers facing this error when launching moveit.launch.py
- checking how to make sure controller starts because sometimes it runs sometime it doesn't
### Progress 
- I changed output from "screen" to "log" to see if it works and now controller connects all the time. 
