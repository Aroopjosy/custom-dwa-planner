# DWA Local Planner (ROS 2, Python)

This is a complete **Dynamic Window Approach (DWA) Local Planner** implementation in **ROS 2 (Python)** for **TurtleBot3 Burger** in Gazebo.

## 🚀 Quick Start

## 1. Create New Workspace

```bash
mkdir -p turtle_ws/src
cd turtle_ws/src
 ```

## 2. Clone the repository and Build

   ```bash
   git https://github.com/Aroopjosy/custom-dwa-planner.git .
   cd ..
   colcon build --symlink-install
   ```

## 3 open .bashrc add 

    
    export TURTLEBOT3_MODEL=burger
    source ~/turtle_ws/install/setup.bash

## 4 open terminal and run

    ```bash
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py 
    ```
# 5 run it 
    ```bash
    ros2 run dwa_planner dwa_planner_node.py
    ```

    
    input goal location recommend x = 2 and y = 1

    
