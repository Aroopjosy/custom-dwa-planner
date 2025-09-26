# DWA Local Planner (ROS 2, Python)

This is a complete **Dynamic Window Approach (DWA) Local Planner** implementation in **ROS 2 (Python)** for **TurtleBot3 Burger** in Gazebo.

## 🚀 Quick Start

## 1. Create New Workspace

```
mkdir -p turtle_ws/src
cd turtle_ws/src
 ```

## 2. Clone the repository and Build

```
git clone https://github.com/Aroopjosy/custom-dwa-planner.git .
cd ..
colcon build --symlink-install
```

## 3. Setup Environment Variables
Open your ~/.bashrc file and add the following lines:

```
export TURTLEBOT3_MODEL=burger
source ~/turtle_ws/install/setup.bash
```
Apply the changes:

```
source ~/.bashrc
```
## 4. Launch Gazebo with TurtleBot3 World
In a new terminal:

```
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

```
## 5. Launch DWA planner node, rviz
In another terminal:

```
ros2 launch dwa_planner dwa_map.launch.py
```
## 6. Give Goal location

* click `2D Goal pose` and drag to set open area of the map,
  
✨ Now your TurtleBot3 is started to navigate using the custom DWA local planner!
  
## 🎯 Goal Reached Indicator

* When the robot successfully reaches the target position,
the **goal arrow in RViz automatically changes its color to green**,
giving you a clear visual confirmation of success. ✅