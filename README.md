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
    

## 5. Run the DWA Planner Node
In another terminal:

```
ros2 run dwa_planner dwa_planner_node.py
```
 **Recommended goal position**
 * `goal X = 2`
 * `goal y = 1`


## 6. Visualize in RViz
```
rviz2
```
Then:
* Click the **Add** button
* Select **By topic**
* Choose the **Marker** topic to visualize the planning output