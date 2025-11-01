#  ROS2 Trajectory Tracking

A ROS2-based project that demonstrates **path smoothing** and **PID-based trajectory tracking** for a differential drive robot.  
The robot follows a smooth, spline-generated trajectory in **Gazebo** and visualizes its motion in **RViz**.

---

### ✨ Features
- 🧩 **Path Smoothing** using Natural Cubic Splines  
- ⚙️ **PID Controller** for real-time trajectory tracking  
- 🛰️ **ROS2 Integration** with `/odom` and `/cmd_vel` topics  
- 🧱 Tested in **Gazebo + RViz** simulation  
- 🚧 Ready for future **LiDAR-based obstacle avoidance** extension  

---

###  Simulation Result
<img width="755" height="423" alt="image" src="https://github.com/user-attachments/assets/295caf43-93b4-4b46-bc23-56a09679a8e5" />


---

### 📄 Tasks Implemented
1. **Task 1:** Path Smoothing using Cubic Splines  
2. **Task 2:** Trajectory Generation from discrete waypoints  
3. **Task 3:** PID Trajectory Tracking Controller  

---

### 🔧 Technologies Used
`ROS2 (Humble)` • `Python3` • `Gazebo` • `RViz` • `Numpy` • `Matplotlib` • `Pandas`

---

### 📂 File Descriptions

| **File** | **Description** |
|-----------|-----------------|
| **`path.py`** | Generates a smooth trajectory from discrete waypoints using a **Natural Cubic Spline**. The resulting `(x, y)` points are saved to a CSV file for the controller to follow. |
| **`path_marker.py`** | Publishes **visual markers** of the trajectory in **RViz**, allowing you to see the generated spline path for verification and visualization. |
| **`bot_pid.py`** | Implements the **PID-based trajectory tracking controller**. It subscribes to `/odom` for robot pose, computes velocity commands, and publishes them to `/cmd_vel` to make the robot follow the path. |
| **`gazebo.launch.py`** | Launches the **Gazebo simulation environment** along with the robot model and necessary ROS2 nodes (e.g., controller and visualization tools). This file automates the full simulation setup. |

---

### ⚙️ Setup Instructions

**System Requirements**
- Ubuntu 22.04 LTS  
- ROS2 Humble (or later)  
- Python 3.10+  
- Packages:  
  ```bash
  sudo apt install ros-humble-rviz2 ros-humble-gazebo-ros ros-humble-turtlebot3
  pip install numpy matplotlib pandas

> 🚀 *Developed by Pranav P as part of a robotics assignment on path smoothing and trajectory tracking.*
