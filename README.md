# 🤖 **Marker Detection with Robot and Camera Control**

## ✨ **Project Overview**

This project involves detecting **ArUco markers** in a simulated Gazebo environment. It is divided into two parts:  

1. **Robot Control**: The robot is moved within the environment to locate and identify markers.  
2. **Camera Control**: A fixed camera is controled to detect markers within its field of view.  

The implementation combines **robot motion control** and **vision-based marker detection** using ROS and Python.

---

## 🌟 **Features**

- **ArUco Marker Detection**: Identify marker IDs and positions.  
- **Robot Navigation**: Move the robot to align with markers.  
- **Camera Adjustment**: Control the camera to focus on marker detection.  
- **Image Processing**: Annotate detected markers on the output image.  
- **Gazebo Integration**: Simulate the environment and test functionality.  

---

## 🛠️ **Installation**

### 📋 **Prerequisites**

Before running this project, ensure you have the following installed:

- **ROS Noetic**  
- **Gazebo**  
- **Python 3** and necessary libraries:
  - `cv_bridge`
  - `aruco_ros`
  - `controller_manager`

### 📥 **Setup**

1. Clone the repository into your catkin workspace:  
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/li-dia/Experimental_Assignment_1.git
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash

2. To start the simulation, use one of the following launch files depending on the desired functionality:
   
#### Rotating Camera Mode:
   ```bash
   roslaunch robot_urdf camera_rotating.launch

#### Fixed Camera Mode:
   ```bash
   roslaunch robot_urdf camera_fixed.launch


