# 🛠️ 4-Wheeled Robot Simulation with Obstacle Stop & Orange Box Detection (ROS 2 + Gazebo + OpenCV)

## 1. 📌 Project Overview

This project simulates a **4-wheeled differential drive robot** in **Gazebo**, integrated with **LIDAR** and a **camera** using **ROS 2 Humble**.  
The robot can:
- Stop when an obstacle is detected via **LIDAR**
- Detect an **orange-colored box** using a **camera** and **OpenCV**
- Move via **teleoperation** or programmed commands

---

## 2. 🔩 URDF File Overview

The robot is defined using a **XACRO-based URDF** (`robot.xacro`) and includes:
- A `base_link` body (rectangular box)
- Four wheels (two front + two rear), each with joints
- A **LIDAR sensor** mounted on top
- A **camera sensor** attached to the base

The URDF handles:
- Visual, collision, and inertial properties of all links
- Continuous joints for wheels (used by differential drive)
- Plugins for LIDAR and differential drive behavior in Gazebo

---

## 3. 🚧 Obstacle Detection using LIDAR

The obstacle detection node (`obstacle_stop.py`) works as follows:
- **Subscribes** to `/scan` topic from the LIDAR
- Reads the **front ranges** (center values of 360-degree scan)
- If any object is closer than **0.5 meters**, it:
  - Logs a warning
  - **Publishes zero velocity** to `/cmd_vel` to stop the robot

This ensures autonomous safety by halting the robot near obstacles.

---

## 4. 🟠 Orange Box Detection using Camera + OpenCV

The camera node (`camera_detect.py`) detects an orange box using these steps:
- **Subscribes** to the robot's camera topic `/bot_camera/image_raw`
- Converts the incoming image from ROS to OpenCV format
- Transforms the image from **BGR to HSV** for robust color filtering
- Applies a **binary mask** to extract orange regions
- Uses **contour detection** to locate the orange object
- If a large enough orange area is found, it logs that detection

This mimics a basic **vision-based perception system**.

---

## 5. 🧬 URDF - Published Topics & Their Subscribers

| **Topic**            | **Published by**             | **Subscribed by**                  |
|----------------------|------------------------------|------------------------------------|
| `/robot_description` | `robot_state_publisher`      | `spawn_entity.py` (for Gazebo)     |
| `/cmd_vel`           | `teleop_twist_keyboard` / Obstacle Node | `diff_drive` plugin (Gazebo)  |
| `/odom`              | `diff_drive` plugin          | Optional (e.g., SLAM/Navigation)   |
| `/scan`              | LIDAR Plugin (`libgazebo_ros_ray_sensor.so`) | `obstacle_stop.py` |
| `/bot_camera/image_raw` | Camera Plugin (`libgazebo_ros_camera.so`) | `camera_detect.py` (OpenCV)  |
| `/joint_states`      | `robot_state_publisher`      | Any visualizers / control nodes    |

---

## ✅ Final Outcome

- Launch robot in Gazebo with wall & orange box using:
  ```bash
  ros2 launch bot_description gazebo.launch.py
