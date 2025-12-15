# Duckiebot Autonomous Driving in Isaac Sim

## 1. Project Overview
This project implements autonomous driving (Red Box Tracking) for a Duckiebot within the **NVIDIA Isaac Sim** virtual environment. It integrates **High-level** and **Low-level** control systems using **ROS2 Humble** and **Python**.

* **Development Environment:** WSL2 (Ubuntu 22.04), ROS2 Humble, NVIDIA Isaac Sim 4.x
* **Key Features:**
    * OpenCV-based Color Recognition & Tracking (Red Box Follower)
    * P-Control based Autonomous Driving Algorithm
    * Motor Control based on Inverse Kinematics

## 2. File Descriptions
* **`lsy_project.usd` (Simulation Stage)**
  - **[Core]** The NVIDIA Isaac Sim project file.
  - Includes the Duckiebot robot model, physics properties, and camera sensor settings.
  - Contains the built-in **Action Graph** responsible for communication between ROS2 and the simulator.

* **`duckie_driver.py` (High-level Control)**
  - Subscribes to the camera topic (`/duckie/camera/image_raw`) to detect red objects.
  - Controls speed based on distance (object area) and rotation based on position error using P-Control.
  - Publishes Twist (linear/angular velocity) commands to the `/duckie/cmd_vel` topic.

* **`duckie_low_level.py` (Low-level Control)**
  - Subscribes to `cmd_vel` commands and calculates **Inverse Kinematics**.
  - Converts target velocities into individual wheel rotation speeds (rad/s) based on kinematic parameters (L, R).
  - Publishes final motor commands to `/duckie/wheel_left` and `/duckie/wheel_right` topics.

## 3. How to Run

This project requires **WSL2 (Ubuntu 22.04)** for the ROS2 nodes and **Isaac Sim on Windows**.

### Step 1: Launch Simulation (Isaac Sim)
1. Launch **NVIDIA Isaac Sim** on Windows.
2. Go to `File` -> `Open` and select the **`lsy_project.usd`** file included in this repository.
3. Once loaded, click the **[Play]** button on the left toolbar.
   * *(Note: The ROS2 Bridge is activated only after clicking Play.)*

### Step 2: Run Low-level Control Node (Terminal 1)
Open a new WSL terminal and run the following commands:
```bash
# Source ROS2 environment (Required)
source /opt/ros/humble/setup.bash

# Navigate to project directory (Adjust path accordingly)
cd ~/path/to/your/project

# Run the node
python3 duckie_low_level.py

```
### Step 3: Run Autonomous Driving Driver (Terminal 2)
Open another WSL terminal and run:
```bash
# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Navigate to project directory
cd ~/path/to/your/project

# Run the node
python3 duckie_driver.py

```
### Step 4: Verification
OpenCV Window: Verify that the "Red Box View" window appears, showing the robot's vision (Green Bounding Box).

Isaac Sim: Confirm that the Duckiebot is autonomously following the red box.

Terminal Log: Check for logs such as Target Reached! Stopping.
