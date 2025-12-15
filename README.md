# Duckiebot Autonomous Driving in Isaac Sim

## 1. Project Overview
This project implements autonomous driving (Red Box Tracking) for a Duckiebot within the **NVIDIA Isaac Sim** virtual environment. It integrates **High-level** and **Low-level** control systems using **ROS2 Humble** and **Python**.

* **Development Environment:** WSL2 (Ubuntu 22.04), ROS2 Humble, NVIDIA Isaac Sim 4.x
* **Key Features:**
    * **Autonomous Driving:** OpenCV-based Color Recognition & Tracking (Red Box Follower)
    * **High-level Control:** Manual driving support via Keyboard Teleoperation & P-Control Algorithm
    * **Low-level Control:** Motor Control based on Inverse Kinematics
    * **LED Control Logic:** Publishes status-based LED color commands via ROS2 topics

## 2. File Descriptions
* **`lsy_project.usd` (Simulation Stage)**
  - **[Core]** The NVIDIA Isaac Sim project file.
  - Includes the Duckiebot robot model, physics properties, and camera sensor settings.
  - Contains the built-in **Action Graph** responsible for communication between ROS2 and the simulator.

* **`duckie_driver.py` (High-level Control - Auto)**
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
Open a new WSL terminal and run the following commands to start the motor driver.
```bash
# Source ROS2 environment (Required)
source /opt/ros/humble/setup.bash

# Navigate to project directory (Adjust path accordingly)
cd ~/path/to/your/project

# Run the node
python3 duckie_low_level.py

```
### Step 3: Choose Control Mode (Terminal 2)
You can choose between Autonomous Mode or Manual Keyboard Mode.

Option A: Autonomous Driving (Red Box Follower)
Runs the computer vision algorithm to track red objects.

```bash
# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Navigate to project directory
cd ~/path/to/your/project

# Run the driver
python3 duckie_driver.py
```
Option B: Manual High-level Control (Keyboard)
Allows you to drive the Duckiebot manually using keyboard commands.

Install the Teleop Package (If not installed):


```bash
sudo apt-get install ros-humble-teleop-twist-keyboard

```
Run the Keyboard Controller:

Note: We remap the topic to match our robot's configuration.


```bash
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/duckie/cmd_vel
```
Controls: i (Forward), k (Stop), j (Left), l (Right), , (Backward)


### Step 4: LED Control Test (Terminal 3)
You can test the LED control logic by publishing a Vector3 message manually.

Red Light:


```bash
source /opt/ros/humble/setup.bash
ros2 topic pub --once /duckie/led_inner geometry_msgs/msg/Vector3 "{x: 1.0, y: 0.0, z: 0.0}"

```
Green Light:

```bash

ros2 topic pub --once /duckie/led_inner geometry_msgs/msg/Vector3 "{x: 0.0, y: 1.0, z: 0.0}"
```
### Step 5: Verification
OpenCV Window: Verify that the "Red Box View" window appears (in Option A).

Isaac Sim: Confirm that the Duckiebot moves according to the selected control mode.

Terminal Log: Check for logs such as Target Reached! Stopping.
