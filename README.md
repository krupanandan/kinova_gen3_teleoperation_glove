# ROS 2 Kortex – RSS Project
### *Simulation-Only Environment for Controlling a Virtual Kinova Gen3 Arm Using Sensor Data*

---

## 🎯 Purpose of This Workspace
This repository provides a **Dockerized ROS 2 environment** for working with the Kinova Gen3 robotic arm **entirely in simulation**.  
It is tailored for the **RSS Project**, where students must:

- Read sensor data from microcontrollers (e.g., ESP8266/MPU6050)  
- Send this data to ROS 2 over Wi-Fi  
- Map sensor values to robot motions  
- Control a **simulated Kinova Gen3 arm** in real time  

**No real robot is required.**  
**No IP address or hardware connection is needed.**

This workspace includes:
- **URDF visualization**
- **ros2_control with fake hardware** (primary mode)
- **MoveIt2 for motion planning**
- **Gazebo/Ignition for 3D simulation**

---

## 📦 Installation Instructions

### 1. Clone the Repository

Each project group has its own repository named:

**RSS_WS26_Project_Group_<GROUP_NUMBER>**

To clone your group’s repository, use the following command (replacing <GROUP_NUMBER> with the number of your group):

```bash
git clone --recurse-submodules https://git-ce.rwth-aachen.de/wzl-mq-ms/forschung-lehre/robotic-sensor-systems/rss_ws26_project_group_<GROUP_NUMBER>.git

cd rss_ws26_project_group_<GROUP_NUMBER>
```

If you forgot `--recurse-submodules`:

```bash
git submodule update --init --recursive
```

---

### 2. Build the Docker Image

Run from the root folder:

```bash
bash docker_build.sh
```

This builds the image:

```
ros2-kortex:latest
```

---

### 3. Run the Docker Container

```bash
cd docker_run
bash docker_run.sh
```

This opens a ready-to-use ROS 2 Humble environment.

---

# 🤖 Running the Simulated Kinova Robot

### Source the environment
Inside the container:

```bash
source /opt/ros/humble/setup.bash
source /colcon_ws/install/setup.bash
```

---

# 🔷 Option 1 — URDF Visualization Only

```bash
ros2 launch kortex_description view_robot.launch.py
```

Arguments:
- `robot_type:=gen3`
- `dof:=7`
- `gripper:=robotiq_2f_85`

---

# 🔷 Option 2 — Fake Hardware (Recommended for RSS Project)

This is the **primary mode** for the assignment.

It loads:
- ros2_control controllers  
- Fake hardware interface  
- Joint state publisher  
- Action servers  
- Command interfaces  
- Full TF tree  

Launch:

```bash
ros2 launch kortex_bringup gen3.launch.py     use_fake_hardware:=true
```

Provides ROS interfaces such as:

- `/joint_states`
- `/joint_trajectory_controller/command`
- `/joint_trajectory_controller/follow_joint_trajectory`

Perfect for real-time control from sensors.

---

# 🔷 Option 3 — MoveIt2 Simulation

```bash
ros2 launch kinova_gen3_7dof_robotiq_2f_85_moveit_config sim.launch.py     use_sim_time:=true
```

MoveIt2 enables:

- Trajectory generation  
- Collision checking  
- Visual planning  

---

# 🛰️ Integrating Sensors (Project Goal)

Students typically:

### 1. Publish sensor data to ROS 2
Example:

```bash
ros2 topic pub /my_sensor std_msgs/Float32 "data: 0.8"
```

### 2. Map sensor → joint command
Conceptual flow:

```
sensor value → normalization → position target → trajectory command
```

### 3. Send commands to the robot

```bash
ros2 topic pub /joint_trajectory_controller/commands   trajectory_msgs/msg/JointTrajectory "..."
```

The fake hardware simulates execution.

---

# 🛠 Useful ROS 2 Commands

```bash
ros2 topic list
ros2 topic echo /joint_states
ros2 control list_controllers
rviz2
```

---

# ⚠️ Important Notes

- ❌ Do NOT connect to a physical robot  
- ❌ Do NOT set `robot_ip`  
- ❌ Do NOT install hardware drivers  

This workspace is purely for simulation.

---

# 🧰 Troubleshooting

### RViz does not start
```bash
rviz2 --disable-qt5-fix
```

### Topics missing
```bash
source /colcon_ws/install/setup.bash
```

### DDS issues
```bash
export ROS_DOMAIN_ID=5
```

---

# 🌐 Optional: Gazebo/Ignition Simulation

Advanced students may explore 3D physics simulation.

Example Ignition launch (if configured):

```bash
ros2 launch kortex_description gen3_ignition.launch.py
```

Or include URDF manually in custom Gazebo worlds.

This is not required for the RSS Project but is available for exploration.

---

# 📘 End of README
