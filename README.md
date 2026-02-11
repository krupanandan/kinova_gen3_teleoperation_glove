# Kinova Gen3 Teleoperation via Wearable Sensors
### *Real-Time Control of a 6-DOF Robotic Arm using an ESP8266 & MPU6050 Glove*

---

## 🎯 Project Overview
This project implements an intuitive teleoperation system for a **Kinova Gen3 robotic arm**. By utilizing a custom-built sensor glove, users can control the arm's 3D position and orientation in a simulated environment through natural hand gestures. 

The system bridges physical hardware (IMU and Rotary Encoders) with a **ROS 2 Humble** simulation using a custom-developed Inverse Kinematics (IK) translation layer.

## 📺 Demo
*(Insert a GIF or YouTube link here showing your glove moving the robot in Gazebo)*

---

## 🚀 Key Technical Highlights
- **Custom Inverse Kinematics Bridge:** Developed a real-time IK solver using `ikpy` that translates Cartesian hand coordinates into 7-joint trajectories, bypassing simulation-only interface limitations.
- **Multi-Threaded Architecture:** Utilized Python's `threading` library to decouple high-frequency serial data ingestion (200Hz) from the ROS 2 control loop (30Hz), ensuring zero-latency responsiveness.
- **Signal Processing & Noise Mitigation:** Implemented software deadzones and input normalization to filter raw MPU6050 sensor noise, resulting in smooth and stable robotic motion.
- **Asynchronous Gripper Control:** Integrated a ROS 2 Action Client to handle Robotiq gripper commands asynchronously, allowing the arm to move while the fingers are in motion.

---

## 🛠️ System Architecture
- **Hardware:** ESP8266 Microcontroller, MPU6050 (Pitch/Roll), Rotary Encoder (Z-axis/Yaw), IR Sensor (Gripper Toggle).
- **Software:** ROS 2 Humble, Gazebo Physics, IKPy, NumPy.
- **Communication:** Serial-over-USB at 3,000,000 Baud.

---

## 📦 Installation & Setup

### 1. Build the Docker Image
Run from the root folder:
```bash
bash docker_build.sh
```

### 2. Run the Docker Container

```bash
cd docker_run
bash docker_run.sh
```

### 3. Launch the Simulation

Inside the container, source the workspace and launch the fake hardware simulation:

```bash
source /opt/ros/humble/setup.bash
source /colcon_ws/install/setup.bash
ros2 launch kortex_bringup gen3.launch.py use_fake_hardware:=true
```

### 4. Running the Teleoperation Node

Ensure your sensor glove is connected to /dev/ttyUSB0 and execute the custom controller:

```bash
python3 Kinova_TeleOp.py
```

---

## 🎮 Controls

Pitch/Roll: Tilt hand forward/back or left/right to move the arm in the XY plane (Mode 0) or rotate the wrist (Mode 1).

Encoder Wheel: Controls the vertical Z-axis height.

IR Sensor: Toggles the Robotiq 2F-85 gripper between Open and Closed states.

Encoder Button: Resets the robot to the Home Position and recalibrates the glove offsets.

---

## 🎓 Academic Context

Developed as part of the Robotic Sensor Systems (RSS) course at RWTH Aachen University (WZL/MQ).

---

## ⚠️ Troubleshooting

Serial Permission: If /dev/ttyUSB0 is denied, run sudo chmod 666 /dev/ttyUSB0.

URDF Issues: The script automatically modifies the URDF to replace continuous joints with revolute for IK stability.

# 📘 End of README
