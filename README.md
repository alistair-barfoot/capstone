# Unitree G1 VR Teleoperation

This repository contains the software stack, control algorithms, and simulation environments for our engineering capstone project: **VR Teleoperation of a Unitree G1 Humanoid Robot**. 

<img src="Photos and Videos/pro live pipeline.JPG" width="700">

*Live Teleoperation of the G1 Humanoid*

Video of live teleoperation with headset view: https://www.youtube.com/watch?v=aiy3NT2tO8U

## Project Overview

The primary goal of this capstone is to establish a low-latency, highly intuitive teleoperation pipeline that allows a human operator to control a Unitree G1 humanoid robot in real-time using Virtual Reality. By mapping the operator's head and hand movements from a VR headset to the robot's kinematics, we enable complex manipulation and environmental interaction capabilities.

The system bridges high-level VR tracking data with low-level robot hardware, utilizing ROS2 for communication, CasADi/Pinocchio for high-speed Inverse Kinematics (IK), and the Unitree SDK for direct motor actuation.

---

## Repository Structure

Our project is divided into several distinct modules to separate the VR simulation, the ROS communication layer, and the low-level kinematic control logic. Please refer to the specific `README.md` files within each sub-directory for deep technical details, setup instructions, and execution steps.

* **[`/Control`](./Control)**
  Contains the core Python scripts (e.g., `armControlRos.py`) and setup files responsible for the high-frequency control loops. This includes the CasADi optimization-based IK solvers, the Pinocchio reduced-model kinematics, and the Unitree SDK2 interfaces that command the G1's joints and Dex3 grippers.

* **[`/Humanoid ROS`](./Humanoid%20ROS)**
  Houses the ROS2 workspace on the humanoid robot which includes the ROS-TCP library to convert the data from the Meta Quest Pro to ROS2 topics, as well as the Intel Realsense ROS2 library.

* **[`/Unity Environment`](./Unity%20Environment)**
  Contains the Unity project used to capture the operator's movements. This module handles tracking the VR headset and controllers, processing button states (like grip inputs), and publishing the `PoseStamped` and `Bool` messages over the TCP network to the humanoid.

* **[`/Photos and Videos`](./Photos%20and%20Videos)**
  A collection of media demonstrating the project in action. 

---

