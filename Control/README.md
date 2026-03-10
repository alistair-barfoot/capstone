# G1 VR Teleoperation Controller

This repository contains a ROS2-based controller for the real-time VR teleoperation of a Unitree G1 humanoid robot. It acts as the bridge between VR tracking data and the robot's low-level hardware, utilizing CasADi and Pinocchio for rapid Inverse Kinematics (IK) solving and the Unitree SDK for motor control.

## Mujuco Simulation

## Prerequisites

Before running the controller, ensure the following dependencies are installed:
* **ROS2** (Tested with Humble and Foxy)
* **Unitree SDK2** (`unitree_sdk2py`)
* **Pinocchio & CasADi** (Must be linked together correctly, most easily done using openRobots version)
* **Python 3.10+**
* The `g1_29dof.urdf` file must be located in the same directory as the execution script.

## Environment Setup

A dedicated shell script (`controlSetup.sh`) is provided to streamline the initialization of the workspace.

1. Ensure you have a Python virtual environment **PROVIDE SETUP** named `armControl` initialized in your workspace.
2. Verify that your Unitree ROS2 workspace is located at `~/unitree_ros2`.
3. Source the environment setup script to configure all necessary paths for Pinocchio, CasADi, and ROS2: (double check path locations match the setup file)
   `source controlSetup.sh`

## Running the Code

1. Ensure the robot is powered on and connected to the computer's ethernet port (ideally through a router, but can be configured directly from humanoid to the computer)
2. Launch VR tracking on a Meta Quest Pro headset that publishes to the `/pos_rot_left` and `/pos_rot_left_status` topics.
3. Execute the main Python controller:
   `python armControlRos.py`
4. **Shutdown:** Press `Ctrl+C`. 
5. The script includes a safe shutdown sequence that automatically damps the motors. **WARNING** It currently prioritizes immediately releasing torque from all motors, resulting in the arm dropping dramatically if in a lifted position **WARNING**

---

## Architecture & Design

### Design Goals

* **Low-Latency Teleoperation:** Achieves high-frequency control loops (`dt = 0.002s (500Hz)`) to ensure fluid mapping between human VR movements and robot kinematics.
* **Safety & Stability:** Implements weighted moving filters to smooth raw IK outputs and strict motor damping on unexpected crashes or keyboard interrupts.
* **Optimization-Based IK:** Prioritizes end-effector translation over rotation, while naturally regularizing posture to a neutral state to prevent joint singularities using CasADi.

### Code Structure

The system is built around a single ROS2 Node (`G1VRController`) that manages asynchronous ROS subscriptions while running a synchronous high-speed control loop.

### Kinematics Pipeline (Pinocchio + CasADi)

* **Reduced Model Generation:** To optimize calculation times, the 29-DOF URDF is dynamically reduced.
* **Joint Locking:** The legs and waist (15 joints total) are locked to a reference configuration, isolating the computational load entirely to the upper body and arms.
* **Optimization Problem Formulation:** The IK solver is treated as a nonlinear programming (NLP) problem via CasADi's `Opti` interface.
* **Cost Function Weights:** Translation Error (100.0), Rotation Error (10.0), Regularization (0.02), Smoothness (2.0), and Posture deviation (0.1).

### Communication & Control Loop

* **Unitree SDK Interface:** Bypasses high-level ROS commands in favor of Unitree's `ChannelPublisher` and `ChannelSubscriber` for direct `rt/lowcmd` and `rt/lowstate` access.
* **Mode Switching:** Automatically negotiates control with the robot's state machine via `MotionSwitcherClient`, ensuring the default controller releases the motors before the script takes over.
* **Execution Loop:** Reads current joint states, feeds the target VR pose into the IK solver, smooths the output, calculates required velocities, and dispatches commands over the network via CRC-verified packets.

### Hand & Gripper Control

* **Dex3 Integration:** The left and right Dex3 hands are controlled independently via `rt/dex3/left/cmd` and `rt/dex3/right/cmd`.
* **Binary Actuation:** Triggered by a boolean VR input (`/pos_rot_left_status`).
* **Joint Mapping:** Maps the single boolean directly to predefined kinematic arrays to either fully curl or fully extend the 7 joints of the Dex3 hand.