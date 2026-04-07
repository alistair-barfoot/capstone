# Unitree G1 VR Teleoperation Controller

This repository contains a ROS2-based control system for real-time teleoperation of the Unitree G1 humanoid robot. It acts as the bridge between VR tracking data (e.g., from a Meta Quest Pro) and the robot's low-level hardware, utilizing CasADi and Pinocchio for rapid Inverse Kinematics (IK) solving and the Unitree SDK2 for direct motor control.

> **Note on ROS Networking:** Documentation regarding the ROS2 DDS configuration, network setup, and how VR pose topics (`/pos_rot_left`, `/pos_rot_right`) are published across the network can be found in the networking section of this GitHub repository.

## Prerequisites

Before running the controller, ensure the following dependencies are installed:
* **ROS2** (Tested with Humble and Foxy)
* **Unitree SDK2** (`unitree_sdk2py`)
* **Pinocchio & CasADi** (Must be linked together correctly; most easily done using the OpenRobots version)
* **Python 3.10+**
* The `g1_29dof.urdf` file must be located in the same directory as the execution scripts.

## Environment Setup

A dedicated shell script (`controlSetup.sh`) is provided to streamline the initialization of the workspace.

1. Ensure you have a Python virtual environment named `armControl` initialized in your workspace. If not, create one:
   ```bash
   python3 -m venv armControl
   ```
2. Verify that your Unitree ROS2 workspace is located at `~/unitree_ros2`.
3. Source the environment setup script to configure all necessary paths for Pinocchio, CasADi, and ROS2:
   ```bash
   source controlSetup.sh
   ```

## Running the Code

1. Ensure the robot is powered on and connected to the computer's ethernet port (ideally through a router, but can be configured directly from humanoid to the computer).
2. Launch VR tracking on the headset that publishes to the `/pos_rot_left`, `/pos_rot_right`, and their respective `_status` topics.
3. Execute the main Python controller:
   ```bash
   python arm_control_node.py
   ```
4. **Shutdown & Safety Warnings:**
   * **Safe Shutdown (Recommended):** Press `Q` in the terminal. The robot will enter a `RETURNING_HOME` state, override VR inputs, and smoothly fold its arms into a safe, tucked position over 500 iterations before turning off the motors.
   * **Emergency Drop (WARNING):** Pressing `Ctrl+C` catches a `KeyboardInterrupt` that immediately zeroes out all motor torques and PD gains. **If the arms are in a lifted position, they will drop dramatically.**

## Architecture & Design

### Design Goals
* **Low-Latency Teleoperation:** Achieves high-frequency control loops (`dt = 0.002s` or 500Hz) to ensure fluid mapping between human VR movements and robot kinematics. The CasADi solver completes its optimization in roughly **10ms** on average.
* **Safety & Stability:** Implements weighted moving filters to smooth raw IK outputs, self-collision penalties, and active over-torque protection.

### Kinematics Pipeline (Pinocchio + CasADi)
* **Dynamic Model Reduction:** To optimize calculation times, the 29-DOF URDF is dynamically reduced (`buildReducedRobot`). The legs and waist (15 joints total) are locked to a reference configuration, isolating the computational load entirely to the upper body and arms.
* **Optimization Problem Formulation:** The IK solver is treated as a nonlinear programming (NLP) problem via CasADi's `ipopt` interface. It uses warm-starting (`warm_start_init_point: 'yes'`) to ensure rapid convergence between consecutive frames.
* **Cost Function Weights:** * Translation Error: `100.0`
  * Rotation Error: `10.0`
  * Velocity/Smoothness: `2.0`
  * Posture Regularization: `0.05`
  * Joint Regularization: `0.02`
  * Elbow Self-Collision: `10000.0`

### Communication & Control Loop
* **Unitree SDK Interface:** Bypasses high-level ROS commands in favor of Unitree's `ChannelPublisher` and `ChannelSubscriber` for direct `rt/lowcmd` and `rt/lowstate` access.
* **Mode Switching:** Automatically negotiates control with the robot's state machine via `MotionSwitcherClient`, ensuring the default controller releases the motors before the script takes over.

## Built-in Safety Features

Operating humanoid hardware requires strict safety boundaries. This pipeline implements safety at multiple levels:

1. **Active Over-Torque Protection (Dex3 Hands):** The nodes continuously monitor the estimated torque (`tau_est`) of all 14 hand motors. If any motor exceeds the defined safety limit (`1e6`) for more than 0.1 seconds while gripping, the system triggers an `overtorque` fault. It immediately zeroes the target velocity and switches the hand to a position-hold state to prevent motor burnout.
2. **Elbow Self-Collision Avoidance:** The IK solver tracks the X/Y coordinates of both elbows in real-time. If either elbow breaches a 16cm safety cylinder around the robot's Z-axis, the solver incurs a massive cost penalty, physically preventing the optimizer from crashing the arms into the torso.
3. **Automated Soft-Homing Sequence:** As detailed in the setup, pressing `Q` invokes a graceful degradation path rather than dropping the robot's arms abruptly.

## File Structure

* **`arm_control_node.py`**: The primary ROS2 node (`G1VRController`). Subscribes to Cartesian pose topics, communicates with the Unitree SDK2, handles the state machine, and implements over-torque protection.
* **`ik_solver.py`**: The CasADi and Pinocchio kinematics engine. Calculates joint angles by minimizing translation/rotation errors while enforcing joint limits, smoothing, and collision avoidance.
* **`hand_test_node.py`**: A standalone diagnostic ROS2 node. Locks the robot's arms into a static pose and provides a terminal UI (`L`/`R`) to manually actuate the grips and tune torque limits safely.
* **`g1_utils.py`**: Core utilities including the `JOINT_MAP` (mapping URDF names to Unitree motor IDs) and the `WeightedMovingFilter` used to temporally smooth the IK solver's output.
* **`controlSetup.sh`**: The environment configuration script that sources Python virtual environments, ROS2, and OpenRobots libraries.