import time
import copy
import numpy as np
import threading
import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
import pinocchio as pin
from enum import Enum

# Unitree SDK
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_, unitree_hg_msg_dds__HandCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_ as hg_LowCmd, LowState_ as hg_LowState, HandCmd_ as hg_HandCmd, HandState_ as hg_HandState
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

# Local imports
from g1_utils import JOINT_MAP, G1Motors, WeightedMovingFilter
from ik_solver import DualArmIKSolver

class RobotState(Enum):
    RUNNING = 1
    RETURNING_HOME = 2
    DONE = 3

class G1HandTestController(Node):
    def __init__(self):
        super().__init__('g1_hand_test_controller')

        # Dynamically find URDF in ROS 2 package (Mimicking original setup)
        urdf_path = "./g1_29dof.urdf" 
        self.dt = 0.002
        self.ik_solver = DualArmIKSolver(urdf_path, self.dt)
        self.smooth_filter = WeightedMovingFilter(weights=[1.0], window_size=5)

        # Arm state variables (holding constant poses to safely test the hands)
        self.target_pos_l = np.array([0.3, 0.2, 0.23])
        self.target_rot_l = np.eye(3)
        self.target_pos_r = np.array([0.3, -0.2, 0.23])
        self.target_rot_r = np.eye(3)
        
        self.hand_state = {"left": True, "right": True}
        self.current_hand_state = {"left": True, "right": True}

        self.finger_pos = {"left": [0.0]*7, "right": [0.0]*7}
        self.torque_cur = {"left": [0.0]*7, "right": [0.0]*7}
        self.torque_prev = {"left": [0.0]*7, "right": [0.0]*7}
        self.overTorque = {"left": False, "right": False}
        self.overTorque_start_time = {"left": None, "right": None}
        self.torque_limit = 5e5

        # Threading and Unitree States
        self.motor_data_lock = threading.Lock()
        self.motor_data_buffer = None
        self.mode_machine = None
        self.first_read = True
        self.initialized = False
        self.numIterationsToHome = 0
        self.state = RobotState.RUNNING

        self._setup_unitree()
        
        # Start the keyboard listener thread for manual overrides
        self.kb_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        self.kb_thread.start()
        
        # Start control loop
        self.timer = self.create_timer(self.dt, self.controlLoop)

    def keyboard_listener(self):
        """Listen for 'l' or 'r' keypresses to toggle hand states without blocking."""
        print("\n=== Dex3 Hand Control Interface ===")
        print(" [L] Toggle Left Hand")
        print(" [R] Toggle Right Hand")
        print(" [Q] Quit")
        print("===================================\n")
        
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while rclpy.ok():
                # Check for input every 0.1s
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1).lower()
                    if key == 'l':
                        self.hand_state['left'] = not self.hand_state['left']
                        print(f"\r---> LEFT hand set to: {'OPEN' if self.hand_state['left'] else 'CLOSE'}")
                    elif key == 'r':
                        self.hand_state['right'] = not self.hand_state['right']
                        print(f"\r---> RIGHT hand set to: {'OPEN' if self.hand_state['right'] else 'CLOSE'}")
                    elif key == 'q':
                        if self.state == RobotState.RUNNING:
                            self.get_logger().info("RETURNING HOME")
                            self.state = RobotState.RETURNING_HOME
                            self.numIterationsToHome = 0
                        break
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def _setup_unitree(self):
        print("Changing unitree mode...")
        self.msc = MotionSwitcherClient()
        self.msc.SetTimeout(5.0)
        self.msc.Init()
        status, result = self.msc.CheckMode()
        while result['name']:
            self.msc.ReleaseMode()
            _, result = self.msc.CheckMode()
            time.sleep(1)
            
        self.crc = CRC()
        self.cmd = unitree_hg_msg_dds__LowCmd_()
        self.cmd.mode_pr = 0 
        self.cmd.mode_machine = 0

        self.pub = ChannelPublisher("rt/lowcmd", hg_LowCmd)
        self.pub.Init()
        self.sub = ChannelSubscriber("rt/lowstate", hg_LowState)
        self.sub.Init(self.subscribeMotorData, 10)

        self.left_hand_pub = ChannelPublisher("rt/dex3/left/cmd", hg_HandCmd)
        self.left_hand_pub.Init()
        self.left_hand_cmd = unitree_hg_msg_dds__HandCmd_()

        self.right_hand_pub = ChannelPublisher("rt/dex3/right/cmd", hg_HandCmd)
        self.right_hand_pub.Init()
        self.right_hand_cmd = unitree_hg_msg_dds__HandCmd_()
        
        self.left_hand_sub = ChannelSubscriber("rt/dex3/left/state", hg_HandState)
        self.left_hand_sub.Init(lambda msg: self.torque_callback(msg, "left"), 10)

        self.right_hand_sub = ChannelSubscriber("rt/dex3/right/state", hg_HandState)
        self.right_hand_sub.Init(lambda msg: self.torque_callback(msg, "right"), 10)

    def subscribeMotorData(self, msg: hg_LowState):
        if msg is not None:
            lowstate = G1Motors()
            for id in range(35):
                lowstate.motor_data[id].q = msg.motor_state[id].q
                lowstate.motor_data[id].dq = msg.motor_state[id].dq
            with self.motor_data_lock:
                self.motor_data_buffer = lowstate
                self.mode_machine = msg.mode_machine
            self.first_read = False

    def torque_callback(self, msg: hg_HandState, side: str):
        if msg is None or not msg.motor_state:
            return

        self.torque_cur[side] = [round(motor.tau_est, 2) for motor in msg.motor_state[:7]]
        self.finger_pos[side] = [motor.q for motor in msg.motor_state[:7]]

    def control_hand(self):

        for side in ["left", "right"]:
            cmd = self.left_hand_cmd if side == "left" else self.right_hand_cmd
            pub = self.left_hand_pub if side == "left" else self.right_hand_pub

            publish_required = False

            # Over-Torque Check
            is_exceeding = any(abs(t) > self.torque_limit for t in self.torque_cur[side])

            if is_exceeding and not self.hand_state[side]: #if its not opening/open 
                # Start the timer if it hasn't been started yet
                if self.overTorque_start_time[side] is None:
                    self.overTorque_start_time[side] = time.time()
                
                # Only trigger hold logic if 1 second has elapsed
                elif time.time() - self.overTorque_start_time[side] >= 0.1:
                    if not self.overTorque[side]:
                        print(f"\n[{side.upper()}] Over-Torque Detected for >1s! Holding position.")
                        self.overTorque[side] = True
                    
                    target_pos = self.finger_pos[side]
                    for i in range(7):
                        cmd.motor_cmd[i].mode = 1
                        cmd.motor_cmd[i].q = target_pos[i]
                        cmd.motor_cmd[i].dq = 0.0
                        cmd.motor_cmd[i].kp = 10.0
                        cmd.motor_cmd[i].kd = 1.0
                        cmd.motor_cmd[i].tau = 0.0
                    publish_required = True
            else:
                # Reset the timer if torque drops below limit or user commands hand to open
                self.overTorque_start_time[side] = None

            # 2. State Change Check (User Input)
            if self.hand_state[side] != self.current_hand_state[side]:
                to_open = self.hand_state[side]
                self.current_hand_state[side] = to_open
                
                # Reset over-torque ONLY if the user is commanding the hand to OPEN
                if to_open and self.overTorque[side]:
                    self.overTorque[side] = False

                # Only command the new position if we aren't currently over-torqued
                if not self.overTorque[side]:
                    target_pos = [0.0] * 7 # Default Open
                    if not to_open: # Close
                        target_pos = [0, 0, 1.75, -1.57, -1.75, -1.57, -1.75] if side == "left" else [0, 0, -1.75, 1.57, 1.57, 1.57, 1.57]
                    
                    for i in range(7):
                        cmd.motor_cmd[i].mode = 1
                        cmd.motor_cmd[i].q = target_pos[i]
                        cmd.motor_cmd[i].dq = 0.0
                        cmd.motor_cmd[i].kp = 20.0
                        cmd.motor_cmd[i].kd = 1.0
                        cmd.motor_cmd[i].tau = 0.0
                    
                    self.get_logger().info(f"Dex3 {side} hand commanded to {'OPEN' if to_open else 'CLOSE'}")
                    publish_required = True

            # 3. Publish if any changes were made
            if publish_required:
                pub.Write(cmd)
    


    def log_and_print_torques(self):
        """Records torques to a CSV and throttles console output for readability."""
        # 1. Initialize file and print counter on the first run
        if not hasattr(self, 'torque_file'):
            filename = f"hand_torques_{int(time.time())}.csv"
            self.torque_file = open(filename, "w")
            self.torque_file.write("time,L0,L1,L2,L3,L4,L5,L6,R0,R1,R2,R3,R4,R5,R6\n")
            self.print_counter = 0

        current_time = time.time()
        l_tau = self.torque_cur["left"]
        r_tau = self.torque_cur["right"]
        
        # 2. Record to file (Runs at loop frequency)
        row = [current_time] + l_tau + r_tau
        self.torque_file.write(",".join(map(str, row)) + "\n")
        
        # 3. Print to console (Throttled to ~5Hz to prevent lag)
        #self.print_counter += 1
        #if self.print_counter >= 100:  # 100 loops * 0.002s = 0.2 seconds
        #    print(f"Torques | L: {[round(t, 2) for t in l_tau]} | R: {[round(t, 2) for t in r_tau]}")
        #    self.print_counter = 0


    def controlLoop(self):
        if self.first_read:
            self.get_logger().info("WAITING TO SUBSCRIBE HAND DATA")
            return
            
        if not self.initialized:
            with self.motor_data_lock:
                current_q_sensor = pin.neutral(self.ik_solver.model)
                for name, motorId in JOINT_MAP.items():
                    if self.ik_solver.model.existJointName(name):
                        idx_q = self.ik_solver.model.joints[self.ik_solver.model.getJointId(name)].idx_q
                        if idx_q != -1:
                            current_q_sensor[idx_q] = self.motor_data_buffer.motor_data[motorId].q
                self.last_solved_q = current_q_sensor
            self.smooth_filter.add_data(self.last_solved_q)
            self.initialized = True
            return

        try:
            self.control_hand()
            if self.state == RobotState.RETURNING_HOME:
                self.get_logger().info("shutting down")
                #tell the arm to return home
                self.target_pos_l = np.array([0.1, 0.25, -0.3])
                self.target_rot_l = np.eye(3)
                self.target_pos_r = np.array([0.2, -0.2, -0.25])
                self.target_rot_r = np.array([
                    [1.0,  0.0,  0.0],
                    [0.0, np.cos(-np.pi/6.0),  -np.sin(-np.pi/6.0)],
                    [0.0,  np.sin(-np.pi/6.0), np.cos(-np.pi/6.0)]
                ])
                self.numIterationsToHome = self.numIterationsToHome + 1
            #self.log_and_print_torques()
            # Solve IK for both arms
            raw_q, v_Cmd, success = self.ik_solver.solve(
                self.target_pos_l, self.target_rot_l, 
                self.target_pos_r, self.target_rot_r, 
                self.last_solved_q
            )
            
            self.smooth_filter.add_data(raw_q)
            qCmd = self.smooth_filter.filtered_data
            self.last_solved_q = qCmd
            
            # Send commands
            for name, motorId in JOINT_MAP.items():
                if self.ik_solver.model.existJointName(name):
                    idx_q = self.ik_solver.model.joints[self.ik_solver.model.getJointId(name)].idx_q
                    if idx_q != -1:
                        self.cmd.motor_cmd[motorId].mode = 1
                        self.cmd.motor_cmd[motorId].q = qCmd[idx_q]
                        self.cmd.motor_cmd[motorId].kp = 40.0  
                        self.cmd.motor_cmd[motorId].kd = 1   

            self.cmd.mode_pr = 1
            with self.motor_data_lock:
                self.cmd.mode_machine = self.mode_machine
            
            self.cmd.crc = self.crc.Crc(self.cmd)
            self.pub.Write(self.cmd)

            if self.state == RobotState.RETURNING_HOME and self.numIterationsToHome > 500:
                error = np.linalg.norm(qCmd - self.last_solved_q)
                if error < 0.005:
                    self.get_logger().info("Reached home, shutting down")
                    self.state = RobotState.DONE
                    for i in range(35):
                        self.cmd.motor_cmd[i].mode = 0x01 
                        self.cmd.motor_cmd[i].q = 0.0
                        self.cmd.motor_cmd[i].kp = 0.0
                        self.cmd.motor_cmd[i].dq = 0.0
                        self.cmd.motor_cmd[i].tau = 0.0
                        self.cmd.motor_cmd[i].kd = 1.0    
                    self.cmd.crc = self.crc.Crc(self.cmd)
                    self.pub.Write(self.cmd)
                    if rclpy.ok():
                        rclpy.shutdown()
        except (KeyboardInterrupt, RuntimeError) as e:
            print(f"\n[Interrupt/Error] Exiting cleanly... {e}")
            for i in range(35):
                self.cmd.motor_cmd[i].mode = 0x01 
                self.cmd.motor_cmd[i].q = 0.0
                self.cmd.motor_cmd[i].kp = 0.0
                self.cmd.motor_cmd[i].dq = 0.0
                self.cmd.motor_cmd[i].tau = 0.0
                self.cmd.motor_cmd[i].kd = 1.0    
            self.cmd.crc = self.crc.Crc(self.cmd)
            self.pub.Write(self.cmd)

def main(args=None):
    ChannelFactoryInitialize(0) 
    rclpy.init(args=args)
    controller = G1HandTestController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()