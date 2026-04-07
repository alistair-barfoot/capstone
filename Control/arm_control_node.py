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
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import pinocchio as pin
from ament_index_python.packages import get_package_share_directory
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

class G1VRController(Node):
    def __init__(self):
        super().__init__('g1_vr_controller')

        self.state = RobotState.RUNNING
        urdf_path = "./g1_29dof.urdf" #f"{get_package_share_directory('g1_vr_control')}/urdf/g1_29dof.urdf"
        self.dt = 0.002
        self.ik_solver = DualArmIKSolver(urdf_path, self.dt)
        self.smooth_filter = WeightedMovingFilter(weights=[1.0], window_size=5)

        # Arm state variables
        self.target_pos_l = np.array([0.3, 0.3, 0.23])
        self.target_rot_l = np.eye(3)
        self.target_pos_r = np.array([0.3, -0.3, 0.23])
        self.target_rot_r = np.eye(3)
        
        self.hand_state = {"left": True, "right": True}
        self.current_hand_state = {"left": False, "right": False}

        self.finger_pos = {"left": [0.0]*7, "right": [0.0]*7}
        self.torque_cur = {"left": [0.0]*7, "right": [0.0]*7}
        self.overtorque_start_time = {"left": None, "right": None}
        self.overtorque = {"left": False, "right": False}
        self.torque_limit = 1e6
        # ROS2 Subscribers
        self.create_subscription(PoseStamped, '/pos_rot_left', lambda msg: self.arm_callback(msg, True), 10)
        self.create_subscription(Bool, '/pos_rot_left_status', lambda msg: self.grip_callback(msg, "left"), 10)
        self.create_subscription(PoseStamped, '/pos_rot_right', lambda msg: self.arm_callback(msg, False), 10)
        self.create_subscription(Bool, '/pos_rot_right_status', lambda msg: self.grip_callback(msg, "right"), 10)

        # Threading and Unitree States
        self.motor_data_lock = threading.Lock()
        self.motor_data_buffer = None
        self.mode_machine = None
        self.first_read = True
        self.initialized = False
        self.quit = False
        self.num_iter_to_home = 0

        self._setup_unitree()

        # Start the keyboard listener thread for manual overrides
        self.kb_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        self.kb_thread.start()

        self.timer = self.create_timer(self.dt, self.controlLoop)

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

    def torque_callback(self, msg: hg_HandState, side: str):
        if msg is None or not msg.motor_state:
            return

        self.torque_cur[side] = [round(motor.tau_est, 2) for motor in msg.motor_state[:7]]
        self.finger_pos[side] = [motor.q for motor in msg.motor_state[:7]]

    def arm_callback(self, msg, left):
        pos = np.array([msg.pose.position.y, msg.pose.position.x * -1.0, (msg.pose.position.z * -1.0) + 0.55])
        
        
        if left:
            quat = pin.Quaternion(msg.pose.orientation.w, msg.pose.orientation.y, msg.pose.orientation.z * -1.0, msg.pose.orientation.x)
            self.target_pos_l, self.target_rot_l = pos, quat.matrix()
        else:
            quat = pin.Quaternion(msg.pose.orientation.w, msg.pose.orientation.y, msg.pose.orientation.x * -1.0, msg.pose.orientation.z)
            self.target_pos_r, self.target_rot_r = pos, (quat.matrix() @ np.array([[1,0,0],[0,-1,0],[0,0,-1]]))

    def grip_callback(self, msg, side):
        self.hand_state[side] = msg.data

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

    def keyboard_listener(self):
        print("\n=== Keyboard Commands ===")
        print(" [Q] Go to home position and stop control")
        print("===================================\n")
        
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while rclpy.ok():
                # Check for input every 0.1s
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1).lower()
                    if key == 'q':
                        if self.state == RobotState.RUNNING:
                            self.get_logger().info("RETURNING HOME")
                            self.state = RobotState.RETURNING_HOME
                            self.num_iter_to_home = 0
                        break
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def control_hand(self):

        for side in ["left", "right"]:
            cmd = self.left_hand_cmd if side == "left" else self.right_hand_cmd
            pub = self.left_hand_pub if side == "left" else self.right_hand_pub

            publish_required = False

            # Over-Torque Check
            is_exceeding = any(abs(t) > self.torque_limit for t in self.torque_cur[side])

            if is_exceeding and not self.hand_state[side]:
                # Start the timer if it hasn't been started yet
                if self.overtorque_start_time[side] is None:
                    self.overtorque_start_time[side] = time.time()
                
                # Only trigger hold logic if 0.1 second has elapsed
                elif time.time() - self.overtorque_start_time[side] >= 0.1:
                    if not self.overtorque[side]:
                        print(f"\n[{side.upper()}] Over-Torque Detected for >0.1s! Holding position.")
                        self.overtorque[side] = True
                    
                    target_pos = self.finger_pos[side]
                    for i in range(7):
                        cmd.motor_cmd[i].mode = 1
                        cmd.motor_cmd[i].q = target_pos[i]
                        cmd.motor_cmd[i].dq = 0.0
                        cmd.motor_cmd[i].kp = 15.0
                        cmd.motor_cmd[i].kd = 1.0
                        cmd.motor_cmd[i].tau = 0.0
                    publish_required = True
            else:
                # Reset the timer if torque drops below limit or user commands hand to open
                self.overtorque_start_time[side] = None

            #State Change Check (User Input)
            if self.hand_state[side] != self.current_hand_state[side]:
                to_open = self.hand_state[side]
                self.current_hand_state[side] = to_open
                
                # Reset over-torque only if the user is commanding the hand to OPEN
                if to_open and self.overtorque[side]:
                    self.overtorque[side] = False

                # Only command the new position if we aren't currently over-torqued
                if not self.overtorque[side]:
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

            # Publish if any changes were made
            if publish_required:
                pub.Write(cmd)
    
        
    def controlLoop(self):
        #startTime = time.perf_counter()
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
                self.target_pos_l = np.array([0.1, 0.2, -0.3])
                self.target_rot_l = np.eye(3)
                self.target_pos_r = np.array([0.2, -0.2, -0.25])
                self.target_rot_r = np.array([
                    [1.0,  0.0,  0.0],
                    [0.0, np.cos(-np.pi/6.0),  -np.sin(-np.pi/6.0)],
                    [0.0,  np.sin(-np.pi/6.0), np.cos(-np.pi/6.0)]
                ])
                self.num_iter_to_home = self.num_iter_to_home + 1
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
            if self.state == RobotState.RETURNING_HOME and self.num_iter_to_home > 500:
                self.get_logger().info("Reached home, shut down SUCCESSFUL")
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
        #endTime = time.perf_counter()
        #latency_ms = (endTime - startTime) * 1000
        #print(f"Component latency: {latency_ms:.2f} ms")


def main(args=None):
    ChannelFactoryInitialize(0) 
    rclpy.init(args=args)
    controller = G1VRController()
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