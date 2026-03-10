import time
import copy
import numpy as np
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import pinocchio as pin
from ament_index_python.packages import get_package_share_directory

# Unitree SDK
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_, unitree_hg_msg_dds__HandCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_ as hg_LowCmd, LowState_ as hg_LowState, HandCmd_ as hg_HandCmd
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

# Local imports
from .g1_utils import JOINT_MAP, G1Motors, WeightedMovingFilter
from .ik_solver import DualArmIKSolver

class G1VRController(Node):
    def __init__(self):
        super().__init__('g1_vr_controller')

        # Dynamically find URDF in ROS 2 package
        urdf_path = f"{get_package_share_directory('g1_vr_control')}/urdf/g1_29dof.urdf"
        self.dt = 0.002
        self.ik_solver = DualArmIKSolver(urdf_path, self.dt)
        self.smoothFilter = WeightedMovingFilter(weights=[1.0], window_size=5)

        # Arm state variables
        self.target_pos_l = np.array([0.3, 0.3, 0.23])
        self.target_rot_l = np.eye(3)
        self.target_pos_r = np.array([0.3, -0.3, 0.23])
        self.target_rot_r = np.eye(3)
        
        self.hand_state = {"left": True, "right": True}
        self.current_hand_state = {"left": False, "right": False}

        # Subscribers
        self.create_subscription(PoseStamped, '/pos_rot_left', lambda msg: self.arm_callback(msg, true), 10)
        self.create_subscription(Bool, '/pos_rot_left_status', lambda msg: self.grip_callback(msg, true), 10)
        self.create_subscription(PoseStamped, '/pos_rot_right', lambda msg: self.arm_callback(msg, false), 10)
        self.create_subscription(Bool, '/pos_rot_right_status', lambda msg: self.grip_callback(msg, false), 10)

        # Threading and Unitree States
        self.motorDataLock = threading.Lock()
        self.motorDataBuffer = None
        self.modeMachine = None
        self.firstRead = True
        self.initialized = False
        
        self._setup_unitree()
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

    def arm_callback(self, msg, left):
        pos = np.array([msg.pose.position.y, msg.pose.position.x * -1.0, (msg.pose.position.z * -1.0) + 0.24])
        quat = pin.Quaternion(msg.pose.orientation.w, msg.pose.orientation.y, msg.pose.orientation.x, msg.pose.orientation.z)
        
        if left:
            self.target_pos_l, self.target_rot_l = pos, quat.matrix()
        else:
            self.target_pos_r, self.target_rot_r = pos, quat.matrix()

    def grip_callback(self, msg, side):
        self.hand_state[side] = msg.data

    def subscribeMotorData(self, msg: hg_LowState):
        if msg is not None:
            lowstate = G1Motors()
            for id in range(35):
                lowstate.motorData[id].q = msg.motor_state[id].q
                lowstate.motorData[id].dq = msg.motor_state[id].dq
            with self.motorDataLock:
                self.motorDataBuffer = lowstate
                self.modeMachine = msg.mode_machine
            self.firstRead = False

    def control_hand(self, side, is_open):
        cmd = self.left_hand_cmd if side == "left" else self.right_hand_cmd
        pub = self.left_hand_pub if side == "left" else self.right_hand_pub
        
        if is_open:
            target_pos = [0, 0, 0, 0, 0, 0, 0]
        else:
            target_pos = [0, 0, 1.75, -1.57, -1.75, -1.57, -1.75] if side == "left" else [0, 0, -1.75, 1.57, 1.57, 1.57, 1.57]
            
        for i in range(7):
            cmd.motor_cmd[i].mode = 1
            cmd.motor_cmd[i].q = target_pos[i]
            cmd.motor_cmd[i].dq = 0.0
            cmd.motor_cmd[i].kp = 20.0
            cmd.motor_cmd[i].kd = 1.0
            cmd.motor_cmd[i].tau = 0.0
            
        pub.Write(cmd)
        self.get_logger().info(f"Dex3 {side} hand commanded to {'OPEN' if is_open else 'CLOSE'}")

    def controlLoop(self):
        if self.firstRead:
            return
            
        if not self.initialized:
            with self.motorDataLock:
                currentQSensor = pin.neutral(self.ik_solver.model)
                for name, motorId in JOINT_MAP.items():
                    if self.ik_solver.model.existJointName(name):
                        idx_q = self.ik_solver.model.joints[self.ik_solver.model.getJointId(name)].idx_q
                        if idx_q != -1:
                            currentQSensor[idx_q] = self.motorDataBuffer.motorData[motorId].q
                self.lastSolvedQ = currentQSensor
            self.smoothFilter.add_data(self.lastSolvedQ)
            self.initialized = True
            return

        try:
            # Handle grippers dynamically for both sides
            for side in ["left", "right"]:
                if self.hand_state[side] != self.current_hand_state[side]:
                    self.control_hand(side, self.hand_state[side])
                    self.current_hand_state[side] = self.hand_state[side]

            # Solve IK for both arms
            raw_q, vCmd, success = self.ik_solver.solve(
                self.target_pos_l, self.target_rot_l, 
                self.target_pos_r, self.target_rot_r, 
                self.lastSolvedQ
            )
            
            self.smoothFilter.add_data(raw_q)
            qCmd = self.smoothFilter.filtered_data
            self.lastSolvedQ = qCmd

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
            with self.motorDataLock:
                self.cmd.mode_machine = self.modeMachine
            
            self.cmd.crc = self.crc.Crc(self.cmd)
            self.pub.Write(self.cmd)
    
        except (KeyboardInterrupt, RuntimeError) as e:
            print(f"\n[Interrupt/Error] Exiting cleanly... {e}")
            for i in range(35):
                self.cmd.motor_cmd[i].mode = 0x01 
                self.cmd.motor_cmd[i].q = 0.0
                self.cmd.motor_cmd[i].kp, self.cmd.motor_cmd[i].dq, self.cmd.motor_cmd[i].tau = 0.0, 0.0, 0.0
                self.cmd.motor_cmd[i].kd = 1.0    
            self.cmd.crc = self.crc.Crc(self.cmd)
            self.pub.Write(self.cmd)

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