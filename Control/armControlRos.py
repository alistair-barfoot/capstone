import time
import sys
import numpy as np
import pinocchio as pin
import threading
import rclpy
import casadi
import copy
from pinocchio import casadi as cpin
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.default import (unitree_hg_msg_dds__LowCmd_,unitree_hg_msg_dds__HandCmd_)
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import (LowCmd_ as hg_LowCmd, LowState_ as hg_LowState, HandCmd_ as hg_HandCmd)
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

URDF_PATH = "./g1_29dof.urdf" 
endEffectorName = "left_end_effector" 
lowStateTopic = "rt/lowstate"
simMode = False
JOINT_MAP = {
    "right_shoulder_pitch_joint": 22,
    "right_shoulder_roll_joint":  23,
    "right_shoulder_yaw_joint":   24,
    "right_elbow_joint":          25, 
    "right_wrist_roll_joint":     26,
    "right_wrist_pitch_joint":    27,
    "right_wrist_yaw_joint":      28,

    "left_shoulder_pitch_joint":  15,
    "left_shoulder_roll_joint":   16,
    "left_shoulder_yaw_joint":    17,
    "left_elbow_joint":           18,
    "left_wrist_roll_joint":      19,
    "left_wrist_pitch_joint":     20,
    "left_wrist_yaw_joint":       21,
}

class MotorData:
    def __init__(self):
        self.q = 0.0
        self.dq = 0.0

class G1Motors:
    def __init__(self):
        self.motorData = [MotorData() for _ in range (35)] 

class WeightedMovingFilter:
    def __init__(self, weights, window_size):
        self.weights = np.array(weights)
        self.window_size = window_size
        self.data_buffer = []

    def add_data(self, new_data):
        self.data_buffer.append(new_data)
        if len(self.data_buffer) > self.window_size:
            self.data_buffer.pop(0)

    @property
    def filtered_data(self):
        if not self.data_buffer:
            return None
        return np.mean(self.data_buffer, axis=0)
    

class G1VRController(Node):
    def __init__(self):
        super().__init__('g1_vr_controller')

        self.leftHandSub = self.create_subscription(
            PoseStamped,
            '/pos_rot_left',  
            self.leftHandCallback,
            10)
        
        self.leftHandGripSub = self.create_subscription(
            Bool,
            '/pos_rot_left_status',  
            self.leftHandGripCallback,
            10)
        self.get_logger().info("Waiting for VR hand data...")
        
        
        print("changing unitree mode")
        #Comment out till initializing start location when using simulation
        self.msc = MotionSwitcherClient()
        self.msc.SetTimeout(5.0)
        self.msc.Init()
        status, result = self.msc.CheckMode()
        print(f"Status is: {status}")
        print(result)
        while result['name']:
            self.msc.ReleaseMode()
            status, result = self.msc.CheckMode()
            time.sleep(1)
        print("Successfully switched control mode")
        
        self.motorDataBuffer = None
        self.motorDataLock = threading.Lock()
        self.modeMachine = None
        self.firstRead = True
        self.running = True
        self.initialized = False
        self.handOnly = False
        self.numIter = 1
        self.leftHandOpen = True
        self.leftHandCurrentOpen = False
        self.cmd = unitree_hg_msg_dds__LowCmd_()
        self.cmd.mode_pr = 0 
        self.cmd.mode_machine = 0
        self.crc = CRC()
        
        self.pub = ChannelPublisher("rt/lowcmd", hg_LowCmd)
        self.pub.Init()

        self.sub = ChannelSubscriber(lowStateTopic, hg_LowState)
        self.sub.Init(self.subscribeMotorData, 10)
        
        # Initialize Left Dex3 Hand
        self.left_hand_pub = ChannelPublisher("rt/dex3/left/cmd", hg_HandCmd)
        self.left_hand_pub.Init()
        self.left_hand_cmd = unitree_hg_msg_dds__HandCmd_()

        # Initialize Right Dex3 Hand
        self.right_hand_pub = ChannelPublisher("rt/dex3/right/cmd", hg_HandCmd)
        self.right_hand_pub.Init()
        self.right_hand_cmd = unitree_hg_msg_dds__HandCmd_()


        try:
            self.robot = pin.RobotWrapper.BuildFromURDF(URDF_PATH,'.')
        except Exception as e:
            print(f"Error loading URDF: {e}")
            sys.exit(1)

        self.mixed_jointsToLockIDs = [
                                        "left_hip_pitch_joint" ,
                                        "left_hip_roll_joint" ,
                                        "left_hip_yaw_joint" ,
                                        "left_knee_joint" ,
                                        "left_ankle_pitch_joint" ,
                                        "left_ankle_roll_joint" ,
                                        "right_hip_pitch_joint" ,
                                        "right_hip_roll_joint" ,
                                        "right_hip_yaw_joint" ,
                                        "right_knee_joint" ,
                                        "right_ankle_pitch_joint" ,
                                        "right_ankle_roll_joint" ,
                                        "waist_yaw_joint" ,
                                        "waist_roll_joint" ,
                                        "waist_pitch_joint" 
                                    ]
        
        
        self.reducedRobot = self.robot.buildReducedRobot(
            list_of_joints_to_lock=self.mixed_jointsToLockIDs,
            reference_configuration=np.array([0.0] * self.robot.model.nq),
        )      
        self.reducedModel = self.reducedRobot.model  
        self.reducedModel.addFrame(
            pin.Frame('left_end_effector',
                      self.reducedModel.getJointId('left_wrist_yaw_joint'),
                      pin.SE3(np.eye(3),
                              np.array([0.05,0,0]).T),
                      pin.FrameType.OP_FRAME)
        )
        
        self.reducedModel.addFrame(
            pin.Frame('right_end_effector',
                      self.reducedModel.getJointId('right_wrist_yaw_joint'),
                      pin.SE3(np.eye(3),
                              np.array([0.05,0,0]).T),
                      pin.FrameType.OP_FRAME)
        )
        
        self.data = self.reducedModel.createData()
        if not self.reducedModel.existFrame(endEffectorName):
            print(f"Error: Frame '{endEffectorName}' not found in URDF!")
            sys.exit(1)
            
        self.eeId = self.reducedModel.getFrameId(endEffectorName)
        self.q = pin.neutral(self.reducedModel) 
        self.currentMesuredPos = pin.neutral(self.reducedModel)
        self.damp = 1e-2                 
        self.dt = 0.002                  

        self.cmodel = cpin.Model(self.reducedModel)
        self.cdata = self.cmodel.createData()

        self.cq = casadi.SX.sym("q", self.reducedModel.nq, 1)
        self.cTfTarget = casadi.SX.sym("tf_target", 4, 4) 
        
        cpin.framesForwardKinematics(self.cmodel, self.cdata, self.cq)
        
        self.transErrorFn = casadi.Function(
            "trans_error", 
            [self.cq, self.cTfTarget],
            [self.cdata.oMf[self.eeId].translation - self.cTfTarget[:3, 3]]
        )
        
        self.rotErrorFn = casadi.Function(
            "rot_error",
            [self.cq, self.cTfTarget],
            [cpin.log3(self.cdata.oMf[self.eeId].rotation @ self.cTfTarget[:3, :3].T)]
        )

        self.opti = casadi.Opti()
        
        self.varQ = self.opti.variable(self.reducedModel.nq)
        
        self.paramTfTarget = self.opti.parameter(4, 4)      
        self.paramQLast = self.opti.parameter(self.reducedModel.nq) 
        self.paramNeutral = self.opti.parameter(self.reducedModel.nq)
        self.opti.set_value(self.paramNeutral,[ 8.39828998e-02,  2.80442294e-02,  2.97575742e-02,  1.03259480e+00,
                                               -2.21307400e-05,  1.26232252e-01, -2.62358430e-04,  8.39352757e-02,
                                               -3.02904937e-02, -2.99052037e-02,  1.03319776e+00,  3.11785057e-04,
                                                1.26131579e-01,  3.66911350e-04,]) 
        self.costTrans = casadi.sumsqr(self.transErrorFn(self.varQ, self.paramTfTarget))
        self.costRot = casadi.sumsqr(self.rotErrorFn(self.varQ, self.paramTfTarget))
        
        self.costReg = casadi.sumsqr(self.varQ)
        
        self.costSmooth = casadi.sumsqr(self.varQ - self.paramQLast)
        
        self.costPosture = casadi.sumsqr(self.varQ - self.paramNeutral)
        self.opti.minimize(100. * self.costTrans + 10.0 * self.costRot + 0.02 * self.costReg + 2.0 * self.costSmooth + 0.1 * self.costPosture)

        self.opti.subject_to(self.opti.bounded(
            self.reducedModel.lowerPositionLimit,
            self.varQ,
            self.reducedModel.upperPositionLimit
        ))
        self.opti.subject_to(self.opti.bounded(
            -0.03,
            self.varQ - self.paramQLast,
            0.03
        ))

        opts = {
            'ipopt': {
                'print_level': 0,    
                'max_iter': 50,      
                'tol': 1e-4,         
                'warm_start_init_point': 'yes'
            },
            'print_time': False,
            'calc_lam_p': False
        }
        self.opti.solver("ipopt", opts)
        
        self.lastSolvedQ = pin.neutral(self.reducedModel)
        self.smoothFilter = WeightedMovingFilter(weights=[1.0], window_size=5)
        
        self.target_pos = np.array([0.3, 0.3, 0.23])#Right: ([0.3, -0.3, 0.23])
        self.target_rot = np.eye(3)

        self.timer = self.create_timer(self.dt, self.controlLoop)


    def leftHandCallback(self, msg):
        self.target_pos = np.array([msg.pose.position.y, 
                                    msg.pose.position.x * -1.0 , 
                                    (msg.pose.position.z * -1.0) + 0.24])

        quat = pin.Quaternion(msg.pose.orientation.w, 
                              msg.pose.orientation.y, 
                              msg.pose.orientation.x, 
                              msg.pose.orientation.z)
        self.target_rot = quat.matrix()

    def leftHandGripCallback(self, msg):
        self.leftHandOpen = msg.data

    def subscribeMotorData(self, msg: hg_LowState):
        msgIn = msg
        if msg is not None:
            lowstate = G1Motors()
            for id in range(35):
                lowstate.motorData[id].q  = msgIn.motor_state[id].q
                lowstate.motorData[id].dq = msgIn.motor_state[id].dq
            with self.motorDataLock:
                self.motorDataBuffer = lowstate
                self.modeMachine = msgIn.mode_machine
            self.firstRead = False
        time.sleep(0.002)

    def openLeftHand(self):
        left_max_pos = [0 ,  0 , 0 ,   0   ,  0    , 0     , 0]
        
        for i in range(7):
            # Left hand
            self.left_hand_cmd.motor_cmd[i].mode = 1    # 1 = Position control mode
            self.left_hand_cmd.motor_cmd[i].q = left_max_pos[i]    # 0 radians = fully open
            self.left_hand_cmd.motor_cmd[i].dq = 0.0
            self.left_hand_cmd.motor_cmd[i].kp = 20.0   
            self.left_hand_cmd.motor_cmd[i].kd = 1.0
            self.left_hand_cmd.motor_cmd[i].tau = 0.0
            
        self.left_hand_pub.Write(self.left_hand_cmd)
        self.get_logger().info("Dex3 left hand Commanded to OPEN")

    def openRightHand(self):
        right_max_pos = [0 , 0  ,   0  ,  0, 0 , 0  , 0]
        
        for i in range(7):
            # Right hand
            self.right_hand_cmd.motor_cmd[i].mode = 1
            self.right_hand_cmd.motor_cmd[i].q = right_max_pos[i]
            self.right_hand_cmd.motor_cmd[i].dq = 0.0
            self.right_hand_cmd.motor_cmd[i].kp = 20.0
            self.right_hand_cmd.motor_cmd[i].kd = 1.0
            self.right_hand_cmd.motor_cmd[i].tau = 0.0
            
        self.right_hand_pub.Write(self.right_hand_cmd)
        self.get_logger().info("Dex3 right hand Commanded to OPEN")

    def closeLeftHand(self):
        """Fully close both Dex3 hands by setting curling joints to upper limits."""
        left_min_pos = [ 0 , 0 ,   1.75  , -1.57 , -1.75 , -1.57  ,-1.75]
        for i in range(7):
            # Left hand
            self.left_hand_cmd.motor_cmd[i].mode = 1
            self.left_hand_cmd.motor_cmd[i].q = left_min_pos[i]
            self.left_hand_cmd.motor_cmd[i].dq = 0.0
            self.left_hand_cmd.motor_cmd[i].kp = 20.0 
            self.left_hand_cmd.motor_cmd[i].kd = 1.0
            self.left_hand_cmd.motor_cmd[i].tau = 0.0
            
            
        self.left_hand_pub.Write(self.left_hand_cmd)
        self.get_logger().info("Dex3 left hand Commanded to CLOSE")

    def closeRightHand(self):
        """Fully close both Dex3 hands by setting curling joints to upper limits."""
        right_min_pos = [0 , 0 , -1.75,    1.57  ,  1.57    ,   1.57   ,1.57]
        for i in range(7):            
            # Right hand
            self.right_hand_cmd.motor_cmd[i].mode = 1
            self.right_hand_cmd.motor_cmd[i].q = right_min_pos[i]
            self.right_hand_cmd.motor_cmd[i].dq = 0.0
            self.right_hand_cmd.motor_cmd[i].kp = 20.0
            self.right_hand_cmd.motor_cmd[i].kd = 1.0
            self.right_hand_cmd.motor_cmd[i].tau = 0.0
            
        self.right_hand_pub.Write(self.right_hand_cmd)
        self.get_logger().info("Dex3 right hand Commanded to CLOSE")
    def controlLoop(self):
        if self.firstRead:
            print("Waiting to subscribe to g1 joint positions")
            return
            
        if not self.initialized:
            print("Initializing start location")
            with self.motorDataLock:
                currentQSensor = pin.neutral(self.reducedModel)
                for name, motorId in JOINT_MAP.items():
                    if self.reducedModel.existJointName(name):
                        jointId = self.reducedModel.getJointId(name)
                        idx_q = self.reducedModel.joints[jointId].idx_q
                        
                        if idx_q != -1:
                            sensor_val = self.motorDataBuffer.motorData[motorId].q
                            
                            currentQSensor[idx_q] = sensor_val

                self.lastSolvedQ = currentQSensor
                print(self.lastSolvedQ)
            self.smoothFilter.add_data(self.lastSolvedQ)
            print("Starting Control Loop...")
            self.initialized = True
            return
        
        if self.handOnly:
            #open/close hand if position has changed
            if self.leftHandOpen != self.leftHandCurrentOpen:
                if self.leftHandOpen:
                    self.openLeftHand()
                else:
                    self.closeLeftHand()
                self.leftHandCurrentOpen = self.leftHandOpen
            return
        try:
            #open/close hand if position has changed
            if self.leftHandOpen != self.leftHandCurrentOpen:
                if self.leftHandOpen:
                    self.openLeftHand()
                else:
                    self.closeLeftHand()
                self.leftHandCurrentOpen = self.leftHandOpen

            with self.motorDataLock:
                currentSensor = copy.deepcopy(self.motorDataBuffer)
            currentQSensor = pin.neutral(self.reducedModel)
            for name, motorId in JOINT_MAP.items():
                if self.reducedModel.existJointName(name):
                    jointId = self.reducedModel.getJointId(name)
                    idx_q = self.reducedModel.joints[jointId].idx_q
                    
                    if idx_q != -1:
                        sensor_val = None
                        with self.motorDataLock:
                            sensor_val = currentSensor.motorData[motorId].q
                        
                        currentQSensor[idx_q] = sensor_val
            
            self.currentMesuredPos = currentQSensor

            qCmd ,vCmd, success = self.solveIk(self.target_pos, self.target_rot, self.currentMesuredPos)

            pin.forwardKinematics(self.reducedModel, self.data, qCmd)
            pin.updateFramePlacements(self.reducedModel, self.data)
            solved_pos = self.data.oMf[self.eeId].translation
            pos_error = np.linalg.norm(self.target_pos - solved_pos)

            velocity_effort = np.linalg.norm(vCmd)

            if pos_error < 0.02:
                print(f"\033[92m[OK] ARRIVED | Err: {pos_error*1000:.1f}mm\033[0m", end='\r')

            elif velocity_effort > 0.1:
                print(f"\033[93m[..] MOVING  | Err: {pos_error*1000:.1f}mm | Vel: {velocity_effort:.2f}\033[0m", end='\r')

            else:
                print(f"\033[91m[!!] STUCK   | Err: {pos_error*1000:.1f}mm | Vel: {velocity_effort:.2f} (Target Unreachable?)\033[0m")

            for name, motorId in JOINT_MAP.items():
                if self.reducedModel.existJointName(name):
                    jointId = self.reducedModel.getJointId(name)
                    idx_q = self.reducedModel.joints[jointId].idx_q
                    idx_v = self.reducedModel.joints[jointId].idx_v
                    
                    if idx_q != -1:
                        if "shoulder" in name or "elbow" in name:
                            
                            self.cmd.motor_cmd[motorId].mode = 1
                            self.cmd.motor_cmd[motorId].q = qCmd[idx_q]
                            self.cmd.motor_cmd[motorId].kp = 40.0  
                            #self.cmd.motor_cmd[motorId].dq = vCmd[idx_v]
                            self.cmd.motor_cmd[motorId].kd = 1   
                            #self.cmd.motor_cmd[motorId].tau = 0.0
                        else:
                            self.cmd.motor_cmd[motorId].mode = 1
                            self.cmd.motor_cmd[motorId].q = qCmd[idx_q]
                            self.cmd.motor_cmd[motorId].kp = 40.0  
                            #self.cmd.motor_cmd[motorId].dq = vCmd[idx_v]
                            self.cmd.motor_cmd[motorId].kd = 1   
                            #self.cmd.motor_cmd[motorId].tau = 0.0
            
            self.cmd.mode_pr = 1
            with self.motorDataLock:
                self.cmd.mode_machine = self.modeMachine
            
            self.cmd.crc = self.crc.Crc(self.cmd)
            self.pub.Write(self.cmd)
    
        except (KeyboardInterrupt, RuntimeError, SystemError) as e:
            error_msg = str(e)
            if isinstance(e, KeyboardInterrupt) or "KeyboardInterruptException" in error_msg or "Opti::solve" in error_msg:
                print("\n\033[93m[User Interrupt] Exiting cleanly...\033[0m")
            else:
                print(f"\n\033[91m[Error] Loop crashed: {e}\033[0m")

            self.running = False
            
            for i in range(35):
                self.cmd.motor_cmd[i].mode = 0x01 
                self.cmd.motor_cmd[i].q = 0.0
                self.cmd.motor_cmd[i].kp = 0.0
                self.cmd.motor_cmd[i].dq = 0.0
                self.cmd.motor_cmd[i].kd = 1.0    
                self.cmd.motor_cmd[i].tau = 0.0
            
            self.cmd.crc = self.crc.Crc(self.cmd)
            self.pub.Write(self.cmd)
            print("Motors damped!")
            
    def solveIk(self, targetPos, targetRot,currentMesuredPos):
        self.opti.set_initial(self.varQ, self.lastSolvedQ)
        tfTargetVal = np.eye(4)
        tfTargetVal[:3, :3] = targetRot
        tfTargetVal[:3, 3] = targetPos
        
        self.opti.set_value(self.paramTfTarget, tfTargetVal)
        self.opti.set_value(self.paramQLast, self.lastSolvedQ)
        
        try:
            solution = self.opti.solve()
            rawQ = self.opti.value(self.varQ)
            self.smoothFilter.add_data(rawQ)
            qNext = self.smoothFilter.filtered_data

            v = (qNext-self.lastSolvedQ)/self.dt
            self.lastSolvedQ = qNext
            return qNext, v, True
        except np.linalg.LinAlgError:
            print("IK Solver Singularity")
            return currentMesuredPos, np.zeros(self.reducedModel.nv),False
    
def main(args=None):
    ChannelFactoryInitialize(0) 
    rclpy.init(args=args)
    controller = G1VRController()

    try:
        # Spin keeps the node alive and processing callbacks
        rclpy.spin(controller)
    except (KeyboardInterrupt):
        print("Keyboard Interrupt")
        # Allow standard Ctrl+C to exit the spin loop gracefully
        pass
    except Exception as e:
        # Catch unexpected crashes
        print(f"[ERROR] Node crashed: {e}")
    finally:
        controller.destroy_node()

        # Only call shutdown if ROS is still running
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass

if __name__ == '__main__':
    main()