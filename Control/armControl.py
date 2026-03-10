import time
import sys
import numpy as np
import pinocchio as pin
import casadi
from pinocchio import casadi as cpin    
import threading
import copy

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import (LowCmd_ as hg_LowCmd, LowState_ as hg_LowState)
from unitree_sdk2py.utils.crc import CRC
"""
X is forward/back (positive forward)
Y is left/right (positive is left)
Z is up/down (0.54 is shoulder height)
"""
# --- Configuration ---
URDF_PATH = "./g1_29dof.urdf" 
endEffectorName = "right_end_effector" 
lowStateTopic = "rt/lowstate"


# Map URDF joint names to Unitree G1 Motor IDs
JOINT_MAP = {
    # --- Right Arm ---
    "right_shoulder_pitch_joint": 22,
    "right_shoulder_roll_joint":  23,
    "right_shoulder_yaw_joint":   24,
    "right_elbow_joint":          25, 
    "right_wrist_roll_joint":     26,
    "right_wrist_pitch_joint":    27,
    "right_wrist_yaw_joint":      28,

    # --- Left Arm ---
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
        self.motorData = [MotorData() for _ in range (35)] #number of motors in G1

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
        # Simple weighted average of the last N elements
        return np.mean(self.data_buffer, axis=0)
    
        
class G1Helper:
    def __init__(self):
        # --- Unitree SDK Setup ---
        #Need to check settings for simulation/real
        ChannelFactoryInitialize(0,"enp58s0")#(1,"lo")#
        self.pub = ChannelPublisher("rt/arm_sdk", hg_LowCmd)
        self.pub.Init()
        self.cmd = unitree_hg_msg_dds__LowCmd_()
        self.crc = CRC()
        self.sub = ChannelSubscriber(lowStateTopic, hg_LowState)
        self.sub.Init()
        self.motorDataBuffer = None
        self.motorDataLock = threading.Lock()
        self.firstRead = True
        self.running = True
        # initialize subscribe thread
        self.subscribeThread = threading.Thread(target=self.subscribeMotorData)
        self.subscribeThread.daemon = True
        self.subscribeThread.start()
        # 3. Setup Helpers
        self.crc = CRC()

        # 4. Prepare Command Struct
        self.cmd = unitree_hg_msg_dds__LowCmd_()
        self.cmd.mode_pr = 0 
        self.cmd.mode_machine = 0
        # Load Model
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
        # Check Frame
        if not self.reducedModel.existFrame(endEffectorName):
            print(f"Error: Frame '{endEffectorName}' not found in URDF!")
            sys.exit(1)
            
        self.eeId = self.reducedModel.getFrameId(endEffectorName)
        self.q = pin.neutral(self.reducedModel) # Current estimated state
        self.currentMesuredPos = pin.neutral(self.reducedModel)
        self.damp = 1e-2                 # IK Damping factor
        self.dt = 0.002                  # 500Hz

        # CasADi + Pinocchio Setup
        # Create symbolic model and data
        self.cmodel = cpin.Model(self.reducedModel)
        self.cdata = self.cmodel.createData()

        # Symbolic Variables
        self.cq = casadi.SX.sym("q", self.reducedModel.nq, 1)
        self.cTfTarget = casadi.SX.sym("tf_target", 4, 4) # Target Homogeneous Matrix
        
        # Compute FK symbolically
        cpin.framesForwardKinematics(self.cmodel, self.cdata, self.cq)
        
        # 4. Define Error Functions (Symbolic)
        # Translation Error: Current Pos - Target Pos
        self.transErrorFn = casadi.Function(
            "trans_error", 
            [self.cq, self.cTfTarget],
            [self.cdata.oMf[self.eeId].translation - self.cTfTarget[:3, 3]]
        )
        
        # Rotation Error: Log3(Rotation Matrix Difference)
        self.rotErrorFn = casadi.Function(
            "rot_error",
            [self.cq, self.cTfTarget],
            [cpin.log3(self.cdata.oMf[self.eeId].rotation @ self.cTfTarget[:3, :3].T)]
        )

        # 5. Define Optimization Problem (NLP)
        self.opti = casadi.Opti()
        
        # Variables to optimize
        self.varQ = self.opti.variable(self.reducedModel.nq)
        
        # Parameters (Inputs that change every loop)
        self.paramTfTarget = self.opti.parameter(4, 4)      # The target pose
        self.paramQLast = self.opti.parameter(self.reducedModel.nq) # Previous q (for smoothing)
        self.paramNeutral = self.opti.parameter(self.reducedModel.nq)
        self.opti.set_value(self.paramNeutral,[ 8.39828998e-02,  2.80442294e-02,  2.97575742e-02,  1.03259480e+00,
                                               -2.21307400e-05,  1.26232252e-01, -2.62358430e-04,  8.39352757e-02,
                                               -3.02904937e-02, -2.99052037e-02,  1.03319776e+00,  3.11785057e-04,
                                                1.26131579e-01,  3.66911350e-04,]) # Arms at the side of the robot, wrists facing inwards
        # Cost Terms
        # Reach Target (Position + Rotation)
        self.costTrans = casadi.sumsqr(self.transErrorFn(self.varQ, self.paramTfTarget))
        self.costRot = casadi.sumsqr(self.rotErrorFn(self.varQ, self.paramTfTarget))
        
        # Regularization (Keep close to neutral/zero to avoid weird poses)
        self.costReg = casadi.sumsqr(self.varQ)
        
        # Smoothness (Minimize velocity / jerk)
        self.costSmooth = casadi.sumsqr(self.varQ - self.paramQLast)
        #
        self.costPosture = casadi.sumsqr(self.varQ - self.paramNeutral)
        # Total Cost (Weights taken from your G1 reference code)
        self.opti.minimize(100 * self.costTrans + 5.0 * self.costRot + 0.02 * self.costReg + 1.0 * self.costSmooth + 0.1 * self.costPosture)

        # Constraints (Joint Limits)
        self.opti.subject_to(self.opti.bounded(
            self.reducedModel.lowerPositionLimit,
            self.varQ,
            self.reducedModel.upperPositionLimit
        ))
        #limit joint speeds
        self.opti.subject_to(self.opti.bounded(
            -0.01,
            self.varQ - self.paramQLast,
            0.01
        ))

        # 6. Solver Settings (IPOPT)
        opts = {
            'ipopt': {
                'print_level': 0,    # Silent
                'max_iter': 50,      # Fast limits
                'tol': 1e-4,         # Loose tolerance for speed
                'warm_start_init_point': 'yes'
            },
            'print_time': False,
            'calc_lam_p': False
        }
        self.opti.solver("ipopt", opts)
        
        # Internal state for velocity calculation
        self.lastSolvedQ = pin.neutral(self.reducedModel)
        self.smoothFilter = WeightedMovingFilter(weights=[1.0], window_size=5)



    def subscribeMotorData(self):
        while self.running:
            msg = self.sub.Read()
            if msg is not None:
                lowstate = G1Motors()
                for id in range(35):
                    lowstate.motorData[id].q  = msg.motor_state[id].q
                    #print(lowstate.motorData[id].q)
                    lowstate.motorData[id].dq = msg.motor_state[id].dq
                    #print(msg.motor_state[id].q)
                with self.motorDataLock:
                    self.motorDataBuffer = lowstate

                self.firstRead = False
            time.sleep(0.002)


    def controlLoop(self):
        print("waiting for initial joint read")
        while self.firstRead:
            pass
        print("Initializing start location")
        with self.motorDataLock:
            #create a template q vector for the REDUCED model
            currentQSensor = pin.neutral(self.reducedModel)
            #Iterate over the joint Map 
            for name, motorId in JOINT_MAP.items():
                # Check if this joint exists in the reduced pinocchio model
                if self.reducedModel.existJointName(name):
                    jointId = self.reducedModel.getJointId(name)
                    idx_q = self.reducedModel.joints[jointId].idx_q
                    
                    # idx_q != -1 means the joint is not locked
                    if idx_q != -1:
                        # Get the value from the buffer
                        sensor_val = self.motorDataBuffer.motorData[motorId].q
                        
                        # Assign it to the correct index in the Pinocchio vector
                        currentQSensor[idx_q] = sensor_val

            self.lastSolvedQ = currentQSensor
            print(self.lastSolvedQ)
        self.smoothFilter.add_data(self.lastSolvedQ)
        print("Starting Control Loop...")
            # 5. Target Variables
        # Initial Target
        targetPos = np.array([0.2, 0.2, 0.6]) 
        targetRot = np.array([0,0,0,1])
    
        t = 0.0
        try:
            f = open("latest_pose.txt", 'r')
            while True:
                start_time = time.perf_counter()
    
                #Read most up to date values
                f.seek(0)
                line = f.readline()
                #parse values
                if line:
                    try:
                        # Parse CSV string to floats
                        # strip() removes the newline char
                        values = [float(x) for x in line.strip().split(',')]
                        if len(values) == 7:
                            targetPos = np.array([values[1],-1 * values[0],values[2] + 0.262])
                            quat = pin.Quaternion(values[6],values[3],values[4],values[5])#(0.0,0.0,0.0,1.0)#(values[6],values[3],values[4],values[5])
                            quat.normalize()
                            A = np.array([[0,1,0],
                                          [-1,0,0],
                                          [0,0,1]])
                            targetRot = A @ quat.matrix() @ A.T
                            #print(f"Tgt: {targetPos[0]:.3f}, {targetPos[1]:.3f}, {targetPos[2]:.3f}", end='\r')
                            
                    except ValueError:
                        # Occurs if reading while the file is being writrgetPos = np.array([values[0],values[1],values[2]+0.292])ten (empty or partial string)
                        # Skip this frame and keep the old values
                        pass

                #Setup current position and solve the inverse kinematics
                with self.motorDataLock:
                    currentSensor = copy.deepcopy(self.motorDataBuffer)
                #create a template q vector for the REDUCED model
                currentQSensor = pin.neutral(self.reducedModel)
                #Iterate over the joint Map 
                for name, motorId in JOINT_MAP.items():
                    # Check if this joint exists in the reduced pinocchio model
                    if self.reducedModel.existJointName(name):
                        jointId = self.reducedModel.getJointId(name)
                        idx_q = self.reducedModel.joints[jointId].idx_q
                        
                        # idx_q != -1 means the joint is not locked
                        if idx_q != -1:
                            # Get the value from the buffer
                            sensor_val = None
                            with self.motorDataLock:
                                sensor_val = currentSensor.motorData[motorId].q
                            
                            # Assign it to the correct index in the Pinocchio vector
                            currentQSensor[idx_q] = sensor_val
                
                # Update self.currentMesuredPos for the solver to use
                self.currentMesuredPos = currentQSensor

                # --- C. SOLVE IK ---
                # Pass the SENSOR derived position as the start point
                qCmd ,vCmd, success = self.solveIk(targetPos, targetRot, self.currentMesuredPos)

                # --- DEBUG CHECK ---
                #Calculate Position Error
                pin.forwardKinematics(self.reducedModel, self.data, qCmd)
                pin.updateFramePlacements(self.reducedModel, self.data)
                solved_pos = self.data.oMf[self.eeId].translation
                pos_error = np.linalg.norm(targetPos - solved_pos)

                #Calculate Command Intensity (How hard are we trying to move?)
                velocity_effort = np.linalg.norm(vCmd)

                #Status Logic
                if pos_error < 0.02:
                    # Less than 2cm error -> GREEN (.Arrived)
                    print(f"\033[92m[OK] ARRIVED | Err: {pos_error*1000:.1f}mm\033[0m", end='\r')

                elif velocity_effort > 0.1:
                    # Error is high, but we are moving fast -> YELLOW (Traveling)
                    print(f"\033[93m[..] MOVING  | Err: {pos_error*1000:.1f}mm | Vel: {velocity_effort:.2f}\033[0m", end='\r')

                else:
                    # Error is high AND we stopped moving -> RED (Stuck/Unreachable)
                    print(f"\033[91m[!!] STUCK   | Err: {pos_error*1000:.1f}mm | Vel: {velocity_effort:.2f} (Target Unreachable?)\033[0m")

                # iterate over the map to apply angles to specific motors
                for name, motorId in JOINT_MAP.items():
                    if self.reducedModel.existJointName(name):
                        jointId = self.reducedModel.getJointId(name)
                        idx_q = self.reducedModel.joints[jointId].idx_q
                        idx_v = self.reducedModel.joints[jointId].idx_v
                        print(qCmd)
                        if idx_q != -1:
                            if "shoulder" in name or "elbow" in name:
                                # Apply to command struct
                                self.cmd.motor_cmd[motorId].mode = 1
                                self.cmd.motor_cmd[motorId].q = qCmd[idx_q]
                                self.cmd.motor_cmd[motorId].kp = 80.0  # Stiffness
                                self.cmd.motor_cmd[motorId].dq = vCmd[idx_v]
                                self.cmd.motor_cmd[motorId].kd = 3   # Damping
                                self.cmd.motor_cmd[motorId].tau = 0.0
                            else:
                                # Apply to command struct
                                self.cmd.motor_cmd[motorId].mode = 1
                                self.cmd.motor_cmd[motorId].q = qCmd[idx_q]
                                self.cmd.motor_cmd[motorId].kp = 40.0  # Stiffness
                                self.cmd.motor_cmd[motorId].dq = vCmd[idx_v]
                                self.cmd.motor_cmd[motorId].kd = 2   # Damping
                                self.cmd.motor_cmd[motorId].tau = 0.0
    
                # --- D. PUBLISH ---
                self.cmd.crc = self.crc.Crc(self.cmd)
                self.pub.Write(self.cmd)
                # Sleep to maintain ~500Hz (0.002s)
                # Adjust sleep to account for computation time
                elapsed = time.perf_counter() - start_time
                sleep_time = 0.002 - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
    
        except (KeyboardInterrupt, RuntimeError, SystemError) as e:
            # CasADi throws a RuntimeError on Ctrl+C
            error_msg = str(e)
            if isinstance(e, KeyboardInterrupt) or "KeyboardInterruptException" in error_msg or "Opti::solve" in error_msg:
                print("\n\033[93m[User Interrupt] Exiting cleanly...\033[0m")
            else:
                # If it was a real crash, print it
                print(f"\n\033[91m[Error] Loop crashed: {e}\033[0m")

            self.running = False
            
            # --- SAFE SHUTDOWN ---
            # Return all motors to a safe default (Damping mode)
            for i in range(35):
                self.cmd.motor_cmd[i].mode = 0x01 # PMSM mode
                self.cmd.motor_cmd[i].q = 0.0
                self.cmd.motor_cmd[i].kp = 0.0
                self.cmd.motor_cmd[i].dq = 0.0
                self.cmd.motor_cmd[i].kd = 1.0    # Slight damping for safety
                self.cmd.motor_cmd[i].tau = 0.0
            
            self.cmd.crc = self.crc.Crc(self.cmd)
            self.pub.Write(self.cmd)
            print("Motors damped!")
            
    #target position as a tuple, target rotation as a matrix and current position as a pinocchio model
    #current position is that robot measured current position
    def solveIk(self, targetPos, targetRot,currentMesuredPos):
        """
        Computes the next joint configuration (q) to reach targetPos/rot
        """
        self.opti.set_initial(self.varQ, self.lastSolvedQ)
        # Prep Target
        tfTargetVal = np.eye(4)
        tfTargetVal[:3, :3] = targetRot
        tfTargetVal[:3, 3] = targetPos
        
        # Set Parameters
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
        

if __name__ == "__main__":
    print("Initializing Unitree SDK and Pinocchio IK...")

    motorControl = G1Helper()
    motorControl.controlLoop()
