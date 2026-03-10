import numpy as np
import casadi
import pinocchio as pin
from pinocchio import casadi as cpin

class DualArmIKSolver:
    def __init__(self, urdf_path, dt=0.002):
        self.dt = dt
        try:
            self.robot = pin.RobotWrapper.BuildFromURDF(urdf_path, '.')
        except Exception as e:
            raise RuntimeError(f"Error loading URDF: {e}")

        locked_joints = [
            "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
            "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
            "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
            "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
            "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"
        ]
        
        self.reducedRobot = self.robot.buildReducedRobot(
            list_of_joints_to_lock=locked_joints,
            reference_configuration=np.array([0.0] * self.robot.model.nq),
        )      
        self.model = self.reducedRobot.model  
        
        # Add both frames
        self.model.addFrame(pin.Frame('left_end_effector', self.model.getJointId('left_wrist_yaw_joint'), pin.SE3(np.eye(3), np.array([0.05,0,0]).T), pin.FrameType.OP_FRAME))
        self.model.addFrame(pin.Frame('right_end_effector', self.model.getJointId('right_wrist_yaw_joint'), pin.SE3(np.eye(3), np.array([0.05,0,0]).T), pin.FrameType.OP_FRAME))
        
        self.data = self.model.createData()
        self.left_ee_id = self.model.getFrameId('left_end_effector')
        self.right_ee_id = self.model.getFrameId('right_end_effector')
        
        self._setup_casadi()

    def _setup_casadi(self):
        cmodel = cpin.Model(self.model)
        cdata = cmodel.createData()

        cq = casadi.SX.sym("q", self.model.nq, 1)
        cpin.framesForwardKinematics(cmodel, cdata, cq)
        
        self.opti = casadi.Opti()
        self.varQ = self.opti.variable(self.model.nq)
        
        # Dual Targets
        self.paramTfTarget_L = self.opti.parameter(4, 4)      
        self.paramTfTarget_R = self.opti.parameter(4, 4)
        self.paramQLast = self.opti.parameter(self.model.nq) 
        self.paramNeutral = self.opti.parameter(self.model.nq)
        
        self.opti.set_value(self.paramNeutral,[ 8.39828998e-02,  2.80442294e-02,  2.97575742e-02,  1.03259480e+00,
                                               -2.21307400e-05,  1.26232252e-01, -2.62358430e-04,  8.39352757e-02,
                                               -3.02904937e-02, -2.99052037e-02,  1.03319776e+00,  3.11785057e-04,
                                                1.26131579e-01,  3.66911350e-04,])         
        # Error tracking for both arms
        trans_err_L = casadi.sumsqr(cdata.oMf[self.left_ee_id].translation - self.paramTfTarget_L[:3, 3])
        rot_err_L = casadi.sumsqr(cpin.log3(cdata.oMf[self.left_ee_id].rotation @ self.paramTfTarget_L[:3, :3].T))
        
        trans_err_R = casadi.sumsqr(cdata.oMf[self.right_ee_id].translation - self.paramTfTarget_R[:3, 3])
        rot_err_R = casadi.sumsqr(cpin.log3(cdata.oMf[self.right_ee_id].rotation @ self.paramTfTarget_R[:3, :3].T))

        costTrans = trans_err_L + trans_err_R
        costRot = rot_err_L + rot_err_R
        costReg = casadi.sumsqr(self.varQ)
        costSmooth = casadi.sumsqr(self.varQ - self.paramQLast)
        costPosture = casadi.sumsqr(self.varQ - self.paramNeutral)
        
        self.opti.minimize(100. * costTrans + 10.0 * costRot + 0.02 * costReg + 2.0 * costSmooth + 0.1 * costPosture)

        # Constraints
        self.opti.subject_to(self.opti.bounded(self.model.lowerPositionLimit, self.varQ, self.model.upperPositionLimit))
        self.opti.subject_to(self.opti.bounded(-0.03, self.varQ - self.paramQLast, 0.03))

        self.opti.solver("ipopt", {'ipopt': {'print_level': 0, 'max_iter': 50, 'tol': 1e-4, 'warm_start_init_point': 'yes'}, 'print_time': False, 'calc_lam_p': False})

    def solve(self, target_pos_l, target_rot_l, target_pos_r, target_rot_r, last_q):
        self.opti.set_initial(self.varQ, last_q)
        
        tfL, tfR = np.eye(4), np.eye(4)
        tfL[:3, :3], tfL[:3, 3] = target_rot_l, target_pos_l
        tfR[:3, :3], tfR[:3, 3] = target_rot_r, target_pos_r
        
        self.opti.set_value(self.paramTfTarget_L, tfL)
        self.opti.set_value(self.paramTfTarget_R, tfR)
        self.opti.set_value(self.paramQLast, last_q)
        
        try:
            self.opti.solve()
            raw_q = self.opti.value(self.varQ)
            return raw_q, (raw_q - last_q) / self.dt, True
        except RuntimeError:
            return last_q, np.zeros(self.model.nv), False