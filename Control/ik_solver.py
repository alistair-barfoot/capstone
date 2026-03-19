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
        
        self.reduced_robot = self.robot.buildReducedRobot(
            list_of_joints_to_lock=locked_joints,
            reference_configuration=np.array([0.0] * self.robot.model.nq),
        )      
        self.model = self.reduced_robot.model  
        
        # Add end effector frames to account for additional hand length
        self.model.addFrame(pin.Frame('left_end_effector', self.model.getJointId('left_wrist_yaw_joint'), pin.SE3(np.eye(3), np.array([0.05,0,0]).T), pin.FrameType.OP_FRAME))
        self.model.addFrame(pin.Frame('right_end_effector', self.model.getJointId('right_wrist_yaw_joint'), pin.SE3(np.eye(3), np.array([0.05,0,0]).T), pin.FrameType.OP_FRAME))
        
        #add elbow frames to make it easier to track elbow position and prevent colisions
        self.model.addFrame(pin.Frame('left_elbow_col', self.model.getJointId('left_elbow_joint'), pin.SE3.Identity(), pin.FrameType.OP_FRAME))
        self.model.addFrame(pin.Frame('right_elbow_col', self.model.getJointId('right_elbow_joint'), pin.SE3.Identity(), pin.FrameType.OP_FRAME))


        self.data = self.model.createData()
        self.left_ee_id = self.model.getFrameId('left_end_effector')
        self.right_ee_id = self.model.getFrameId('right_end_effector')
        
        self.left_elbow_id = self.model.getFrameId('left_elbow_col')
        self.right_elbow_id = self.model.getFrameId('right_elbow_col')

        self._setup_casadi()

    def _setup_casadi(self):
        cmodel = cpin.Model(self.model)
        cdata = cmodel.createData()

        #Create SX symbols for Pinocchio
        cq = casadi.SX.sym("q", self.model.nq, 1)
        cTfTarget_L = casadi.SX.sym("tf_target_L", 4, 4)
        cTfTarget_R = casadi.SX.sym("tf_target_R", 4, 4)

        # Calculate forward kinematics using the SX symbol
        cpin.framesForwardKinematics(cmodel, cdata, cq)
        
        trans_err_L_fn = casadi.Function("trans_err_L", [cq, cTfTarget_L], [cdata.oMf[self.left_ee_id].translation - cTfTarget_L[:3, 3]])
        rot_err_L_fn = casadi.Function("rot_err_L", [cq, cTfTarget_L], [cpin.log3(cdata.oMf[self.left_ee_id].rotation @ cTfTarget_L[:3, :3].T)])
        
        trans_err_R_fn = casadi.Function("trans_err_R", [cq, cTfTarget_R], [cdata.oMf[self.right_ee_id].translation - cTfTarget_R[:3, 3]])
        rot_err_R_fn = casadi.Function("rot_err_R", [cq, cTfTarget_R], [cpin.log3(cdata.oMf[self.right_ee_id].rotation @ cTfTarget_R[:3, :3].T)])

        #get x and y position of both elbows
        left_elbow_xy = cdata.oMf[self.left_elbow_id].translation[:2]
        right_elbow_xy = cdata.oMf[self.right_elbow_id].translation[:2]
        
        # Create a function that calculates the squared distance from the Z-axis (X=0, Y=0)
        elbow_dist_sq_L_fn = casadi.Function("dist_L", [cq], [casadi.sumsqr(left_elbow_xy)])
        elbow_dist_sq_R_fn = casadi.Function("dist_R", [cq], [casadi.sumsqr(right_elbow_xy)])

        self.opti = casadi.Opti()
        self.varQ = self.opti.variable(self.model.nq)
        
        self.param_tf_target_L = self.opti.parameter(4, 4)      
        self.param_tf_target_R = self.opti.parameter(4, 4)
        self.param_q_last = self.opti.parameter(self.model.nq) 
        self.param_neutral = self.opti.parameter(self.model.nq)
        
        self.opti.set_value(self.param_neutral,[ 8.39828998e-02,  2.80442294e-02,  2.97575742e-02,  1.03259480e+00,
                                               -2.21307400e-05,  1.26232252e-01, -2.62358430e-04,  8.39352757e-02,
                                               -3.02904937e-02, -2.99052037e-02,  1.03319776e+00,  3.11785057e-04,
                                                1.26131579e-01,  3.66911350e-04,]) 
        cost_trans = casadi.sumsqr(trans_err_L_fn(self.varQ, self.param_tf_target_L)) + casadi.sumsqr(trans_err_R_fn(self.varQ, self.param_tf_target_R))
        cost_rot = casadi.sumsqr(rot_err_L_fn(self.varQ, self.param_tf_target_L)) + casadi.sumsqr(rot_err_R_fn(self.varQ, self.param_tf_target_R))
        
        cost_reg = casadi.sumsqr(self.varQ)
        cost_smooth = casadi.sumsqr(self.varQ - self.param_q_last)
        cost_posture = casadi.sumsqr(self.varQ - self.param_neutral)
        
        #prevent elbows from hitting robot
        safe_radius = 0.165  # 16 cm radius
        safe_radius_sq = safe_radius ** 2 
        costCol_L = casadi.sumsqr(casadi.fmax(0, safe_radius_sq - elbow_dist_sq_L_fn(self.varQ)))
        costCol_R = casadi.sumsqr(casadi.fmax(0, safe_radius_sq - elbow_dist_sq_R_fn(self.varQ)))

        self.opti.minimize(100. * cost_trans + 10.0 * cost_rot + 0.02 * cost_reg + 2.0 * cost_smooth + 0.05 * cost_posture + 10000. * (costCol_L + costCol_R))

        # Constraints
        self.opti.subject_to(self.opti.bounded(
            self.model.lowerPositionLimit, 
            self.varQ, 
            self.model.upperPositionLimit
            ))
        
        self.opti.subject_to(self.opti.bounded(
            -0.03, 
            self.varQ - self.param_q_last, 
            0.03
            ))
        


        self.opti.solver("ipopt", {'ipopt': {
            'print_level': 0,
            'max_iter': 50,
            'tol': 1e-4,
            'warm_start_init_point': 'yes'
            }, 
            'print_time': False, 
            'calc_lam_p': False
            })
    
    
    def solve(self, target_pos_l, target_rot_l, target_pos_r, target_rot_r, last_q):
        self.opti.set_initial(self.varQ, last_q)
        
        tf_l, tf_r = np.eye(4), np.eye(4)
        tf_l[:3, :3], tf_l[:3, 3] = target_rot_l, target_pos_l
        tf_r[:3, :3], tf_r[:3, 3] = target_rot_r, target_pos_r
        
        self.opti.set_value(self.param_tf_target_L, tf_l)
        self.opti.set_value(self.param_tf_target_R, tf_r)
        self.opti.set_value(self.param_q_last, last_q)
        
        try:
            self.opti.solve()
            raw_q = self.opti.value(self.varQ)
            return raw_q, (raw_q - last_q) / self.dt, True
        except RuntimeError:
            return last_q, np.zeros(self.model.nv), False