import numpy as np

JOINT_MAP = {
    "right_shoulder_pitch_joint": 22, "right_shoulder_roll_joint": 23,
    "right_shoulder_yaw_joint": 24, "right_elbow_joint": 25, 
    "right_wrist_roll_joint": 26, "right_wrist_pitch_joint": 27,
    "right_wrist_yaw_joint": 28, "left_shoulder_pitch_joint": 15,
    "left_shoulder_roll_joint": 16, "left_shoulder_yaw_joint": 17,
    "left_elbow_joint": 18, "left_wrist_roll_joint": 19,
    "left_wrist_pitch_joint": 20, "left_wrist_yaw_joint": 21,
}

class MotorData:
    def __init__(self):
        self.q = 0.0
        self.dq = 0.0

class G1Motors:
    def __init__(self):
        self.motorData = [MotorData() for _ in range(35)]

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