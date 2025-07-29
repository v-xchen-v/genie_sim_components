from genie_sim_components.kinematics.pose_transformer import PoseTransformer
import numpy as np

import os
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
URDF_PATH = os.path.join(CURRENT_DIR, "../configs/g1/G1_omnipicker.urdf")
transformer = PoseTransformer(URDF_PATH)

# Input pose in robot base frame
pos = np.array([0.4, -0.1, 0.3])
quat = np.array([0, 0, 0, 1])
T_pose = transformer.make_pose_matrix(pos, quat)

# Transform to arm_base_link frame
T_in_arm = transformer.transform_ee_pose_between_base_and_arm(T_pose, direction='base_to_arm')
pos_out, quat_out = transformer.decompose_pose_matrix(T_in_arm)
print("Transformed pose in arm_base_link frame:", pos_out, quat_out)

# And back
T_in_base = transformer.transform_ee_pose_between_base_and_arm(T_in_arm, direction='arm_to_base')

# Decompose for verification
pos_out, quat_out = transformer.decompose_pose_matrix(T_in_base)
print("Recovered:", pos_out, quat_out)
