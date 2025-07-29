from genie_sim_components.kinematics.pose_transformer import PoseTransformer
import numpy as np

import os
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
URDF_PATH = os.path.join(CURRENT_DIR, "../configs/g1/G1_omnipicker.urdf")
transformer = PoseTransformer(URDF_PATH)

pos = np.array([0.1, 0.0, 0.2])
quat = np.array([0, 0, 0, 1])
T_pose = transformer.make_pose_matrix(pos, quat)

# Convert from base_link to arm_base_link
T_in_arm = transformer.transform_pose(T_pose, from_link='base_link', to_link='arm_base_link')

# Convert back
T_back = transformer.transform_pose_inverse(T_in_arm, from_link='base_link', to_link='arm_base_link')
print("Recovered pose:", transformer.decompose_pose_matrix(T_back))
