from genie_sim_components.kinematics.pose_transformer import PoseTransformer
import numpy as np

import os
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
URDF_PATH = os.path.join(CURRENT_DIR, "../configs/g1/G1_omnipicker.urdf")
transformer = PoseTransformer(URDF_PATH)

pos = np.array([0.1, 0.0, 0.2])
quat = np.array([0, 0, 0, 1])
T_pose = transformer.make_pose_matrix(pos, quat)

"""
joints between base_link and arm_base_link:
idx01_body_joint1
idx02_body_joint2
idx11_head_joint1
idx12_head_joint2
these joint angles will influence the transform between base_link and arm_base_link.
"""
joint_angles = {
    'idx01_body_joint1': 0.0,
    'idx02_body_joint2': -0.5,
    'idx11_head_joint1': 0.1,
    'idx12_head_joint2': -0.1,
}

# Convert from base_link to arm_base_link
T_in_arm = transformer.transform_pose(T_pose, from_link='base_link', to_link='arm_base_link', joint_angles=joint_angles)
print("Transformed pose in arm_base_link frame:", transformer.decompose_pose_matrix(T_in_arm))

# Convert back
T_back = transformer.transform_pose_inverse(T_in_arm, from_link='base_link', to_link='arm_base_link', joint_angles=joint_angles)
print("Recovered pose:", transformer.decompose_pose_matrix(T_back))
