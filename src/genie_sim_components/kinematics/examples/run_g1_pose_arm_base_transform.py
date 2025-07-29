from genie_sim_components.kinematics.pose_transformer import PoseTransformer
import numpy as np

import os
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
URDF_PATH = os.path.join(CURRENT_DIR, "../configs/g1/G1_omnipicker.urdf")
transformer = PoseTransformer(URDF_PATH)

# list actuators
for joint in transformer.robot.actuated_joints:
    print(joint.name)
"""
idx01_body_joint1
idx02_body_joint2
idx11_head_joint1
idx12_head_joint2
idx21_arm_l_joint1
idx61_arm_r_joint1
idx62_arm_r_joint2
idx22_arm_l_joint2
idx63_arm_r_joint3
idx23_arm_l_joint3
idx24_arm_l_joint4
idx64_arm_r_joint4
idx25_arm_l_joint5
idx65_arm_r_joint5
idx26_arm_l_joint6
idx66_arm_r_joint6
idx27_arm_l_joint7
idx67_arm_r_joint7
idx41_gripper_l_outer_joint1
idx39_gripper_l_inner_joint2
idx81_gripper_r_outer_joint1
idx89_gripper_r_outer_joint2
idx79_gripper_r_inner_joint2
idx49_gripper_l_outer_joint2
idx82_gripper_r_outer_joint3
idx72_gripper_r_inner_joint3
idx32_gripper_l_inner_joint3
idx42_gripper_l_outer_joint3
idx43_gripper_l_outer_joint4
idx33_gripper_l_inner_joint4
idx73_gripper_r_inner_joint4
idx83_gripper_r_outer_joint4
"""    

# Input pose in robot base frame
pos = np.array([0.4, -0.1, 0.3])
quat = np.array([0, 0, 0, 1])
T_pose = transformer.make_pose_matrix(pos, quat)

# Transform to arm_base_link frame
T_in_arm = transformer.transform_ee_pose_between_base_and_arm(T_pose, direction='base_to_arm', joint_angles=joint_angles)
pos_out, quat_out = transformer.decompose_pose_matrix(T_in_arm)
print("Transformed pose in arm_base_link frame:", pos_out, quat_out)

# And back
T_in_base = transformer.transform_ee_pose_between_base_and_arm(T_in_arm, direction='arm_to_base', joint_angles=joint_angles)

# Decompose for verification
pos_out, quat_out = transformer.decompose_pose_matrix(T_in_base)
print("Recovered:", pos_out, quat_out)
