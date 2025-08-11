# kinematics/pose_transformer.py

# --- Compatibility patches for older libraries like urdfpy ---
import collections
import collections.abc
collections.Mapping = collections.abc.Mapping
collections.Set = collections.abc.Set
collections.Iterable = collections.abc.Iterable

import math
import fractions
fractions.gcd = math.gcd

import numpy as np
np.int = int
np.float_ = float
np.float = float


from urdfpy import URDF
from scipy.spatial.transform import Rotation as R

robot = URDF.load("src/genie_sim_components/kinematics/configs/g1/G1_omnipicker.urdf")  # Load your URDF file here
# def head_to_armr_transform(q_head1, q_head2):
#     """
#     Return 4x4 transform from arm_r_base_link -> head_link2
#     given head joint angles (radians).
#     """
#     cfg = {
#         "idx11_head_joint1": float(q_head1),
#         "idx12_head_joint2": float(q_head2),
#         # all other joints default to 0; fixed joints are handled automatically
#     }
#     # Forward kinematics to every link (relative to root/base_link)
#     fk = robot.link_fk(cfg)

#     T_base_to_armr = fk[robot.link_map["arm_r_base_link"]]
#     T_base_to_head = fk[robot.link_map["head_link2"]]

#     # Relative: arm_r_base_link -> head_link2
#     T_armr_to_head = np.linalg.inv(T_base_to_armr) @ T_base_to_head
#     return T_armr_to_head


# # Example
# q1 = 0.10  # idx11_head_joint1 in radians
# q2 = -0.20 # idx12_head_joint2 in radians
# T_armr_to_head = head_to_armr_transform(q1, q2)

# # (optional) Decompose to R, t
# R_armr_to_head = T_armr_to_head[:3, :3]
# t_armr_to_head = T_armr_to_head[:3, 3]
# euler_xyz = R.from_matrix(R_armr_to_head).as_euler('xyz')  # if you want Euler
# print("T_armr_to_head:\n", T_armr_to_head)
# print("t:", t_armr_to_head, "euler_xyz:", euler_xyz)

# # If you need the inverse (head_link2 -> arm_r_base_link):
# T_head_to_armr = np.linalg.inv(T_armr_to_head)

def relative_transform(robot, link_from, link_to, joint_values):
    """
    Compute transform from link_from -> link_to.

    Parameters:
        robot: urdfpy.URDF
        link_from (str): name of source link
        link_to (str): name of target link
        joint_values (dict): joint name -> value in radians (others default to 0)
    Returns:
        4x4 numpy array
    """
    fk = robot.link_fk(joint_values)
    T_base_from = fk[robot.link_map[link_from]]
    T_base_to = fk[robot.link_map[link_to]]
    return np.linalg.inv(T_base_from) @ T_base_to

# Example joint values
q1 = 0.10  # idx11_head_joint1
q2 = -0.20 # idx12_head_joint2

joint_cfg = {
    "idx11_head_joint1": q1,
    "idx12_head_joint2": q2
}

# Optional: Decompose to rotation (Euler) + translation
def decompose_transform(T):
    R_mat = T[:3, :3]
    t_vec = T[:3, 3]
    euler_xyz = R.from_matrix(R_mat).as_euler('xyz')
    return t_vec, euler_xyz

# Forward transforms
T_armr_to_head = relative_transform(robot, "arm_r_base_link", "head_link2", joint_cfg)
T_arml_to_head = relative_transform(robot, "arm_l_base_link", "head_link2", joint_cfg)

# Reverse transforms
T_head_to_armr = np.linalg.inv(T_armr_to_head)
T_head_to_arml = np.linalg.inv(T_arml_to_head)

# Print all
print("arm_r_base_link -> head_link2:\n", T_armr_to_head)
print("head_link2 -> arm_r_base_link:\n", T_head_to_armr)

print("arm_l_base_link -> head_link2:\n", T_arml_to_head)
print("head_link2 -> arm_l_base_link:\n", T_head_to_arml)

# Optional: decomposed into translation + Euler rotation
print("\nDecomposed (arm_r -> head):", decompose_transform(T_armr_to_head))
print("Decomposed (head -> arm_r):", decompose_transform(T_head_to_armr))
print("Decomposed (arm_l -> head):", decompose_transform(T_arml_to_head))
print("Decomposed (head -> arm_l):", decompose_transform(T_head_to_arml))