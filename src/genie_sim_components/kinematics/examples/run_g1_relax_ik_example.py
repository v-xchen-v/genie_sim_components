"""
Example: Using G1RelaxSolver to solve inverse kinematics for the G1 robot arm.
"""

from genie_sim_components.kinematics.g1_relax_ik import G1RelaxSolver
import numpy as np
import os

# Get absolute path relative to this script
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
URDF_PATH = os.path.join(CURRENT_DIR, "../configs/g1/G1_NO_GRIPPER.urdf")
CONFIG_PATH = os.path.join(CURRENT_DIR, "../configs/g1/g1_solver.yaml")

# Initialize the solver
solver = G1RelaxSolver(
    urdf_path=URDF_PATH,
    config_path=CONFIG_PATH,
    arm="right"
)

# Optional: Sync target with initial joint configuration
initial_joint_angles = np.zeros(7)
solver.set_current_state(initial_joint_angles)

# Example 1: Solve from 4x4 SE(3) pose
pose_matrix = np.eye(4)
pose_matrix[:3, 3] = [0.3, 0.2, 0.5]  # Set translation only
joint_solution = solver.solve_from_pose(pose_matrix)
print("Joint solution from SE(3) pose:\n", joint_solution)

# Example 2: Solve from position and quaternion
position = np.array([0.4, 0.1, 0.3])
quaternion_xyzw = np.array([0, 0, 0, 1])  # Identity quaternion
joint_solution = solver.solve_from_pos_quat(position, quaternion_xyzw)
print("Joint solution from pos + quat:\n", joint_solution)

# Example 3: Forward Kinematics
print("\n=== Forward Kinematics ===")
test_joint_angles = np.array([0.1, -0.2, 0.3, -0.1, 0.5, -0.4, 0.2])
ee_pose = solver.compute_fk(test_joint_angles)
print("End-effector pose from FK:\n", ee_pose)