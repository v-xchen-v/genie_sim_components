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


class PoseTransformer:
    def __init__(self, urdf_path_or_object):
        """
        Initialize the PoseTransformer.

        Args:
            urdf_path_or_object (str or URDF): Path to URDF file or a urdfpy.URDF instance.
        """
        if isinstance(urdf_path_or_object, str):
            self.robot = URDF.load(urdf_path_or_object)
        elif isinstance(urdf_path_or_object, URDF):
            self.robot = urdf_path_or_object
        else:
            raise TypeError("Expected URDF object or path to URDF")

        self._linkname2id = {link.name: i for i, link in enumerate(self.robot.links)}
        self._id2linkname = {i: link.name for i, link in enumerate(self.robot.links)}

    def _get_link_id(self, link_name):
        if link_name not in self._linkname2id:
            raise ValueError(f"Link name '{link_name}' not found in URDF.")
        return self._linkname2id[link_name]

    def _get_link_name(self, link_id):
        return self._id2linkname.get(link_id, None)

    def get_link_pose(self, link_name, joint_angles=None):
        """
        Get the 4x4 transform of a link in the world (root) frame.
        """
        if joint_angles is None:
            fk = self.robot.link_fk()
        else:
            fk = self.robot.link_fk(cfg=joint_angles)

        link = self.robot.links[self._get_link_id(link_name)]
        return fk[link]

    def get_transform(self, from_link, to_link, joint_angles=None):
        """
        Get transform matrix from `from_link` to `to_link`.
        """
        T_to = self.get_link_pose(to_link, joint_angles=joint_angles)
        T_from = self.get_link_pose(from_link, joint_angles=joint_angles)
        return T_to @ np.linalg.inv(T_from)

    def transform_pose(self, pose, from_link, to_link, joint_angles=None):
        """
        Transform a 4x4 pose from `from_link` to `to_link`.
        """
        T = self.get_transform(from_link, to_link, joint_angles)
        return T @ pose

    def transform_pose_inverse(self, pose, from_link, to_link, joint_angles=None):
        """
        Transform a 4x4 pose from `to_link` back to `from_link`.
        """
        T = self.get_transform(from_link, to_link, joint_angles)
        return np.linalg.inv(T) @ pose

    def transform_ee_pose_between_base_and_arm(
        self,
        pose: np.ndarray,
        direction: str,
        robot_base_link: str = 'base_link',
        arm_base_link: str = 'arm_base_link',
        joint_angles: dict = None
    ) -> np.ndarray:
        """
        Convenience wrapper for transforming EE pose between robot base and arm base.

        Args:
            pose: 4x4 homogeneous pose.
            direction: 'base_to_arm' or 'arm_to_base'.
            joint_angles: Optional dict of joint angles.

        Returns:
            Transformed 4x4 pose.
        """
        if direction == 'base_to_arm':
            return self.transform_pose(pose, robot_base_link, arm_base_link, joint_angles)
        elif direction == 'arm_to_base':
            return self.transform_pose(pose, arm_base_link, robot_base_link, joint_angles)
        else:
            raise ValueError("direction must be either 'base_to_arm' or 'arm_to_base'")

    @staticmethod
    def make_pose_matrix(position, quaternion):
        """
        Create 4x4 homogeneous transform from position + quaternion.
        """
        T = np.eye(4)
        T[:3, :3] = R.from_quat(quaternion).as_matrix()
        T[:3, 3] = position
        return T

    @staticmethod
    def decompose_pose_matrix(T):
        """
        Extract (position, quaternion) from 4x4 pose.
        """
        pos = T[:3, 3]
        quat = R.from_matrix(T[:3, :3]).as_quat()
        return pos, quat
