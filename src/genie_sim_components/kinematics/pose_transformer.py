# kinematics/pose_transformer.py

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
np.float_ = float  # Monkey patch to avoid AttributeError
np.float = float
import numpy as np
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
            raise TypeError("Input must be a URDF object or path to a URDF file.")

        self._transform_cache = {}  # (from_link, to_link) -> 4x4 matrix

    def _init_urdfpy_robot(self):
        """
        Initialize the URDF robot model if not already done.
        This is a placeholder for any additional initialization logic.
        """
        if not hasattr(self, 'robot'):
            raise ValueError("URDF robot model is not initialized.")
        
        self.fk = self.robot.link_fk()
        # linkname2id and id2linkname
        self.linkname2id = {link.name: i for i, link in enumerate(self.robot.links)}
        self.id2linkname = {i: link.name for i, link in enumerate(self.robot.links)}
        
    def _get_link_id(self, link_name):
        """
        Get the ID of a link by its name.
        
        Args:
            link_name (str): Name of the link.
        
        Returns:
            int: ID of the link.
        """
        if not hasattr(self, 'linkname2id'):
            self._init_urdfpy_robot()
        return self.linkname2id.get(link_name, None)
    
    def _get_link_name(self, link_id):
        """
        Get the name of a link by its ID.
        
        Args:
            link_id (int): ID of the link.
        
        Returns:
            str: Name of the link.
        """
        if not hasattr(self, 'id2linkname'):
            self._init_urdfpy_robot()
        return self.id2linkname.get(link_id, None)
    
    def _get_link_pose(self, link_name):
        """
        Get the pose of a link in the base frame.
        
        Args:
            link_name (str): Name of the link.
        
        Returns:
            np.ndarray: 4x4 transformation matrix representing the pose.
        """
        if not hasattr(self, 'fk'):
            self._init_urdfpy_robot()
        if link_name not in [link.name for link in self.robot.links]:
            raise ValueError(f"Link '{link_name}' not found in the robot model.")
        return self.fk[self.robot.links[self._get_link_id(link_name)]]
        
    def get_transform(self, from_link, to_link):
        """
        Get the 4x4 transformation matrix from from_link to to_link.

        Caches result for future use.
        """
        key = (from_link, to_link)
        if key not in self._transform_cache:
            # try:
            T_to_link = self._get_link_pose(to_link)
            T_from_link = self._get_link_pose(from_link)
            T = T_to_link @ np.linalg.inv(T_from_link)  # from from_link to to_link
            # except Exception as e:
            #     raise ValueError(f"Failed to compute transform from '{from_link}' to '{to_link}': {e}")
            self._transform_cache[key] = T
        return self._transform_cache[key]

    def transform_pose(self, pose: np.ndarray, from_link: str, to_link: str) -> np.ndarray:
        """
        Transform a pose (4x4 matrix) from from_link frame to to_link frame.
        """
        T_from_to = self.get_transform(from_link, to_link)
        return T_from_to @ pose

    def transform_pose_inverse(self, pose: np.ndarray, from_link: str, to_link: str) -> np.ndarray:
        """
        Transform a pose in the opposite direction: from to_link to from_link.
        """
        T_from_to = self.get_transform(from_link, to_link)
        T_to_from = np.linalg.inv(T_from_to)
        return T_to_from @ pose

    @staticmethod
    def make_pose_matrix(position: np.ndarray, quaternion: np.ndarray) -> np.ndarray:
        """
        Create a 4x4 homogeneous transform matrix from position and quaternion.
        """
        T = np.eye(4)
        T[:3, :3] = R.from_quat(quaternion).as_matrix()
        T[:3, 3] = position
        return T

    @staticmethod
    def decompose_pose_matrix(T: np.ndarray):
        """
        Extract (position, quaternion) from a 4x4 pose matrix.
        """
        position = T[:3, 3]
        quaternion = R.from_matrix(T[:3, :3]).as_quat()
        return position, quaternion

    def transform_ee_pose_between_base_and_arm(
        self,
        pose: np.ndarray,
        direction: str,
        robot_base_link: str = 'base_link',
        arm_base_link: str = 'arm_base_link'
    ) -> np.ndarray:
        """
        Convenience wrapper to transform an end-effector pose between robot base and arm base frame.

        Args:
            pose: 4x4 homogeneous pose matrix of the end-effector.
            direction: Either 'base_to_arm' or 'arm_to_base'.
            robot_base_link: Name of the robot base link.
            arm_base_link: Name of the arm base link.

        Returns:
            4x4 transformed pose matrix.
        """
        if direction == 'base_to_arm':
            return self.transform_pose(pose, from_link=robot_base_link, to_link=arm_base_link)
        elif direction == 'arm_to_base':
            return self.transform_pose(pose, from_link=arm_base_link, to_link=robot_base_link)
        else:
            raise ValueError("Direction must be either 'base_to_arm' or 'arm_to_base'.")
