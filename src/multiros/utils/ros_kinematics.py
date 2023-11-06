#!/bin/python3

"""
This class provides kinematics functionality for a robot. Instead of using computation-heavy ROS packages like MoveIt,
this class uses the KDL library to perform kinematics calculations.

With this class, you can calculate,
- Forward kinematics - compute the pose of the end-effector given the joint angles
- Inverse kinematics - compute the joint angles given the pose of the end-effector
"""

import numpy as np
# from urdf_parser_py.urdf import URDF
# from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
# from pykdl_utils.kdl_kinematics import KDLKinematics
from tf.transformations import euler_from_matrix  # Import euler_from_matrix instead of quaternion_from_euler

import PyKDL as kdl
import copy
import tf
from kdl_parser_py.urdf import treeFromParam
from trac_ik_python import trac_ik
import rospy


class Kinematics(object):
    """
    This is modified from the Kinematics class in the pyrobot repo.
    link: https://github.com/facebookresearch/pyrobot/blob/b334b60842271d9d8f4ed7a97bc4e5efe8bb72d6/pyrobot_bridge/nodes/kinematics.py
    """

    def __init__(self, robot_description_parm: str, base_link: str, end_link: str, debug=False):
        """
        Initialize the Kinematics class.

        Args:
            robot_description_parm: robot description parameter name including the namespace
            base_link: base link of the robot including the namespace
            end_link: end-effector link of the robot including the namespace
            debug: debug flag
        """
        # Validate inputs
        if not robot_description_parm or not base_link or not end_link:
            raise ValueError("Inputs robot_description, base_link, and end_link must be non-empty strings.")

        # robot description from parameter server
        robot_description = rospy.get_param(param_name=robot_description_parm)

        # get robot urdf from parameter server
        # robot_urdf = URDF.from_parameter_server(key=self._robot_description_parm_name)

        # create ik solver
        self.num_ik_solver = trac_ik.IK(base_link=base_link, tip_link=end_link, urdf_string=robot_description)

        # get the kinematic tree from the robot urdf
        _, self.urdf_tree = treeFromParam(param=robot_description_parm)

        # get the kinematic chain from the kinematic tree
        self.urdf_chain = self.urdf_tree.getChain(chain_root=base_link, chain_tip=end_link)

        # get the joint names from the kinematic chain
        self.arm_joint_names = self._get_kdl_joint_names()
        if debug:
            print("arm_joint_names:", self.arm_joint_names)

        # get the link names from the kinematic chain
        self.arm_link_names = self._get_kdl_link_names()
        if debug:
            print("arm_link_names:", self.arm_link_names)

        # get the number of joints in the kinematic chain
        self.arm_dof = len(self.arm_joint_names)
        if debug:
            print("arm_dof:", self.arm_dof)

        # compute the Jacobian matrix.
        # represents how the velocity of the end-effector is related to the velocities of the joints
        self.jac_solver = kdl.ChainJntToJacSolver(self.urdf_chain)

        # compute the forward kinematics - position
        self.fk_solver_pos = kdl.ChainFkSolverPos_recursive(self.urdf_chain)

        # compute the forward kinematics - velocity
        self.fk_solver_vel = kdl.ChainFkSolverVel_recursive(self.urdf_chain)

    def _get_kdl_link_names(self):
        """
        Get the link names from the kinematic chain.
        """

        # get the number of links in the kinematic chain - the rigid bodies
        num_links = self.urdf_chain.getNrOfSegments()

        # get the link names from the kinematic chain
        link_names = []
        for i in range(num_links):
            link_names.append(self.urdf_chain.getSegment(i).getName())
        return copy.deepcopy(link_names)

    def _get_kdl_joint_names(self):
        """
        Get the joint names from the kinematic chain.
        """

        # same as above
        num_links = self.urdf_chain.getNrOfSegments()

        # get the joint names from the kinematic chain
        num_joints = self.urdf_chain.getNrOfJoints()  # for assertion

        # get the joint names from the kinematic chain
        joint_names = []
        for i in range(num_links):
            link = self.urdf_chain.getSegment(i)
            joint = link.getJoint()
            joint_type = joint.getType()  # types 0 and 1 represent rotational and translational joints, respectively.
            # JointType definition: [RotAxis,RotX,RotY,RotZ,TransAxis,
            #                        TransX,TransY,TransZ,None]
            if joint_type > 1:
                continue
            joint_names.append(joint.getName())
        assert num_joints == len(joint_names)
        return copy.deepcopy(joint_names)

    def calculate_ik(self, target_pose, tolerance, init_joint_positions):
        """
        Calculate the inverse kinematics for a given pose.

        Args:
            target_pose (list): The desired position and orientation of the end effector given as [x, y, z, qx, qy, qz, qw]
            tolerance (list): The tolerance for each of the pose components
            init_joint_positions (list): The initial positions of the joints from which to start the IK calculation

        Returns:
            A tuple (success, joint_positions), where `success` is a boolean indicating success of IK,
            and `joint_positions` are the calculated joint angles.
        """

        # Validate inputs
        if len(target_pose) < 7 or len(tolerance) < 6 or len(init_joint_positions) != self.arm_dof:
            rospy.logerr("Incorrect IK parameters. Please fix them.")
            return False, None

        # Calculate IK
        joint_positions_IK = self.num_ik_solver.get_ik(
            init_joint_positions,
            target_pose[0], target_pose[1], target_pose[2],
            target_pose[3], target_pose[4], target_pose[5], target_pose[6],
            tolerance[0], tolerance[1], tolerance[2],
            tolerance[3], tolerance[4], tolerance[5],
        )

        # Check if a valid joint position list is returned
        if joint_positions_IK is None:
            rospy.logwarn("Failed to find an IK solution.")
            return False, None

        # Return the joint positions
        return True, joint_positions_IK

    def _kdl_frame_to_numpy(self, frame):
        """
        Convert the KDL frame to a numpy array.

        Args:
            frame: KDL frame
        """
        p = frame.p
        M = frame.M
        return np.array(
            [
                [M[0, 0], M[0, 1], M[0, 2], p.x()],
                [M[1, 0], M[1, 1], M[1, 2], p.y()],
                [M[2, 0], M[2, 1], M[2, 2], p.z()],
                [0, 0, 0, 1],
            ]
        )

    @staticmethod
    def rot_mat_to_quat(rot):
        """
        Convert the rotation matrix into quaternion.

        Args:
            rot (numpy.ndarray): the rotation matrix (shape: :math:`[3, 3]`)
        Returns:
            quaternion (numpy.ndarray) [x, y, z, w] (shape: :math:`[4,]`)
        """
        R = np.eye(4)
        R[:3, :3] = rot
        return tf.transformations.quaternion_from_matrix(R)

    @staticmethod
    def joints_to_kdl(joint_values):
        """
        Convert the numpy array into KDL data format

        Args:
            joint_values: values for the joints
        Returns:
            kdl data type for the joints
        """
        num_jts = joint_values.size
        kdl_array = kdl.JntArray(num_jts)
        for idx in range(num_jts):
            kdl_array[idx] = joint_values[idx]
        return kdl_array

    def calculate_fk(self, joint_positions, des_frame, euler=True):
        """
        Given joint angles, compute the pose of desired_frame with respect
        to the base frame. The desired frame must be in self.arm_link_names.

        Args:
            joint_positions (np.ndarray): Joint angles.
            des_frame (str): Desired frame.
            euler (bool): If True, return the orientation in Euler angles.

        Returns:
            A tuple (success, pos, rpy), where `success` is a boolean indicating success of FK,
            and `pos` are the calculated position and `rpy` are the calculated roll, pitch, yaw.
        """
        # Validate inputs
        if not isinstance(joint_positions, np.ndarray):
            raise ValueError("joint_positions must be of type np.ndarray")
        if des_frame not in self.arm_link_names:
            raise ValueError("des_frame must be one of the configured link names")
        if joint_positions.size != self.arm_dof:
            raise ValueError("Invalid length of joint angles! Expected length: {}".format(self.arm_dof))

        # covert joint positions to kdl data type
        kdl_jnt_angles = self.joints_to_kdl(joint_positions)

        # Create a KDL frame to hold the result
        kdl_end_frame = kdl.Frame()

        # Get the index of the desired frame within the robot's link names
        idx = self.arm_link_names.index(des_frame) + 1

        # Perform the JntToCart calculation to get the pose of the desired frame
        fk_result = self.fk_solver_pos.JntToCart(kdl_jnt_angles, kdl_end_frame, idx)

        # Check for any errors during computation
        if fk_result < 0:
            rospy.logerr("Error computing forward kinematics with KDL.")
            return False, None, None

        # Convert the KDL frame to a NumPy array to extract the position and orientation
        pose = self._kdl_frame_to_numpy(kdl_end_frame)
        pos = pose[:3, 3].reshape(-1, 1)

        if euler:
            # Convert the rotation matrix to roll-pitch-yaw angles
            rotations = euler_from_matrix(pose[:3, :3], 'sxyz')  # 'sxyz' specifies static (fixed) axes
            rotations = np.array(rotations).flatten()
        else:
            # Convert the rotation matrix to a quaternion
            rotations = self.rot_mat_to_quat(pose[:3, :3])

        # Return position and rotations both as 1D arrays
        return True, pos.flatten(), rotations
