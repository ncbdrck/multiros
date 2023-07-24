#! /usr/bin/env python

"""
This Class is to specify all the common functions related to the handling of ROS Moveit for manipulation tasks.

It has the following methods (Main functions),
    01. set_trajectory_ee: Set a pose target for the end effector of the robot arm.
    02. set_trajectory_joints: Set a joint position target for the arm joints.
    03. set_gripper_joints: Set a joint position target for the gripper joints.
    04. get_robot_pose: Get the current pose of the robot arm.
    05. get_robot_rpy: Get the current roll, pitch, and yaw angles of the robot arm.
    06. get_gripper_pose: Get the current pose of the gripper.
    07. get_gripper_rpy: Get the current roll, pitch, and yaw angles of the gripper.
    08. get_joint_angles_robot_arm: Get the current joint angles of the robot arm.
    09. get_joint_angles_gripper: Get the current joint angles of the gripper.
    10. get_joint_angles: Get the current joint angles of both the robot arm and gripper.
    11. check_goal: Check if a goal position is reachable by the robot arm.
    12. get_randomJointVals: Get random joint values for the robot arm.
    13. get_randomPose: Get a random pose for the robot arm.
    14. set_planning_time: Set the maximum time allowed for planning a trajectory.
    15. set_goal_position_tolerance: Set the goal position tolerance for the robot arm.
    16. set_goal_orientation_tolerance: Set the goal orientation tolerance for the robot arm.
    17. set_goal_joint_tolerance: Set the goal joint tolerance for the robot arm.
    18. set_max_acceleration_scaling_factor: Set the maximum acceleration scaling factor for the robot arm.
    19. set_max_velocity_scaling_factor: Set the maximum velocity scaling factor for the robot arm.

"""
import sys
import moveit_commander
import rospy
from multiros.utils import gazebo_core
from typing import List, Union
import numpy as np
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped


class MoveitMultiros(object):
    """
    A class for using MoveIt!

    This class provides a set of methods for controlling a robot arm and gripper using MoveIt!.
    It allows you to set joint angles, execute trajectories, and check if a goal is reachable.
    It also provides methods for getting the current pose and joint angles of the robot arm and gripper.

    To use this class, you must first initialize it with the appropriate parameters for your robot arm and gripper.
    You can then call the various methods provided by this class to control your robot.
    """

    def __init__(self, arm_name: str, gripper_name: str = None, robot_description: str = None, ns: str = None):

        rospy.loginfo("Initializing MoveitMultiros")

        # Initialize MoveIt! commander
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize RobotCommander, PlanningSceneInterface and MoveGroupCommander objects
        if robot_description is not None and ns is not None:

            self.robot_description = robot_description
            self.ns = ns

            # Instantiate a RobotCommander object with robot_description and namespace
            self.robot = moveit_commander.RobotCommander(robot_description=robot_description,
                                                         ns=ns)

            # Instantiate a PlanningSceneInterface object with namespace
            self.scene = moveit_commander.PlanningSceneInterface(ns=ns)

            # Instantiate a MoveGroupCommander object with robot_arm_name, robot_description and namespace
            self.robot_arm_name = arm_name
            self.robot_arm = moveit_commander.MoveGroupCommander(name=self.robot_arm_name,
                                                                 robot_description=robot_description,
                                                                 ns=ns)
        else:
            # Instantiate a RobotCommander object
            self.robot = moveit_commander.RobotCommander()

            # Instantiate a PlanningSceneInterface object
            self.scene = moveit_commander.PlanningSceneInterface()

            # Instantiate a MoveGroupCommander object with robot_arm_name
            self.robot_arm_name = arm_name
            self.robot_arm = moveit_commander.MoveGroupCommander(name=self.robot_arm_name)

        # Initialize Gripper
        if self.robot_arm.has_end_effector_link():
            rospy.loginfo("End-effector link: %s", self.robot_arm.get_end_effector_link())
        else:
            rospy.logwarn(
                "Robot doesn't have an end-effector link! Please add an end-effector in your MoveIt! configuration package.")

        if gripper_name is not None:
            self.gripper_name = gripper_name

            if robot_description is not None and ns is not None:
                self.gripper = moveit_commander.MoveGroupCommander(name=self.gripper_name,
                                                                   robot_description=robot_description,
                                                                   ns=ns)
            else:
                self.gripper = moveit_commander.MoveGroupCommander(name=self.gripper_name)
        else:
            rospy.loginfo("No gripper is added")

        rospy.loginfo("MoveitMultiros initialization complete")

    """
        helper fns
    """

    def arm_execute_pose(self, pose: Pose) -> bool:
        """
        Execute a pose with the robot arm.

        Args:
            pose (Pose): The target pose for the robot arm.

        Returns:
            bool: The result of the trajectory execution.
        """
        gazebo_core.unpause_gazebo()

        # Set the pose target for the robot arm
        self.robot_arm.set_pose_target(pose)

        # Execute the trajectory to reach the target pose
        result = self.arm_execute_trajectory()

        # Clear the pose targets
        self.robot_arm.clear_pose_targets()

        gazebo_core.pause_gazebo()
        return result

    def arm_execute_joint_trajectory(self, joint_target_values: List[float]) -> bool:
        """
        Execute a joint trajectory with the robot arm.

        Args:
            joint_target_values (List[float]): The target joint values for the robot arm.

        Returns:
            bool: The result of the trajectory execution.

        Raises:
            ValueError: If the input joint target values do not match the number of joints of the robot arm.
        """
        # Get the current joint angles of the robot arm
        joint_goal = self.get_joint_angles_robot_arm()

        # Check if the input joint target values match the number of joints of the robot arm
        if len(joint_target_values) != len(joint_goal):
            raise ValueError("The input joint target values do not match the number of joints of the robot arm.")

        # Set the joint target values
        for i in range(len(joint_target_values)):
            joint_goal[i] = joint_target_values[i]

        gazebo_core.unpause_gazebo()

        # Set the joint value target for the robot arm
        self.robot_arm.set_joint_value_target(joint_goal)

        # Execute the trajectory to reach the target joint values
        result = self.arm_execute_trajectory()

        gazebo_core.pause_gazebo()

        return result

    def gripper_execute_joint_command(self, target_joint_values: List[float]) -> bool:
        """
        Execute a joint command with the gripper.

        Args:
            target_joint_values (List[float]): The target joint values for the gripper.

        Returns:
            bool: The result of the command execution.

        Raises:
            ValueError: If the input joint target values do not match the number of joints of the gripper.
        """
        # Get the current joint angles of the gripper
        gripper_joints = self.get_joint_angles_gripper()

        # Check if the input joint target values match the number of joints of the gripper
        if len(target_joint_values) != len(gripper_joints):
            raise ValueError("The input joint target values do not match the number of joints of the gripper.")

        # Set the joint target values
        for i in range(len(target_joint_values)):
            gripper_joints[i] = float(target_joint_values[i])

        gazebo_core.unpause_gazebo()

        # Move the gripper to the target joint values
        result = self.gripper.go(gripper_joints, wait=True)

        # Stop the gripper from moving
        self.gripper.stop()

        gazebo_core.pause_gazebo()

        return result

    def arm_execute_trajectory(self) -> bool:
        """
        Execute a trajectory with the robot arm.

        Returns:
            bool: The result of the trajectory execution.
        """
        # Plan and execute a trajectory to reach a previously set target
        # plan = self.robot_arm.plan()
        # result = self.robot_arm.execute(plan, wait=True)

        # Execute the trajectory to reach the target
        result = self.robot_arm.go(wait=True)

        # Stop the robot arm from moving
        self.robot_arm.stop()

        return result

    def robot_pose(self):
        """
        Get the current pose of the robot arm.

        Returns:
            PoseStamped: The current pose of the robot arm.
        """
        return self.robot_arm.get_current_pose()

    def robot_rpy(self) -> List[float]:
        """
        Get the current roll, pitch, and yaw angles of the robot arm.

        Returns:
            List[float]: The current roll, pitch, and yaw angles of the robot arm.
        """
        return self.robot_arm.get_current_rpy()

    def gripper_pose(self, gripper_link: str = ""):
        """
        Get the current pose of the gripper.

        Args:
            gripper_link (str): The name of the gripper link to get the pose of.

        Returns:
            PoseStamped: The current pose of the gripper.
        """
        return self.gripper.get_current_pose(end_effector_link=gripper_link)

    def gripper_rpy(self, gripper_link: str = "") -> List[float]:
        """
        Get the current roll, pitch, and yaw angles of the gripper.

        Args:
            gripper_link (str): The name of the gripper link to get the angles of.

        Returns:
            List[float]: The current roll, pitch, and yaw angles of the gripper.
        """
        return self.gripper.get_current_rpy(end_effector_link=gripper_link)

    def joint_angles_arm(self) -> List[float]:
        """
        Get the current joint angles of the robot arm.

        Returns:
            List[float]: The current joint angles of the robot arm.
        """
        return self.robot_arm.get_current_joint_values()

    def joint_angles_gripper(self) -> List[float]:
        """
        Get the current joint angles of the gripper.

        Returns:
            List[float]: The current joint angles of the gripper.
        """
        return self.gripper.get_current_joint_values()

    def joint_angles(self) -> List[float]:
        """
        Get the current joint angles of both the robot arm and gripper.

        Returns:
            List[float]: The current joint angles of both the robot arm and gripper.
        """
        # Merge and return joint angles for both arm and gripper
        return self.joint_angles_arm() + self.joint_angles_gripper()

    def is_goal_reachable(self, goal: Union[List[float], np.ndarray]) -> bool:
        """
        Check if a goal position is reachable by the robot arm.

        Args:
            goal (Union[List[float], np.ndarray]): The target position for the robot arm.

        Returns:
            bool: Whether the goal position is reachable or not.
        """
        # Convert goal to list if it is a numpy array
        if isinstance(goal, np.ndarray):
            goal = goal.tolist()

        # Set the position target of the robot arm to the goal
        self.robot_arm.set_position_target(goal)

        # Plan a trajectory to reach the goal
        plan = self.robot_arm.plan()

        # Get the result of the plan
        result = plan[0]

        # Clear the pose targets
        self.robot_arm.clear_pose_targets()

        return result

    """
            Main functions we can call from the object
    """

    def set_trajectory_ee(self, position: Union[List[float], np.ndarray],
                          orientation: Union[List[float], np.ndarray] = None) -> bool:
        """
        Set a pose target for the end effector of the robot arm.

        Args:
            position (Union[List[float], np.ndarray]): The target position for the end effector.
            orientation (Union[List[float], np.ndarray]): The target orientation for the end effector (Optional).

        Returns:
            bool: The result of the trajectory execution.
        """

        # Convert action to list if it is a numpy array
        if isinstance(position, np.ndarray):
            position = position.tolist()

        # Set up a trajectory message to publish.
        ee_target = Pose()
        ee_target.orientation.x = 1e-6
        ee_target.orientation.y = 1e-6
        ee_target.orientation.z = 1e-6
        ee_target.orientation.w = 1.000000
        ee_target.position.x = position[0]
        ee_target.position.y = position[1]
        ee_target.position.z = position[2]

        if orientation is not None:
            if isinstance(orientation, np.ndarray):
                orientation = orientation.tolist()

            ee_target.orientation.x = orientation[0]
            ee_target.orientation.y = orientation[1]
            ee_target.orientation.z = orientation[2]
            ee_target.orientation.w = orientation[3]

        # Execute the trajectory to move the end effector to the desired position and orientation
        return self.arm_execute_pose(ee_target)

    def set_trajectory_joints(self, q_positions: Union[List[float], np.ndarray]) -> bool:
        """
        Set a joint position target for the arm joints.

        Args:
            q_positions (Union[List[float], np.ndarray]): The target joint positions for the robot arm.

        Returns:
            bool: The result of the trajectory execution.
        """
        # Convert q_positions to list if it is a numpy array
        if isinstance(q_positions, np.ndarray):
            q_positions = q_positions.tolist()

        # Execute a joint trajectory to move the arm to the desired joint positions
        return self.arm_execute_joint_trajectory(q_positions)

    def set_gripper_joints(self, joint_positions: Union[List[float], np.ndarray]) -> bool:
        """
        Set a joint position target for the gripper joints.

        Args:
            joint_positions (Union[List[float], np.ndarray]): The target joint positions for the gripper.

        Returns:
            bool: The result of the command execution.
        """
        # Convert joint_positions to list if it is a numpy array
        if isinstance(joint_positions, np.ndarray):
            joint_positions = joint_positions.tolist()

        # Execute a joint command to move the gripper to the desired joint positions
        return self.gripper_execute_joint_command(joint_positions)

    def get_robot_pose(self) -> PoseStamped:
        """
        Get the current pose of the robot arm.

        Returns:
            PoseStamped: The current pose of the robot arm.
        """
        gazebo_core.unpause_gazebo()
        robot_pose = self.robot_pose()
        gazebo_core.pause_gazebo()
        return robot_pose

    def get_robot_rpy(self) -> List[float]:
        """
        Get the current roll, pitch, and yaw angles of the robot arm.

        Returns:
            List[float]: The current roll, pitch, and yaw angles of the robot arm.
        """
        gazebo_core.unpause_gazebo()
        robot_rpy = self.robot_rpy()
        gazebo_core.pause_gazebo()
        return robot_rpy

    def get_gripper_pose(self, link: str = "") -> PoseStamped:
        """
        Get the current pose of the gripper.

        Args:
            link (str): The name of the gripper link to get the pose of.

        Returns:
            PoseStamped: The current pose of the gripper.
        """
        gazebo_core.unpause_gazebo()
        gripper_pose = self.gripper_pose(gripper_link=link)
        gazebo_core.pause_gazebo()
        return gripper_pose

    def get_gripper_rpy(self, link: str = "") -> List[float]:
        """
        Get the current roll, pitch, and yaw angles of the gripper.

        Args:
            link (str): The name of the gripper link to get the angles of.

        Returns:
            List[float]: The current roll, pitch, and yaw angles of the gripper.
        """
        gazebo_core.unpause_gazebo()
        gripper_rpy = self.gripper_rpy(gripper_link=link)
        gazebo_core.pause_gazebo()
        return gripper_rpy

    def get_joint_angles_robot_arm(self) -> List[float]:
        """
        Get the current joint angles of the robot arm.

        Returns:
            List[float]: The current joint angles of the robot arm.
        """
        gazebo_core.unpause_gazebo()
        robot_joint_angles = self.joint_angles_arm()
        gazebo_core.pause_gazebo()
        return robot_joint_angles

    def get_joint_angles_gripper(self) -> List[float]:
        """
        Get the current joint angles of the gripper.

        Returns:
            List[float]: The current joint angles of the gripper.
        """
        gazebo_core.unpause_gazebo()
        gripper_joint_angles = self.joint_angles_gripper()
        gazebo_core.pause_gazebo()
        return gripper_joint_angles

    def get_joint_angles(self) -> List[float]:
        """
        Get the current joint angles of both the robot arm and gripper.

        Returns:
            List[float]: The current joint angles of both the robot arm and gripper.
        """
        gazebo_core.unpause_gazebo()
        joint_angles = self.joint_angles()
        gazebo_core.pause_gazebo()
        return joint_angles

    def check_goal(self, goal: Union[List[float], np.ndarray]) -> bool:
        """

        Check if a goal position is reachable by the robot arm.

        Args:
            goal (Union[List[float], np.ndarray]): The target position for the robot arm.

        Returns:
            bool: Whether the goal position is reachable or not.
        """
        return self.is_goal_reachable(goal)

    def get_randomJointVals(self) -> List[float]:
        """
        Get random joint values for the robot arm.

        Returns:
            List[float]: Random joint values for the robot arm.
        """
        return self.robot_arm.get_random_joint_values()

    def get_randomPose(self) -> PoseStamped:
        """
        Get a random pose for the robot arm.

        Returns:
            PoseStamped: A random pose for the robot arm.
        """
        gazebo_core.unpause_gazebo()
        random_pose = self.robot_arm.get_random_pose()
        gazebo_core.pause_gazebo()
        return random_pose

    def set_planning_time(self, planning_time: float):
        """
        Set the maximum allowed planning time. Specify the amount of time to be used for motion planning

        Args:
            planning_time (float): The maximum allowed planning time in seconds.
        """
        self.robot_arm.set_planning_time(planning_time)

    def set_goal_position_tolerance(self, position_tolerance: float):
        """
        Set the tolerance for a target end-effector position

        Args:
            position_tolerance (float): The tolerance for the goal position in meters.
        """
        self.robot_arm.set_goal_position_tolerance(position_tolerance)

    def set_goal_orientation_tolerance(self, orientation_tolerance: float):
        """
        Set the tolerance for a target end-effector orientation.

        Args:
            orientation_tolerance (float): The tolerance for the goal orientation in radians.
        """
        self.robot_arm.set_goal_orientation_tolerance(orientation_tolerance)

    def set_goal_joint_tolerance(self, joint_tolerance: float):
        """
        Set the tolerance for a target joint configuration.

        Args:
            joint_tolerance (float): The tolerance for the goal joint configuration in radians.
        """
        self.robot_arm.set_goal_joint_tolerance(joint_tolerance)

    def set_max_acceleration_scaling_factor(self, acceleration_scaling_factor: float):
        """
        Set a scaling factor to reduce the maximum joint accelerations. Allowed values are in (0,1].
        The default value is set in the joint_limits.yaml of the moveit_config package.

        Args:
            acceleration_scaling_factor (float): The maximum acceleration scaling factor.
        """
        self.robot_arm.set_max_acceleration_scaling_factor(acceleration_scaling_factor)

    def set_max_velocity_scaling_factor(self, velocity_scaling_factor: float):
        """
        Set a scaling factor to reduce the maximum joint velocities. Allowed values are in (0,1].
        The default value is set in the joint_limits.yaml of the moveit_config package.

        Args:
            velocity_scaling_factor (float): The maximum velocity scaling factor.
        """
        self.robot_arm.set_max_velocity_scaling_factor(velocity_scaling_factor)

    def set_trajectory_cartesian(self, waypoints: List[PoseStamped], eef_step: float = 0.01,
                                 jump_threshold: float = 0.0,
                                 avoid_collisions: bool = True) -> bool:
        """
        Set a cartesian trajectory for the end effector of the robot arm.

        Args:
            waypoints (List[PoseStamped]): The target waypoints for the end effector.
            eef_step (float): The distance between waypoints in meters (Optional).
            jump_threshold (float): The maximum distance in the configuration space between consecutive points in the resulting path (Optional).
            avoid_collisions (bool): Whether to check for collisions between waypoints or not (Optional).

        Returns:
            bool: The result of the trajectory execution.

        """
        # Set the waypoints for the trajectory
        self.robot_arm.set_pose_targets(waypoints)

        # unpause gazebo
        gazebo_core.unpause_gazebo()

        # Plan a cartesian path to move the end effector through the waypoints
        plan, fraction = self.robot_arm.compute_cartesian_path(waypoints, eef_step, jump_threshold, avoid_collisions)

        # Execute the trajectory to move the end effector through the waypoints
        result = self.robot_arm.execute(plan, wait=True)

        # Stop the robot arm from moving
        self.robot_arm.stop()

        # Clear the pose targets
        self.robot_arm.clear_pose_targets()

        # pause gazebo
        gazebo_core.unpause_gazebo()

        return result






