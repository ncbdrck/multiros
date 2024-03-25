#!/bin/python3

import rospy
import gymnasium as gym
from multiros.utils import gazebo_core
from multiros.utils import gazebo_models
from multiros.utils import gazebo_physics
from multiros.utils import ros_common
from multiros.utils import ros_controllers
from typing import Optional, List, Any, Dict


class GazeboBaseEnv(gym.Env):
    """
    A custom gymnasium environment for reinforcement learning using ROS and Gazebo.
    """

    def __init__(self, spawn_robot: bool = False, urdf_pkg_name: str = None, urdf_file_name: str = None,
                 urdf_folder: str = "/urdf", urdf_xacro_args: List[str] = None, namespace: str = "/",
                 robot_state_publisher_max_freq: float = None, new_robot_state_term: bool = False,
                 robot_model_name: str = "robot", robot_ref_frame: str = "world",
                 robot_pos_x: float = 0.0, robot_pos_y: float = 0.0, robot_pos_z: float = 0.0, robot_ori_w: float = 0.0,
                 robot_ori_x: float = 0.0, robot_ori_y: float = 0.0, robot_ori_z: float = 0.0,
                 controllers_file: str = None, controllers_list: List[str] = None,
                 reset_controllers: bool = False, reset_mode: str = "world", sim_step_mode: int = 1,
                 num_gazebo_steps: int = 1, gazebo_max_update_rate: float = None, gazebo_timestep: float = None,
                 kill_rosmaster: bool = True, kill_gazebo: bool = True, clean_logs: bool = False,
                 ros_port: str = None, gazebo_port: str = None, gazebo_pid=None, seed: int = None,
                 unpause_pause_physics: bool = True, action_cycle_time: float = 0.0):

        """
        Initialize the GazeboBaseEnv.

        Args:
            spawn_robot (bool): Whether to spawn the robot in Gazebo.
            urdf_pkg_name (str): The name of the URDF package.
            urdf_file_name (str): The name of the URDF file.
            urdf_folder (str): The folder containing the URDF file.
            urdf_xacro_args (List[str]): The arguments for the xacro processor.
            namespace (str): The ROS namespace for the robot.
            robot_state_publisher_max_freq (float): The maximum frequency for the robot state publisher.
            new_robot_state_term (bool): Whether to use a new terminal for the robot state publisher.
            robot_model_name (str): The name of the robot model in Gazebo.
            robot_ref_frame (str): The reference frame for the robot in Gazebo.
            robot_pos_x (float): The x position of the robot in Gazebo.
            robot_pos_y (float): The y position of the robot in Gazebo.
            robot_pos_z (float): The z position of the robot in Gazebo.
            robot_ori_w (float): The w orientation of the robot in Gazebo.
            robot_ori_x (float): The x orientation of the robot in Gazebo.
            robot_ori_y (float): The y orientation of the robot in Gazebo.
            robot_ori_z (float): The z orientation of the robot in Gazebo.
            controllers_file (str): The file containing the controller configurations.
            controllers_list (List[str]): The list of ROS controllers to use.
            reset_controllers (bool): Whether to reset the controllers on reset.
            reset_mode (str): The mode to use when resetting Gazebo ("world" or "simulation").
            sim_step_mode (int): The mode to use when stepping the simulation (1 or 2).
            num_gazebo_steps (int): The number of Gazebo steps to take per step call.
            gazebo_max_update_rate (float): The maximum update rate for Gazebo.
            gazebo_timestep (float): The time step for Gazebo.
            kill_rosmaster (bool): Whether to kill the ROS master on close.
            kill_gazebo (bool): Whether to kill Gazebo on close.
            clean_logs (bool): Whether to clean the ROS logs on close.
            ros_port (str): The ROS_MASTER_URI port.
            gazebo_port (str): The GAZEBO_MASTER_URI port.
            gazebo_pid (subprocess.Popen): A subprocess.Popen object representing the running Gazebo instance
            seed (int): Seed for random number generator
            unpause_pause_physics (bool): Whether to unpause and pause Gazebo before and after each step call.
            action_cycle_time (float): The time to wait between applying actions.

        """

        """
        For logging in different colours
        Define ANSI escape codes for different colors
        """

        self.RED = '\033[91m'
        self.GREEN = '\033[92m'
        self.YELLOW = '\033[93m'
        self.BLUE = '\033[94m'
        self.MAGENTA = '\033[95m'
        self.CYAN = '\033[96m'
        self.ENDC = '\033[0m'

        rospy.loginfo(self.CYAN + "Start init GazeboBaseEnv!" + self.ENDC)

        """
        Initialize the variables
        """
        self.ros_port = ros_port
        self.gazebo_port = gazebo_port
        self.gazebo_pid = gazebo_pid
        self.user_seed = seed
        self.unpause_pause_physics = unpause_pause_physics
        self.action_cycle_time = action_cycle_time

        self.info = {}
        self.terminated = None
        self.truncated = None
        self.reward = 0.0
        self.observation = None

        # --------- Change the ros/gazebo master
        # defined in Task Env
        if self.ros_port is not None:
            ros_common.change_ros_gazebo_master(ros_port=self.ros_port, gazebo_port=self.gazebo_port)

        """
        Function to initialise the environment.
        """

        # Init gymnasium.Env
        super().__init__()

        self.namespace = namespace
        self.reset_controllers = reset_controllers
        self.controllers_list = controllers_list
        self.reset_mode = reset_mode
        self.sim_step_mode = sim_step_mode
        self.num_gazebo_steps = num_gazebo_steps
        self.kill_rosmaster = kill_rosmaster
        self.kill_gazebo = kill_gazebo
        self.clean_logs = clean_logs

        """
        Set gazebo physics parameters to change the speed of the simulation
        """

        if gazebo_max_update_rate is not None:
            gazebo_physics.set_gazebo_max_update_rate(real_time_factor=gazebo_max_update_rate)

        if gazebo_timestep is not None:
            gazebo_physics.set_gazebo_time_step(time_step=gazebo_timestep)

        """
        Spawn the robot in Gazebo
        """
        # If spawn robot, spawn it
        if spawn_robot:
            gazebo_models.spawn_robot_in_gazebo(pkg_name=urdf_pkg_name, model_urdf_file=urdf_file_name,
                                                model_urdf_folder=urdf_folder, ns=self.namespace,
                                                args_xacro=urdf_xacro_args, pub_freq=robot_state_publisher_max_freq,
                                                rob_st_term=new_robot_state_term, gazebo_name=robot_model_name,
                                                gz_ref_frame=robot_ref_frame,
                                                pos_x=robot_pos_x, pos_y=robot_pos_y, pos_z=robot_pos_z,
                                                ori_w=robot_ori_w, ori_x=robot_ori_x, ori_y=robot_ori_y,
                                                ori_z=robot_ori_z, controllers_file=controllers_file,
                                                controllers_list=self.controllers_list)

        """
        Reset the controllers
        """
        if self.reset_controllers:
            if self.unpause_pause_physics:
                gazebo_core.unpause_gazebo()

            ros_controllers.reset_controllers(controller_list=self.controllers_list, ns=self.namespace)

            if self.unpause_pause_physics:
                gazebo_core.pause_gazebo()

        rospy.loginfo(self.CYAN + "End init GazeboBaseEnv" + self.ENDC)

    def step(self, action):
        """
        Take a step in the environment.

        Args:
            action (Any): The action to be applied to the robot.

        Returns:
            observation (Any): The observation representing the current state of the environment.
            reward (float): The reward for taking the given action.
            terminated (bool): Whether the agent reaches the terminal state.
            truncated (bool): Whether the episode is truncated due to various reasons.
            (e.g. reaching the maximum number of steps, or end before the terminal state)
            info (dict): Additional information about the environment.
        """

        # ----- Change the ros/gazebo master

        if self.ros_port is not None:
            ros_common.change_ros_gazebo_master(ros_port=self.ros_port, gazebo_port=self.gazebo_port)

        # ----- Start the step env
        # rospy.loginfo(self.MAGENTA + "*************** Started Step Env" + self.ENDC)

        # Unpause Gazebo, apply the action, pause Gazebo
        if self.sim_step_mode == 1:
            if self.unpause_pause_physics:
                gazebo_core.unpause_gazebo()

            self._set_action(action)

            if self.unpause_pause_physics:
                gazebo_core.pause_gazebo()

        # If using the gazebo step command
        elif self.sim_step_mode == 2:
            self._set_action(action)
            gazebo_core.gazebo_step(steps=self.num_gazebo_steps)

        # If using the action cycle time
        if self.action_cycle_time > 0.0:
            rospy.sleep(self.action_cycle_time)

        # Get the observation, reward and terminated, truncated flags
        self.info = {}
        self.observation = self._get_observation()
        self.reward = self._get_reward(info=self.info)
        self.terminated = self._compute_terminated(info=self.info)
        self.truncated = self._compute_truncated(info=self.info)

        # rospy.loginfo(self.MAGENTA + "*************** End Step Env" + self.ENDC)

        return self.observation, self.reward, self.terminated, self.truncated, self.info

    def reset(self, seed: Optional[int] = None, options: Optional[Dict[str, Any]] = None):

        """
        Reset the environment.

        Args:
            seed (int): The seed for the random number generator.
            options (dict): Additional information for resetting the environment.

        Returns:
            observation (Any): The initial observation representing the state of the environment.
            info (dict): Additional information about the environment. Similar to the info returned by step().
        """

        # set the seed (standard way to set the seed in gymnasium)
        if self.user_seed is not None:
            super().reset(seed=self.user_seed)
        else:
            super().reset(seed=seed)

        # reinitialize the info dictionary
        self.info = {}

        # ----- Change the ros/gazebo master
        if self.ros_port is not None:
            ros_common.change_ros_gazebo_master(ros_port=self.ros_port, gazebo_port=self.gazebo_port)

        # ----- Reset the env
        rospy.loginfo(self.MAGENTA + "*************** Start Reset Env" + self.ENDC)

        # Reset the Gazebo and get the initial observation
        self._reset_gazebo(options=options)
        self.observation = self._get_observation()

        rospy.loginfo(self.MAGENTA + "*************** End Reset Env" + self.ENDC)

        return self.observation, self.info

    def close(self):
        """
        Close the environment.
        """

        # ----- Change the ros/gazebo master
        if self.ros_port is not None:
            ros_common.change_ros_gazebo_master(ros_port=self.ros_port, gazebo_port=self.gazebo_port)

        rospy.loginfo(self.CYAN + "*************** Start Closing Env" + self.ENDC)

        # Shutdown the ROS node
        rospy.signal_shutdown("Closing Environment")

        if self.kill_rosmaster:
            if self.ros_port is not None:
                ros_common.ros_kill_master(ros_port=self.ros_port)
                rospy.loginfo("Killed ROS Master!")

        if self.kill_gazebo:
            if self.gazebo_pid is not None:
                gazebo_core.close_gazebo(process=self.gazebo_pid)
                rospy.loginfo("Closed Gazebo!")

        if self.clean_logs:
            ros_common.clean_ros_logs()
            rospy.loginfo("Cleaned ROS Logs!")

        rospy.loginfo(self.CYAN + "*************** Closed Env" + self.ENDC)

    # ---------------------------------------------
    #   Methods to override in CustomTaskEnv

    def _set_action(self, action):
        """
        Function to apply an action to the robot.

        This method should be implemented by subclasses to apply the given action to the robot. The action could be a
        joint position command, a velocity command, or any other type of command that can be applied to the robot.

        Args:
            action: The action to be applied to the robot.
        """
        raise NotImplementedError()

    def _get_observation(self):
        """
        Function to get an observation from the environment.

        This method should be implemented by subclasses to return an observation representing the current state of
        the environment. The observation could be a sensor reading, a joint state, or any other type of observation
        that can be obtained from the environment.

        Returns:
            An observation representing the current state of the environment.
        """
        raise NotImplementedError()

    def _get_reward(self, info: Optional[Dict[str, Any]] = None):
        """
        Function to get a reward from the environment.

        This method should be implemented by subclasses to return a scalar reward value representing how well the agent
        is doing in the current episode. The reward could be based on the distance to a goal, the amount of time taken
        to reach a goal, or any other metric that can be used to measure how well the agent is doing.

        Args:
            info (dict): Additional information for computing the reward.

        Returns:
            A scalar reward value representing how well the agent is doing in the current episode.
        """
        raise NotImplementedError()

    def _compute_terminated(self, info: Optional[Dict[str, Any]] = None):
        """
        Function to check if the episode is terminated due to reaching a terminal state.

        This method should be implemented by subclasses to return a boolean value indicating whether the episode has
        ended (e.g., because a goal has been reached or a failure condition has been triggered).

        Args:
            info (dict): Additional information for computing the termination condition.

        Returns:
            A boolean value indicating whether the episode has ended
            (e.g., because a goal has been reached or a failure condition has been triggered)
        """
        raise NotImplementedError()

    def _compute_truncated(self, info: Optional[Dict[str, Any]] = None):
        """
        Function to check if the episode is truncated due non-terminal reasons.

        This method should be implemented by subclasses to return a boolean value indicating whether the episode has
        been truncated due to reasons other than reaching a terminal state.
        Truncated states are those that are out of the scope of the Markov Decision Process (MDP).
        This could include truncation due to reaching a maximum number of steps, or any other non-terminal condition
        that causes the episode to end early.

        Args:
            info (dict): Additional information for computing the truncation condition.

        Returns:
            A boolean value indicating whether the episode has been truncated.
        """
        raise NotImplementedError()

    def _set_init_params(self, options: Optional[Dict[str, Any]] = None):
        """
        Set initial parameters for the environment.

        This method should be implemented by subclasses to set any initial parameters or state variables for the
        environment. This could include resetting joint positions, resetting sensor readings, or any other initial
        setup that needs to be performed at the start of each episode.

        Args:
            options (dict): Additional options for setting the initial parameters.
        """
        raise NotImplementedError()

    # ------------------------------------------
    #   Methods to override in CustomRobotEnv

    def _check_connection_and_readiness(self):
        """
        Function to check the connection status of subscribers, publishers and services, as well as the readiness of
        all systems.
        """
        raise NotImplementedError()

    # ------------------------------------------
    #   Custom methods for the GazeboBaseEnv

    def _reset_gazebo(self, options: Optional[Dict[str, Any]] = None):
        """
        Helper function to reset the Gazebo simulation and the controllers.

        if self.reset_mode is
            "world": Reset the Gazebo world (Does not reset time) - default
            "simulation": Reset gazebo simulation (Resets time)

        Args:
            options (dict): Additional options for resetting the environment.
        """

        # Pause Gazebo and reset it
        if self.unpause_pause_physics:
            gazebo_core.pause_gazebo()
        gazebo_core.reset_gazebo(reset_type=self.reset_mode)

        # Reset the controllers
        if self.reset_controllers:
            if self.unpause_pause_physics:
                gazebo_core.unpause_gazebo()

            ros_controllers.reset_controllers(controller_list=self.controllers_list, ns=self.namespace)

            if self.unpause_pause_physics:
                gazebo_core.pause_gazebo()

        # Unpause Gazebo and check the connection status
        if self.unpause_pause_physics:
            gazebo_core.unpause_gazebo()
        self._check_connection_and_readiness()

        # set initial parameters for the environment
        self._set_init_params(options=options)

        # pause Gazebo
        if self.unpause_pause_physics:
            gazebo_core.pause_gazebo()
