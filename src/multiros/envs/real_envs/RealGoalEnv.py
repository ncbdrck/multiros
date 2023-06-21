#!/bin/python3

import rospy
import gym
from gym.utils import seeding
from multiros.utils import ros_common
from multiros.utils import ros_controllers
from typing import List
import time


class RealGoalEnv(gym.GoalEnv):
    """
    A custom OpenAI Gym environment for reinforcement learning using ROS and real robots.

    More suited for Goal Envs
    """

    def __init__(self, load_robot: bool = True, robot_pkg_name: str = None, robot_launch_file: str = None,
                 robot_args: List[str] = None, load_urdf: bool = False, urdf_pkg_name: str = None,
                 urdf_file_name: str = None, urdf_folder: str = "/urdf", urdf_xacro_args: List[str] = None,
                 namespace: str = "/", launch_robot_state_pub: bool = False,
                 robot_state_publisher_max_freq: float = None, new_robot_state_term: bool = False,
                 load_controllers: bool = False, controllers_file: str = None, controllers_list: List[str] = None,
                 reset_controllers: bool = False, reset_controllers_prompt: bool = False, kill_rosmaster: bool = True,
                 clean_logs: bool = False, ros_port: str = None, seed: int = None, reset_env_prompt: bool = False):

        """
        Initialize the RealGoalEnv environment.

        Args:
            load_robot (bool): Load the real robot using a launch file.
            robot_pkg_name (str): The name of the package containing the real robot launch file.
            robot_launch_file (str): The name of the robot launch file.
            robot_args (list of str): Args to pass to the robot launch file.
            load_urdf (bool): Whether to load the URDF of the real robot.
            urdf_pkg_name (str): The name of the package containing the URDF file.
            urdf_file_name (str): The name of the URDF file.
            urdf_folder (str): The folder containing the URDF file.
            urdf_xacro_args (List[str]): A list of arguments to pass to xacro when processing the URDF file.
            namespace (str): The ROS namespace to use.
            launch_robot_state_pub (bool): Whether to launch the robot state publisher.
            robot_state_publisher_max_freq (float): The maximum frequency at which to publish robot state messages.
            new_robot_state_term (bool): Whether to launch the robot state publisher in a new terminal.
            load_controllers (bool): Whether to load controllers for the real robot.
            controllers_file (str): The name of the file containing the controller configuration.
            controllers_list (List[str]): A list of controllers to spawn.
            reset_controllers (bool): Whether to reset the controllers when resetting the environment.
            reset_controllers_prompt (bool): Whether to prompt the user before resetting the controllers.
            kill_rosmaster (bool): Whether to kill the ROS master when closing the environment.
            clean_logs (bool): Whether to clean the ROS logs when closing the environment.
            ros_port (str): The port number to use for the ROS master.
            seed (int): The seed for the random number generator.
            reset_env_prompt (bool): Whether to prompt the user for resetting the environment.

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

        rospy.loginfo(self.CYAN + "Start init RealGoalEnv!" + self.ENDC)

        """
        Initialize the variables
        """
        self.ros_port = ros_port
        self.random_seed = seed

        self.info = {}
        self.observation = None
        self.achieved_goal = None
        self.desired_goal = None
        self.reward = 0.0
        self.done = None

        # --------- Change the ros master
        if self.ros_port is not None:
            ros_common.change_ros_master(ros_port=self.ros_port)

        """
        Function to initialise the environment.
        """

        # Init gym.Env
        super().__init__()

        self.namespace = namespace
        self.reset_controllers = reset_controllers
        self.controllers_list = controllers_list
        self.kill_rosmaster = kill_rosmaster
        self.clean_logs = clean_logs
        self.reset_controllers_prompt = reset_controllers_prompt
        self.reset_env_prompt = reset_env_prompt

        # --------- Seed the random number generator
        self.np_random = None

        # defined in Task Env
        if self.random_seed is not None:
            self.seed(self.random_seed)
        else:
            self.seed()

        """
        Load the real robot using a launch file 
        """
        if load_robot:
            ros_common.ros_launch_launcher(pkg_name=robot_pkg_name,
                                           launch_file_name=robot_launch_file,
                                           args=robot_args)

        """
        Load the robot URDF in ROS parameter server and 
        """
        if load_urdf:
            success, _ = ros_common.load_urdf(pkg_name=urdf_pkg_name, file_name=urdf_file_name,
                                              folder=urdf_folder, ns=self.namespace,
                                              args_xacro=urdf_xacro_args)
            if success:
                rospy.loginfo("Robot URDF loaded successfully")
            else:
                rospy.logerr("Error while loading robot URDF")

            time.sleep(0.1)

        """
        launch robot state publisher
        """
        if launch_robot_state_pub:

            success = ros_common.init_robot_state_publisher(ns=self.namespace,
                                                            max_pub_freq=robot_state_publisher_max_freq,
                                                            launch_new_term=new_robot_state_term)
            if success:
                rospy.loginfo("Robot state publisher launched successfully")
            else:
                rospy.logerr("Error while launching robot state publisher")

            time.sleep(0.1)

        """
        Load and spawn the controllers.
        """

        if load_controllers:
            if controllers_file is not None:
                # Load the robot controllers from YAML files in the parameter server
                if ros_common.ros_load_yaml(pkg_name=urdf_pkg_name, file_name=controllers_file, ns=self.namespace):
                    rospy.loginfo("Robot controllers loaded successfully")
                else:
                    rospy.logerr("Error while loading robot controllers")

                time.sleep(0.1)

                # Spawn the controllers
                if ros_controllers.spawn_controllers(controllers_list, ns=self.namespace):
                    rospy.loginfo("Controllers spawned successfully")
                else:
                    rospy.logerr("Error while spawning controllers")

        """
        Reset the controllers
        """
        if self.reset_controllers:
            if self.reset_controllers_prompt:
                user_input = input("Please enter any key to Rest Controllers: ")
                rospy.logdebug(f"Rest Controllers: {user_input}")

            ros_controllers.reset_controllers(controller_list=self.controllers_list, ns=self.namespace)

        rospy.loginfo(self.CYAN + "End init RealGoalEnv" + self.ENDC)

    def seed(self, seed: int = None):
        """
        Seed the random number generator for the environment.

        Args:
            seed (int): The seed for the random number generator.

        Returns:
            seeds (List[int]): The list of seeds used by the random number generator.
        """
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        """
        Take a step in the environment.

        Args:
            action (Any): The action to be applied to the robot.

        Returns:
            observation (dict): A dictionary containing the observation, achieved goal, and desired goal.
            reward (float): The reward for taking the given action.
            done (bool): Whether the episode has ended.
            info (dict): Additional information about the environment.
        """

        # --------- Change the ros master
        if self.ros_port is not None:
            ros_common.change_ros_master(ros_port=self.ros_port)

        # ----- Start the step env
        # rospy.loginfo(self.MAGENTA + "*************** Started Step Env" + self.ENDC)

        # Apply the action
        self._set_action(action)

        # Get the observation, reward, and done flag
        self.info = {}
        self.observation = self._get_observation()
        self.achieved_goal = self._get_achieved_goal()
        self.desired_goal = self._get_desired_goal()
        self.reward = self.compute_reward(self.achieved_goal, self.desired_goal, self.info)
        self.done = self._is_done()

        # rospy.loginfo(self.MAGENTA + "*************** End Step Env" + self.ENDC)

        return {'observation': self.observation, 'achieved_goal': self.achieved_goal,
                'desired_goal': self.desired_goal}, self.reward, self.done, self.info

    def reset(self):
        """
        Reset the environment.

        Returns:
            observation (dict): A dictionary containing the initial observation, achieved goal, and desired goal.
        """

        # --------- Change the ros master
        if self.ros_port is not None:
            ros_common.change_ros_master(ros_port=self.ros_port)

        # ----- Reset the env
        rospy.loginfo(self.MAGENTA + "*************** Start Reset Env" + self.ENDC)

        # Reset real env
        if self.reset_env_prompt:
            user_input = input("Please enter any key after Resting the Environment: ")
            rospy.logdebug(f"Rest Environment!: {user_input}")

        # reset controllers
        if self.reset_controllers:
            if self.reset_controllers_prompt:
                user_input = input("Please enter any key to Rest Controllers: ")
                rospy.logdebug(f"Rest Controllers: {user_input}")

            ros_controllers.reset_controllers(controller_list=self.controllers_list, ns=self.namespace)

        self._check_connection_and_readiness()
        self._set_init_params()

        self.observation = self._get_observation()
        self.achieved_goal = self._get_achieved_goal()
        self.desired_goal = self._get_desired_goal()

        rospy.loginfo(self.MAGENTA + "*************** End Reset Env" + self.ENDC)

        return {'observation': self.observation, 'achieved_goal': self.achieved_goal, 'desired_goal': self.desired_goal}

    def close(self):
        """
        Close the environment.
        """

        # --------- Change the ros master
        if self.ros_port is not None:
            ros_common.change_ros_master(ros_port=self.ros_port)

        rospy.loginfo(self.CYAN + "*************** Start Closing Env" + self.ENDC)

        # Shutdown the ROS node
        rospy.signal_shutdown("Closing Environment")

        if self.kill_rosmaster:
            if self.ros_port is not None:
                ros_common.ros_kill_master(ros_port=self.ros_port)
                rospy.loginfo("Killed ROS Master!")

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

    def _is_done(self):
        """
        Function to check if the episode is done.

        This method should be implemented by subclasses to return a boolean value indicating whether the episode has
        ended (e.g., because a goal has been reached or a failure condition has been triggered).

        Returns:
            A boolean value indicating whether the episode has ended
            (e.g., because a goal has been reached or a failure condition has been triggered)
        """
        raise NotImplementedError()

    def _set_init_params(self):
        """
        Set initial parameters for the environment.

        This method should be implemented by subclasses to set any initial parameters or state variables for the
        environment. This could include resetting joint positions, resetting sensor readings, or any other initial
        setup that needs to be performed at the start of each episode.
        """
        raise NotImplementedError()

    def _get_achieved_goal(self):
        """
        Get the achieved goal from the environment.

        Returns:
            achieved_goal (Any): The achieved goal representing the current state of the environment.
        """
        raise NotImplementedError()

    def _get_desired_goal(self):
        """
        Get the desired goal from the environment.

        Returns:
            desired_goal (Any): The desired goal representing the target state of the environment.
        """
        raise NotImplementedError()

    def compute_reward(self, achieved_goal, desired_goal, info) -> float:
        """
        Compute the reward for achieving a given goal.

        This method should be implemented by subclasses to return a scalar reward value representing how well the agent
        is doing in the current episode. The reward could be based on the distance to a goal, the amount of time taken
        to reach a goal, or any other metric that can be used to measure how well the agent is doing.

        Args:
            achieved_goal (Any): The achieved goal representing the current state of the environment.
            desired_goal (Any): The desired goal representing the target state of the environment.
            info (dict): Additional information about the environment.

        Returns:
            reward (float): The reward for achieving the given goal.
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
    #   Add Custom methods for the RealGoalEnv
