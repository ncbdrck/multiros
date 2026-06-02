#!/bin/python3

import numpy as np
import rospy
import gymnasium as gym
from multiros.utils import mujoco_core
from multiros.utils import mujoco_models
from multiros.utils import mujoco_physics
from multiros.utils import ros_common
from multiros.utils import ros_controllers
from typing import Any, Dict, List, Mapping, Optional, Tuple


def _safe_ros_sleep(duration):
    """rospy.sleep that tolerates the simulation clock being rewound by a reset.

    Under use_sim_time, resetting the simulator rewinds /clock, which makes rospy.sleep raise
    ROSTimeMovedBackwardsException on the first sleep after a reset. The rewind is a one-off at
    the episode boundary, so we swallow it and let the next iteration pace normally.
    """
    try:
        rospy.sleep(duration)
    except rospy.exceptions.ROSTimeMovedBackwardsException:
        pass


class MujocoBaseEnv(gym.Env):
    """
    A custom gymnasium environment for reinforcement learning using ROS and MuJoCo.
    """

    def __init__(self, spawn_robot: bool = False, urdf_pkg_name: str = None, urdf_file_name: str = None,
                 urdf_folder: str = "/urdf", urdf_xacro_args: List[str] = None, namespace: str = "/",
                 robot_state_publisher_max_freq: float = None, new_robot_state_term: bool = False,
                 controllers_file: str = None, controllers_list: List[str] = None,
                 reset_controllers: bool = False, reset_mode: str = "world", sim_step_mode: int = 1,
                 num_mujoco_steps: int = 1, mujoco_max_update_rate: float = None, mujoco_timestep: float = None,
                 kill_rosmaster: bool = True, kill_mujoco: bool = True, clean_logs: bool = False,
                 ros_port: str = None, mujoco_pid=None, server_name: str = "mujoco_server", seed: int = None,
                 unpause_pause_physics: bool = True, action_cycle_time: float = 0.0, log_internal_state: bool = False,
                 controller_package_name: str = None, controlled_joints: List[str] = None
                 ):

        """
        Initialize the MujocoBaseEnv.

        Args:
            spawn_robot (bool): Whether to bring up the robot's ROS interfaces.
            urdf_pkg_name (str): The name of the URDF package.
            urdf_file_name (str): The name of the URDF file.
            urdf_folder (str): The folder containing the URDF file.
            urdf_xacro_args (List[str]): The arguments for the xacro processor.
            namespace (str): The ROS namespace for the robot.
            robot_state_publisher_max_freq (float): The maximum frequency for the robot state publisher.
            new_robot_state_term (bool): Whether to use a new terminal for the robot state publisher.
            controllers_file (str): The file containing the controller configurations.
            controllers_list (List[str]): The list of ROS controllers to use.
            reset_controllers (bool): Whether to reset the controllers on reset.
            reset_mode (str): Accepted for interface compatibility with the Gazebo backend.
            sim_step_mode (int): The mode to use when stepping the simulation (1 or 2). Mode 2 advances the
                simulation with the step action and requires the simulation to be paused.
            num_mujoco_steps (int): The number of simulation steps to take per step call in mode 2.
            mujoco_max_update_rate (float): The real-time factor for the simulation.
            mujoco_timestep (float): The solver time step for the simulation.
            kill_rosmaster (bool): Whether to kill the ROS master on close.
            kill_mujoco (bool): Whether to kill the MuJoCo server on close.
            clean_logs (bool): Whether to clean the ROS logs on close.
            ros_port (str): The ROS_MASTER_URI port.
            mujoco_pid (subprocess.Popen): A subprocess.Popen object representing the running MuJoCo instance.
            server_name (str): The graph name of the MuJoCo server node.
            seed (int): Seed for random number generator.
            unpause_pause_physics (bool): Whether to unpause and pause the simulation before and after each step call.
            action_cycle_time (float): The time to wait between applying actions.
            log_internal_state (bool): Whether to log the internal state of the environment.
            controller_package_name (str): The name of the package containing the controllers.

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

        rospy.loginfo(self.CYAN + "Start init MujocoBaseEnv!" + self.ENDC)

        """
        Initialize the variables
        """
        self.ros_port = ros_port
        self.mujoco_pid = mujoco_pid
        self.server_name = server_name
        # Constructor-supplied seed. Used once on the first reset() that
        # doesn't get an explicit seed= kwarg; after that np_random
        # advances normally (Gymnasium semantics).
        self.user_seed = seed
        self._initial_seed_consumed = False
        self.unpause_pause_physics = unpause_pause_physics
        self.action_cycle_time = action_cycle_time
        self.log_internal_state = log_internal_state

        self.info = {}
        self.terminated = None
        self.truncated = None
        self.reward = 0.0
        self.observation = None

        # --------- Change the ros master
        # defined in Task Env
        if self.ros_port is not None:
            ros_common.change_ros_master(ros_port=self.ros_port)

        """
        Function to initialise the environment.
        """

        # Init gymnasium.Env
        super().__init__()

        self.namespace = namespace
        self.reset_controllers = reset_controllers
        self.controllers_list = controllers_list
        self.controlled_joints = controlled_joints
        self.reset_mode = reset_mode
        self.sim_step_mode = sim_step_mode
        self.num_mujoco_steps = num_mujoco_steps
        self.kill_rosmaster = kill_rosmaster
        self.kill_mujoco = kill_mujoco
        self.clean_logs = clean_logs

        """
        Set MuJoCo physics parameters to change the speed of the simulation
        """

        if mujoco_max_update_rate is not None:
            mujoco_physics.set_mujoco_max_update_rate(real_time_factor=mujoco_max_update_rate,
                                                      server_name=self.server_name)

        if mujoco_timestep is not None:
            mujoco_physics.set_mujoco_time_step(time_step=mujoco_timestep, server_name=self.server_name)

        """
        Bring up the robot's ROS interfaces
        """
        # The robot geometry is provided by the MJCF scene loaded by the server; here we bring up
        # the robot description, state publisher and controllers.
        if spawn_robot:
            mujoco_models.spawn_robot_in_mujoco(pkg_name=urdf_pkg_name, model_urdf_file=urdf_file_name,
                                                model_urdf_folder=urdf_folder, ns=self.namespace,
                                                args_xacro=urdf_xacro_args, pub_freq=robot_state_publisher_max_freq,
                                                rob_st_term=new_robot_state_term,
                                                controllers_file=controllers_file,
                                                controllers_list=self.controllers_list,
                                                controller_package_name=controller_package_name,
                                                controlled_joints=self.controlled_joints
                                                )

        """
        Reset the controllers
        """
        if self.reset_controllers:
            if self.unpause_pause_physics:
                mujoco_core.unpause_mujoco(server_name=self.server_name)

            ros_controllers.reset_controllers(controller_list=self.controllers_list, ns=self.namespace)

            if self.unpause_pause_physics:
                mujoco_core.pause_mujoco(server_name=self.server_name)

        rospy.loginfo(self.CYAN + "End init MujocoBaseEnv" + self.ENDC)

    def step(self, action: Any) -> Tuple[Any, float, bool, bool, Dict[str, Any]]:
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

        # ----- Change the ros master

        if self.ros_port is not None:
            ros_common.change_ros_master(ros_port=self.ros_port)

        # ----- Start the step env

        if self.sim_step_mode == 1:
            if self.unpause_pause_physics:
                # Non-real-time: unpause, apply the action, let the sim run for action_cycle_time,
                # THEN pause. The sleep must happen while unpaused -- under use_sim_time a paused
                # server stops publishing /clock, so sleeping after pausing blocks forever
                # (rospy.sleep waits on sim time that never advances). Running during the sleep also
                # gives the trajectory controller simulated time to actually move the arm.
                mujoco_core.unpause_mujoco(server_name=self.server_name)
                self._set_action(action)
                if self.action_cycle_time > 0.0:
                    _safe_ros_sleep(self.action_cycle_time)
                mujoco_core.pause_mujoco(server_name=self.server_name)
            else:
                # Real-time loop: physics is never paused; apply the action and let the trailing
                # sleep pace the step while the background timer refreshes obs/reward. This is the
                # loop intended for sim->real transfer because it matches the real-env timing.
                self._set_action(action)
                if self.action_cycle_time > 0.0:
                    _safe_ros_sleep(self.action_cycle_time)

        # Deterministic step (fast) mode: the sim must be paused before stepping is honoured
        # (``mujoco_step`` is a no-op when the server is running free), so pause first, advance
        # explicitly for num_mujoco_steps ticks, and leave the sim paused for the obs read.
        # With the default ``unpause_pause_physics=False`` the previous step left the sim
        # unpaused, which silently dropped the step request.
        elif self.sim_step_mode == 2:
            mujoco_core.pause_mujoco(server_name=self.server_name)
            self._set_action(action)
            mujoco_core.mujoco_step(steps=self.num_mujoco_steps, server_name=self.server_name)

        # Get the observation, reward and terminated, truncated flags
        self.info = {}
        self.observation = self._get_observation()
        self.reward = self._get_reward(info=self.info)
        self.terminated = self._compute_terminated(info=self.info)
        self.truncated = self._compute_truncated(info=self.info)

        return self.observation, self.reward, self.terminated, self.truncated, self.info

    def reset(self, seed: Optional[int] = None,
              options: Optional[Mapping[str, Any]] = None,
              ) -> Tuple[Any, Dict[str, Any]]:

        """
        Reset the environment.

        Args:
            seed (int): The seed for the random number generator.
            options (dict): Additional information for resetting the environment.

        Returns:
            observation (Any): The initial observation representing the state of the environment.
            info (dict): Additional information about the environment. Similar to the info returned by step().
        """

        # Gymnasium-correct seed handling:
        #   * If the caller passes seed=X, honour it (overrides constructor seed).
        #   * Otherwise, on the very first reset, fall back to the
        #     constructor seed (consume it once). After that, leave
        #     seed=None so np_random advances normally between episodes.
        # The old behaviour re-applied the constructor seed on every reset,
        # which violated Gymnasium semantics and made every episode start
        # from the same np_random state.
        if seed is None and not self._initial_seed_consumed:
            seed = self.user_seed
            self._initial_seed_consumed = True
        super().reset(seed=seed)

        # reinitialize the info dictionary
        self.info = {}

        # ----- Change the ros master
        if self.ros_port is not None:
            ros_common.change_ros_master(ros_port=self.ros_port)

        # ----- Reset the env
        if self.log_internal_state:
            rospy.loginfo(self.MAGENTA + "*************** Start Reset Env" + self.ENDC)

        # Reset MuJoCo and get the initial observation
        self._reset_mujoco(options=options)
        self.observation = self._get_observation()

        if self.log_internal_state:
            rospy.loginfo(self.MAGENTA + "*************** End Reset Env" + self.ENDC)

        return self.observation, self.info

    def _sample_box(self, box: gym.spaces.Box) -> np.ndarray:
        """
        Sample uniformly from a ``gym.spaces.Box`` using the env's own
        ``self.np_random``.

        ``gym.spaces.Box.sample()`` uses the space's internal RNG, which is
        a separate ``np.random.Generator`` instance not reseeded by
        ``env.reset(seed=...)``. Calling ``reset(seed=X)`` only seeds
        ``self.np_random`` (the env-level generator), so spaces created
        with ``Box(..., seed=...)`` ignore the reset seed and break
        Gymnasium's reproducibility contract.

        Use this helper at sampling sites that should reproduce when the
        caller passes a reset seed (goal poses, cube poses, anything that
        determines the rollout). Returns an array of the box's dtype.
        """
        return self.np_random.uniform(box.low, box.high).astype(box.dtype)

    def _safe_unit_vector(self, vec: np.ndarray, eps: float = 1e-8) -> np.ndarray:
        """
        Return ``vec / ||vec||`` with a zero-norm guard.

        Task envs commonly use a direction-to-goal vector as part of the
        observation: ``linear_dist_ee_goal / np.linalg.norm(...)``. When
        the EE is exactly at the goal (e.g. just after a successful
        reach), the norm is 0 and the division yields NaNs that pollute
        the observation and may fail an SB3 / env_checker observation-
        space check. Returning a zero vector below ``eps`` is the
        conventional safe behaviour: the direction is undefined at the
        goal, and a zero vector encodes "no direction" without breaking
        downstream arithmetic.
        """
        n = float(np.linalg.norm(vec))
        if n < eps:
            return np.zeros_like(vec)
        return vec / n

    def close(self) -> None:
        """
        Close the environment.
        """

        # ----- Change the ros master
        if self.ros_port is not None:
            ros_common.change_ros_master(ros_port=self.ros_port)

        rospy.loginfo(self.CYAN + "*************** Start Closing Env" + self.ENDC)

        # Shutdown the ROS node
        rospy.signal_shutdown("Closing Environment")

        # Close MuJoCo before killing the roscore: close_mujoco first tries the graceful
        # /<server>/shutdown service, which is only reachable while the master is still up.
        # (Killing the master first would force the SIGTERM/SIGKILL fallback every time and can
        # leave server children behind across repeated training runs.)
        if self.kill_mujoco:
            if self.mujoco_pid is not None:
                mujoco_core.close_mujoco(process=self.mujoco_pid, ros_port=self.ros_port,
                                         server_name=self.server_name)
                rospy.loginfo("Closed MuJoCo!")

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
    #   Custom methods for the MujocoBaseEnv

    def _reset_mujoco(self, options: Optional[Dict[str, Any]] = None):
        """
        Helper function to reset the MuJoCo simulation and the controllers.

        The simulation is reset to the model's default configuration and the configured initial joint states.

        Args:
            options (dict): Additional options for resetting the environment.
        """

        # Pause the simulation and reset it
        if self.unpause_pause_physics:
            mujoco_core.pause_mujoco(server_name=self.server_name)
        mujoco_core.reset_mujoco(reset_type=self.reset_mode, server_name=self.server_name)

        # Reset the controllers
        if self.reset_controllers:
            if self.unpause_pause_physics:
                mujoco_core.unpause_mujoco(server_name=self.server_name)

            ros_controllers.reset_controllers(controller_list=self.controllers_list, ns=self.namespace)

            if self.unpause_pause_physics:
                mujoco_core.pause_mujoco(server_name=self.server_name)

        # Unpause the simulation and check the connection status
        if self.unpause_pause_physics:
            mujoco_core.unpause_mujoco(server_name=self.server_name)
        self._check_connection_and_readiness()

        # set initial parameters for the environment
        self._set_init_params(options=options)

        # pause the simulation
        if self.unpause_pause_physics:
            mujoco_core.pause_mujoco(server_name=self.server_name)
