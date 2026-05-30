#!/bin/python3
from typing import Optional, List, Any, Dict

import rospy
from gymnasium.envs.registration import register
from gymnasium import spaces

# Custom robot env
from multiros.templates.robot_envs import MyMujocoRobotEnv

# core modules of the framework
from multiros.utils import mujoco_core
# from multiros.utils import mujoco_models
# from multiros.utils import mujoco_physics
# from multiros.utils.moveit_multiros import MoveitMultiros
from multiros.utils import ros_common
# from multiros.utils import ros_controllers

# Register your environment using the gymnasium register method to utilize gym.make("MyMujocoTaskEnv-v0").
register(
    id='MyMujocoTaskEnv-v0',
    entry_point='multiros.templates.task_envs.MyMujocoTaskEnv:MyMujocoTaskEnv',
    max_episode_steps=100,
)


class MyMujocoTaskEnv(MyMujocoRobotEnv.MyMujocoRobotEnv):
    """
    Use this custom env to implement a task using the robot/sensors related functions defined in the MyMujocoRobotEnv
    """

    def __init__(self, launch_mujoco: bool = True, new_roscore: bool = True, roscore_port: str = None,
                 mujoco_paused: bool = False, mujoco_gui: bool = False, model_path: str = None,
                 model_pkg: str = None, model_name: str = None, server_name: str = "mujoco_server",
                 seed: int = None, real_time: bool = True, action_cycle_time: float = 0.0):
        """

        In the initialization statement, you can initialize any desired number and type of variables and pass the
        values to the environment as shown below:

        env = gym.make("MyMujocoTaskEnv-v0", launch_mujoco = True, new_roscore = True)

        """

        # if didn't include the variables above, uncomment here
        # launch_mujoco = True
        # new_roscore = True
        # roscore_port = None
        # mujoco_paused = False
        # mujoco_gui = False

        """
        seed for the environment. MujocoBaseEnv creates a random number generator with a given seed.
        The random number generator is stored as an instance variable (self.np_random)
        so that it can be used by other methods in your class.
        """
        # if you didn't include the seed variable in __init__, uncomment here
        # seed = None

        """
        variables to keep track of the ros port and the MuJoCo server process id
        """
        ros_port = None
        mujoco_pid = None

        """
        Initialise the env

        It is recommended to launch the MuJoCo server with a new roscore at this point for the following reasons:,
            1.  This allows running a new rosmaster to enable vectorization of the environment and the execution of
                multiple environments concurrently.
            2.  The environment can keep track of the process ID of the server to automatically close it when
                env.close() is called.

        """
        # launch the MuJoCo server
        if launch_mujoco:

            # Update the function to include additional options.
            ros_port, mujoco_pid = self._launch_mujoco(launch_roscore=new_roscore, port=roscore_port,
                                                       paused=mujoco_paused, headless=not mujoco_gui,
                                                       model_path=model_path, model_pkg=model_pkg,
                                                       model_name=model_name, server_name=server_name)

        # Launch new roscore
        elif new_roscore:
            ros_port = self._launch_roscore(port=roscore_port)

        # ros_port of the already running roscore
        elif roscore_port is not None:
            ros_port = roscore_port

            # change to new rosmaster
            ros_common.change_ros_master(ros_port)

        else:
            """
            Check for roscore
            """
            if ros_common.is_roscore_running() is False:
                print("roscore is not running! Launching a new roscore and the MuJoCo server!")
                ros_port, mujoco_pid = mujoco_core.launch_mujoco(launch_roscore=new_roscore,
                                                                 port=roscore_port,
                                                                 paused=mujoco_paused,
                                                                 headless=not mujoco_gui,
                                                                 model_path=model_path,
                                                                 model_pkg=model_pkg,
                                                                 model_name=model_name,
                                                                 server_name=server_name)

        # init the ros node
        if ros_port is not None:
            self.node_name = "TaskEnv" + "_" + ros_port
        else:
            self.node_name = "TaskEnv"

        rospy.init_node(self.node_name, anonymous=True)

        """
        Provide a description of the task.
        """
        rospy.loginfo("Starting Custom Task Env")

        """
        Load YAML param file
        """

        # add to ros parameter server
        # ros_common.ros_load_yaml(pkg_name="pkg_name", file_name="file_name.yaml", ns="ns")
        # self._get_params()

        """
        Define the action space.
        """
        # self.action_space = spaces.Discrete(n_actions)
        # self.action_space = spaces.Box(low=0, high=1, shape=(1,), dtype=np.float32)
        # ROS often uses double-precision (64-bit)
        # But if you are using Stable Baseline3, you need to define them as float32, otherwise it won't work

        """
        Define the observation space.
        """
        # self.observation_space = spaces.Discrete(n_observations)
        # self.observation_space = spaces.Box(low=0, high=1, shape=(1,), dtype=np.float32)

        """
        Define subscribers/publishers and Markers as needed.
        """

        # self.goal_marker = ros_markers.RosMarker(frame_id="world", ns="", marker_type=2, marker_topic="goal_pos",
        #                                          lifetime=10.0)

        """
        Init super class.
        """
        super().__init__(ros_port=ros_port, mujoco_pid=mujoco_pid, server_name=server_name, seed=seed,
                         real_time=real_time, action_cycle_time=action_cycle_time)

        """
        Finished __init__ method
        """
        rospy.loginfo("Finished Init of Custom Task Env")

    # -------------------------------------------------------
    #   Methods for interacting with the environment

    def _set_init_params(self, options: Optional[Dict[str, Any]] = None):
        """
        Set initial parameters for the environment.

        This method should be implemented here to set any initial parameters or state variables for the
        environment. This could include resetting joint positions, resetting sensor readings, or any other initial
        setup that needs to be performed at the start of each episode.

        Args:
            options (dict): Additional options for setting the initial parameters.

        """
        raise NotImplementedError()

    def _set_action(self, action):
        """
        Function to apply an action to the robot.

        This method should be implemented here to apply the given action to the robot. The action could be a
        joint position command, a velocity command, or any other type of command that can be applied to the robot.

        Args:
            action: The action to be applied to the robot.
        """
        raise NotImplementedError()

    def _get_observation(self):
        """
        Function to get an observation from the environment.

        This method should be implemented here to return an observation representing the current state of
        the environment. The observation could be a sensor reading, a joint state, or any other type of observation
        that can be obtained from the environment.

        Returns:
            An observation representing the current state of the environment.
        """
        raise NotImplementedError()

    def _get_reward(self, info: Optional[Dict[str, Any]] = None):
        """
        Function to get a reward from the environment.

        This method should be implemented here to return a scalar reward value representing how well the agent
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

        This method should be implemented here to return a boolean value indicating whether the episode has
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

        This method should be implemented here to return a boolean value indicating whether the episode has
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

    # -------------------------------------------------------
    #   Include any custom methods available for the MyMujocoTaskEnv class

    def _get_params(self):
        """
        Function to get configuration parameters (optional)
        """
        raise NotImplementedError()

    # ------------------------------------------------------
    #   Task Methods for launching the MuJoCo server or roscore
    def _launch_mujoco(self, launch_roscore=True, port=None, paused=False, use_sim_time=True,
                       model_path=None, model_pkg=None, model_name=None, headless=True,
                       no_render=False, realtime=None, mujoco_plugin_config=None,
                       initial_joint_states=None, server_name="mujoco_server", ns="",
                       verbose=False, output='screen', launch_new_term=True):
        """
        Launches a new MuJoCo server with the specified options.

        Returns:
            ros_port: None if only launching the server and no roscore
            mujoco_pid: process id for the launched MuJoCo server

        """
        ros_port, mujoco_pid = mujoco_core.launch_mujoco(
            launch_roscore=launch_roscore,
            port=port,
            paused=paused,
            use_sim_time=use_sim_time,
            model_path=model_path,
            model_pkg=model_pkg,
            model_name=model_name,
            headless=headless,
            no_render=no_render,
            realtime=realtime,
            mujoco_plugin_config=mujoco_plugin_config,
            initial_joint_states=initial_joint_states,
            server_name=server_name,
            ns=ns,
            verbose=verbose,
            output=output,
            launch_new_term=launch_new_term
        )

        return ros_port, mujoco_pid

    def _launch_roscore(self, port=None, set_new_master_vars=False):
        """
        Launches a new roscore with the specified port. Only updates the ros_port.

        Return:
            ros_port: port of launched roscore
        """

        ros_port, _ = ros_common.launch_roscore(port=int(port), set_new_master_vars=set_new_master_vars)

        # change to new rosmaster
        ros_common.change_ros_master(ros_port)

        return ros_port
