#!/bin/python3

import rospy
import numpy as np
from gymnasium import spaces
from gymnasium.envs.registration import register
from typing import List, Any, Dict, Optional

# Custom robot env
from multiros.templates.robot_envs import MyRobotGoalEnv

# core modules of the framework
from multiros.utils import gazebo_core
# from multiros.utils import gazebo_models
# from multiros.utils import gazebo_physics
# from multiros.utils.moveit_multiros import MoveitMultiros
from multiros.utils import ros_common
# from multiros.utils import ros_controllers

# Register your environment using the gymnasium register method to utilize gym.make("MyTaskGoalEnv-v0").
register(
    id='MyTaskGoalEnv-v0',
    entry_point='multiros.templates.task_envs.MyTaskGoalEnv:MyTaskGoalEnv',
    max_episode_steps=100,
)


class MyTaskGoalEnv(MyRobotGoalEnv.MyRobotGoalEnv):
    """
    Use this custom env to implement a task using the robot/sensors related functions defined in the MyRobotGoalEnv
    """

    def __init__(self, launch_gazebo: bool = True, new_roscore: bool = True, roscore_port: str = None,
                 gazebo_paused: bool = False, gazebo_gui: bool = False, seed: int = None, real_time: bool = True,
                 action_cycle_time: float = 0.0):
        """

        In the initialization statement, you can initialize any desired number and type of variables and pass the
        values to the environment as shown below:

        env = gym.make("MyTaskGoalEnv-v0", launch_gazebo = True, new_roscore = True)

        """

        # if didn't include the variables above, uncomment here
        # launch_gazebo = True
        # new_roscore = True
        # roscore_port = None
        # gazebo_paused = False
        # gazebo_gui = False

        """
        seed for the environment. GazeboGoalEnv create a random number generator with a given seed. 
        The random number generator is stored as an instance variable (self.np_random) 
        so that it can be used by other methods in your class.
        """
        # if you didn't include the seed variable in __init__, uncomment here
        # seed = None

        """
        variables to keep track of ros, gazebo ports and gazebo pid 
        """
        ros_port = None
        gazebo_port = None
        gazebo_pid = None

        """
        Initialise the env
        
        It is recommended to launch Gazebo with a new roscore at this point for the following reasons:, 
            1.  This allows running a new rosmaster to enable vectorization of the environment and the execution of 
                multiple environments concurrently.
            2.  The environment can keep track of the process ID of Gazebo to automatically close it when env.close() 
                is called.
        
        """
        # launch gazebo
        if launch_gazebo:

            # Update the function to include additional options.
            ros_port, gazebo_port, gazebo_pid = self._launch_gazebo(launch_roscore=new_roscore, port=roscore_port,
                                                                    paused=gazebo_paused, gui=gazebo_gui)

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
                print("roscore is not running! Launching a new roscore and Gazebo!")
                ros_port, gazebo_port, gazebo_pid = gazebo_core.launch_gazebo(launch_roscore=new_roscore,
                                                                              port=roscore_port,
                                                                              paused=gazebo_paused,
                                                                              gui=gazebo_gui)

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
        # ROS and Gazebo often use double-precision (64-bit)
        # But if you are using Stable Baseline3, you need to define them as float32, otherwise it won't work

        """
        Define the observation space.
        
        observation space is a dictionary with 
            observation: 
            achieved_goal: 
            desired_goal: 
        """

        # Define the achieved_goal and desired_goal subspaces
        self.achieved_goal_space = spaces.Box(low=0, high=1, shape=(3,))
        self.desired_goal_space = spaces.Box(low=0, high=1, shape=(3,))

        # Define the overall observation space
        self.observation_space = spaces.Dict({
            'observation': spaces.Box(low=0, high=1, shape=(5,), dtype=np.float32),
            'achieved_goal': self.achieved_goal_space,
            'desired_goal': self.desired_goal_space
        })

        """
        Define subscribers/publishers and Markers as needed.
        """

        # self.goal_marker = ros_markers.RosMarker(frame_id="world", ns="", marker_type=2, marker_topic="goal_pos",
        #                                          lifetime=10.0)

        """
        Init super class.
        """
        super().__init__(ros_port=ros_port, gazebo_port=gazebo_port, gazebo_pid=gazebo_pid, seed=seed,
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
            options (dict): Additional options for setting the initial parameters. Comes from the env.reset() method.
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

        This method should be implemented here to return a scalar reward value representing how well the agent
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

    def compute_terminated(self, achieved_goal, desired_goal, info):
        """
        Function to check if the episode is terminated due to reaching a terminal state.

        This method should be implemented here to return a boolean value indicating whether the episode has
        ended (e.g., because a goal has been reached or a failure condition has been triggered).

        Args:
            achieved_goal (Any): The achieved goal representing the current state of the environment.
            desired_goal (Any): The desired goal representing the target state of the environment.
            info (dict): Additional information for computing the termination condition.

        Returns:
            A boolean value indicating whether the episode has ended
            (e.g., because a goal has been reached or a failure condition has been triggered)
        """
        raise NotImplementedError()

    def compute_truncated(self, achieved_goal, desired_goal, info):
        """
        Function to check if the episode is truncated due non-terminal reasons.

        This method should be implemented here to return a boolean value indicating whether the episode has
        been truncated due to reasons other than reaching a terminal state.
        Truncated states are those that are out of the scope of the Markov Decision Process (MDP).
        This could include truncation due to reaching a maximum number of steps, or any other non-terminal condition
        that causes the episode to end early.

        Args:
            achieved_goal (Any): The achieved goal representing the current state of the environment.
            desired_goal (Any): The desired goal representing the target state of the environment.
            info (dict): Additional information for computing the truncation condition.

        Returns:
            A boolean value indicating whether the episode has been truncated.
        """
        raise NotImplementedError()

    # -------------------------------------------------------
    #   Include any custom methods available for the MyTaskEnv class

    def _get_params(self):
        """
        Function to get configuration parameters (optional)
        """
        raise NotImplementedError()

    # ------------------------------------------------------
    #   Task Methods for launching gazebo or roscore
    def _launch_gazebo(self, launch_roscore=True, port=None, paused=False, use_sim_time=True,
                       extra_gazebo_args=None, gui=False, recording=False, debug=False,
                       physics="ode", verbose=False, output='screen', respawn_gazebo=False,
                       pub_clock_frequency=100, server_required=False, gui_required=False,
                       custom_world_path=None, custom_world_pkg=None, custom_world_name=None,
                       launch_new_term=True):
        """
        Launches a new Gazebo simulation with the specified options.

        Returns:
            ros_port: None if only launching gazebo and no roscore
            gazebo_port: None if only launching gazebo and no roscore
            gazebo_pid: process id for launched gazebo

        """
        ros_port, gazebo_port, gazebo_pid = gazebo_core.launch_gazebo(
            launch_roscore=launch_roscore,
            port=port,
            paused=paused,
            use_sim_time=use_sim_time,
            extra_gazebo_args=extra_gazebo_args,
            gui=gui,
            recording=recording,
            debug=debug,
            physics=physics,
            verbose=verbose,
            output=output,
            respawn_gazebo=respawn_gazebo,
            pub_clock_frequency=pub_clock_frequency,
            server_required=server_required,
            gui_required=gui_required,
            custom_world_path=custom_world_path,
            custom_world_pkg=custom_world_pkg,
            custom_world_name=custom_world_name,
            launch_new_term=launch_new_term
        )

        return ros_port, gazebo_port, gazebo_pid

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
