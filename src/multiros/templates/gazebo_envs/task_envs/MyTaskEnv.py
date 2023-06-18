#!/bin/python3

import rospy
from gym.envs.registration import register

# Custom robot env
from multiros.templates.gazebo_envs.robot_envs import MyRobotEnv

# core modules of the framework
from multiros.utils import gazebo_core
# from multiros.utils import gazebo_models
# from multiros.utils import gazebo_physics
# from multiros.utils.moveit_multiros import MoveitMultiros
from multiros.utils import ros_common
# from multiros.utils import ros_controllers
from multiros.utils import ros_markers

# Register your environment using the OpenAI register method to utilize gym.make("TaskEnv-v0").
register(
    id='MyTaskEnv-v0',
    entry_point='multiros.templates.gazebo_envs.task_envs.MyTaskEnv:MyTaskEnv',
    max_episode_steps=100,
)


class MyTaskEnv(MyRobotEnv.MyRobotEnv):
    """
    Use this custom env to implement a task using the robot/sensors related functions defined in the MyRobotEnv
    """

    def __init__(self, launch_gazebo: bool = True, new_roscore: bool = True, roscore_port: str = None,
                 gazebo_paused: bool = False, gazebo_gui: bool = False, seed: int = None):
        """

        In the initialization statement, you can initialize any desired number and type of variables and pass the
        values to the environment as shown below:

        env = gym.make("TaskEnv-v0", launch_gazebo = True, new_roscore = True)

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
        super().__init__(ros_port=ros_port, gazebo_port=gazebo_port, gazebo_pid=gazebo_pid, seed=seed)

        """
        Finished __init__ method
        """
        rospy.loginfo("Finished Init of Custom Task Env")

    # -------------------------------------------------------
    #   Methods for interacting with the environment

    def _set_init_params(self):
        """
        Set initial parameters for the environment.

        This method should be implemented here to set any initial parameters or state variables for the
        environment. This could include resetting joint positions, resetting sensor readings, or any other initial
        setup that needs to be performed at the start of each episode.

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

    def _get_reward(self):
        """
        Function to get a reward from the environment.

        This method should be implemented here to return a scalar reward value representing how well the agent
        is doing in the current episode. The reward could be based on the distance to a goal, the amount of time taken
        to reach a goal, or any other metric that can be used to measure how well the agent is doing.

        Returns:
            A scalar reward value representing how well the agent is doing in the current episode.
        """
        raise NotImplementedError()

    def _is_done(self):
        """
        Function to check if the episode is done.

        This method should be implemented here to return a boolean value indicating whether the episode has
        ended (e.g., because a goal has been reached or a failure condition has been triggered).
        
        Returns:
            A boolean value indicating whether the episode has ended
            (e.g., because a goal has been reached or a failure condition has been triggered)
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
