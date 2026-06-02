#!/bin/python3
from typing import Optional, List, Any, Dict

from gymnasium import spaces
from gymnasium.envs.registration import register

from multiros.envs import MujocoBaseEnv

import rospy

# core modules of the framework
from multiros.utils import mujoco_core
from multiros.utils import mujoco_models
from multiros.utils import mujoco_physics
from multiros.utils.moveit_multiros import MoveitMultiros
from multiros.utils import ros_common
from multiros.utils import ros_controllers
from multiros.utils import ros_markers
from multiros.utils import ros_kinematics

"""
Although it is best to register only the task environment, one can also register the robot environment.
This is not necessary, but we can see if this section
(Load the robot in MuJoCo and can control the robot with moveit or ros controllers)
works by calling "gym.make" this env.
but you need to
    1. run the MuJoCo server - mujoco_core.launch_mujoco(launch_roscore=False, paused=False, model_path=...)
    2. init a node - rospy.init_node('test_MyMujocoRobotEnv')
"""
register(
    id='MyMujocoRobotEnv-v0',
    entry_point='multiros.templates.robot_envs.MyMujocoRobotEnv:MyMujocoRobotEnv',
    max_episode_steps=1000,
)


class MyMujocoRobotEnv(MujocoBaseEnv.MujocoBaseEnv):
    """
    Custom Robot Env, use this class to describe the robots and the sensors in the Environment.
    Superclass for all Robot environments.
    """

    def __init__(self, ros_port: Optional[str] = None,
                 mujoco_pid: Optional[Any] = None,
                 server_name: str = "mujoco_server",
                 seed: Optional[int] = None,
                 real_time: bool = False,
                 action_cycle_time: float = 0.0) -> None:
        """
        Initializes a new Robot Environment

        Describe the robot and the sensors used in the env.

        Sensor Topic List:
            /joint_states : JointState received for the joints of the robot

        Actuators Topic List:
            MoveIt!: MoveIt action server is used to send the joint positions to the robot.
        """
        rospy.loginfo("Start Init Custom Robot Env")

        """
        Change the ros master
        """
        if ros_port is not None:
            ros_common.change_ros_master(ros_port=ros_port)

        """
        parameters
        """
        self.real_time = real_time  # if True, the simulation will run in real time
        self.server_name = server_name

        # we don't need to pause/unpause the simulation if we are running in real time
        if self.real_time:
            unpause_pause_physics = False
        else:
            unpause_pause_physics = True

        """
        Unpause the simulation
        """
        if not self.real_time:
            mujoco_core.unpause_mujoco(server_name=self.server_name)

        """
        Bring up the robot's ROS interfaces.

        The robot geometry is provided by the MJCF scene loaded by the MuJoCo server (see the Task
        Env's launch step). Setting spawn_robot=True loads the robot description onto the parameter
        server and starts the robot_state_publisher and controllers.
        """
        spawn_robot = False

        # location of the robot URDF file
        urdf_pkg_name = None
        urdf_file_name = None
        urdf_folder = "/urdf"

        # extra urdf args
        urdf_xacro_args = None

        # namespace of the robot
        namespace = "/"

        # robot state publisher
        robot_state_publisher_max_freq = None
        new_robot_state_term = False

        # controller (must be inside above pkg_name/config/)
        controllers_file = None
        controllers_list = None

        """
        Place additional objects in the scene.

        MuJoCo loads the world and all objects from the MJCF scene at launch. Free-jointed bodies
        declared in the scene can be repositioned at run time, for example:
        # mujoco_models.mujoco_set_body_state(body_name="cube", pos_x=0.2, pos_y=0.0, pos_z=0.1,
        #                                      server_name=self.server_name)
        """

        """
        Set if the controllers in "controller_list" will be reset at the beginning of each episode, default is False.
        """
        reset_controllers = False

        """
        You can adjust the simulation step mode with two options:

            1. Unpause, set the action and pause the simulation.
            2. Use the step action of the MuJoCo server (requires the simulation to be paused).

        By default, the simulation step mode is set to 1. With step mode 2 you can specify the
        number of steps to take in each iteration. The default value for this is 1.
        """
        sim_step_mode = 1
        num_mujoco_steps = 1

        """
        Set MuJoCo physics parameters to change the speed of the simulation
        """
        mujoco_max_update_rate = None
        mujoco_timestep = None

        """
        kill rosmaster at the end of the env
        """
        kill_rosmaster = True

        """
        kill the MuJoCo server at the end of the env
        """
        kill_mujoco = True

        """
        Clean ros Logs at the end of the env
        """
        clean_logs = False

        """
        Init MujocoBaseEnv.
        """
        super().__init__(
            spawn_robot=spawn_robot, urdf_pkg_name=urdf_pkg_name, urdf_file_name=urdf_file_name,
            urdf_folder=urdf_folder, urdf_xacro_args=urdf_xacro_args, namespace=namespace,
            robot_state_publisher_max_freq=robot_state_publisher_max_freq, new_robot_state_term=new_robot_state_term,
            controllers_file=controllers_file, controllers_list=controllers_list,
            reset_controllers=reset_controllers, sim_step_mode=sim_step_mode,
            num_mujoco_steps=num_mujoco_steps, mujoco_max_update_rate=mujoco_max_update_rate,
            mujoco_timestep=mujoco_timestep, kill_rosmaster=kill_rosmaster, kill_mujoco=kill_mujoco,
            clean_logs=clean_logs, ros_port=ros_port, mujoco_pid=mujoco_pid, server_name=server_name, seed=seed,
            unpause_pause_physics=unpause_pause_physics, action_cycle_time=action_cycle_time)

        """
        Define ros publisher, subscribers and services for robot and sensors
        """
        # example: joint state
        # if namespace is not None and namespace != '/':
        #     self.joint_state_topic = namespace + "/joint_states"
        # else:
        #     self.joint_state_topic = "/joint_states"
        #
        # self.joint_state_sub = rospy.Subscriber(self.joint_state_topic, JointState, self.joint_state_callback)
        # self.joint_state = JointState()

        # example: moveit package
        # self.move_RX200_object = MoveitMultiros(arm_name='interbotix_arm',
        #                                         gripper_name='interbotix_gripper',
        #                                         robot_description="rx200/robot_description",
        #                                         ns="rx200", pause_gazebo=False)

        """
        Using the _check_connection_and_readiness method to check for the connection status of subscribers, publishers
        and services
        """
        self._check_connection_and_readiness()

        """
        initialise controller and sensor objects here
        """

        # example - Moveit object
        # self.moveit_robot_object = MoveitMultiros(arm_name='arm_group',
        #                                           gripper_name='gripper_group',
        #                                           robot_description="namespace/robot_description",
        #                                           ns="namespace", pause_gazebo=False)

        """
        Finished __init__ method
        """
        if not self.real_time:
            mujoco_core.pause_mujoco(server_name=self.server_name)
        else:
            mujoco_core.unpause_mujoco(server_name=self.server_name)
        rospy.loginfo("End Init MyMujocoRobotEnv")

    # ---------------------------------------------------
    #   Custom methods for the Custom Robot Environment

    """
    Define the custom methods for the environment
        * callbacks from subscribers
        * functions to move robot
        * functors to read data from robot
        * etc
    """

    # example:1
    # def get_ee_pose(self):
    #     """
    #     Returns the end-effector pose as a geometry_msgs/PoseStamped message
    #     """
    #     ee_pose = self.moveit_robot_object.get_robot_pose()
    #     return ee_pose

    # ---------------------------------------------------
    #   Methods to override in Custom Robot Environment

    def _check_connection_and_readiness(self) -> bool:
        """
        Function to check the connection status of subscribers, publishers and services, as well as the readiness of
        all systems.
        """
        return True

    # ---------------------------------------------------
    #    Methods to override in Custom Task Environment

    def _set_action(self, action: Any) -> None:
        """
        Function to apply an action to the robot.

        This method should be implemented by subclasses to apply the given action to the robot. The action could be a
        joint position command, a velocity command, or any other type of command that can be applied to the robot.

        Args:
            action: The action to be applied to the robot.
        """
        raise NotImplementedError()

    def _get_observation(self) -> Any:
        """
        Get an observation from the environment.

        This method should be implemented by subclasses to return an observation representing the current state of
        the environment. The observation could be a sensor reading, a joint state, or any other type of observation
        that can be obtained from the environment.

        Returns:
            observation (Any): An observation representing the current state of the environment.
        """
        raise NotImplementedError()

    def _get_reward(self, info: Optional[Dict[str, Any]] = None) -> float:
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

    def _compute_terminated(self, info: Optional[Dict[str, Any]] = None) -> bool:
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

    def _compute_truncated(self, info: Optional[Dict[str, Any]] = None) -> bool:
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

    def _set_init_params(self, options: Optional[Dict[str, Any]] = None) -> None:
        """
        Set initial parameters for the environment.

        This method should be implemented by subclasses to set any initial parameters or state variables for the
        environment. This could include resetting joint positions, resetting sensor readings, or any other initial
        setup that needs to be performed at the start of each episode.

        Args:
            options (dict): Additional options for setting the initial parameters. Comes from the env.reset() method.
        """
        raise NotImplementedError()
