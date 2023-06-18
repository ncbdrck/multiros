#!/bin/python3

import rospy
import rostopic
from gym import spaces
from gym.envs.registration import register

from multiros.envs.real_envs import RealBaseEnv

# core modules of the framework
from multiros.utils.moveit_multiros import MoveitMultiros
from multiros.utils import ros_common
from multiros.utils import ros_controllers
from multiros.utils import ros_markers

"""
Although it is best to register only the task environment, one can also register the robot environment. 
This is not necessary, but we can see if this section works by calling "gym.make" this env.
"""
register(
    id='MyRealRobotEnv-v0',
    entry_point='multiros.templates.real_envs.robot_envs.MyRealRobotEnv:MyRealRobotEnv',
    max_episode_steps=1000,
)


class MyRealRobotEnv(RealBaseEnv.RealBaseEnv):
    """
    Custom Robot Env, use this class to describe the robots and the sensors in the Environment.
    Superclass for all Robot environments.
    """

    def __init__(self, ros_port: str = None, seed: int = None, reset_env_prompt: bool = False):
        """
        Initializes a new Robot Environment

        Describe the robot and the sensors used in the env.

        Sensor Topic List:
            /joint_states : JointState received for the joints of the robot

        Actuators Topic List:
            MoveIt! : MoveIt! action server is used to send the joint positions to the robot.
        """
        rospy.loginfo("Start Init Custom Robot Env")

        """
        Change the ros/gazebo master
        """
        if ros_port is not None:
            ros_common.change_ros_master(ros_port=ros_port)

        """
        Launch a roslaunch file that will setup the connection with the real robot 
        """

        load_robot = False
        robot_pkg_name = None
        robot_launch_file = None
        robot_args = None  # like this ["robot_model:=rx200", "use_actual := true" , "dof:=5"])

        """
        Load URDF to parameter server
        """
        load_urdf = False

        # location of the robot URDF file
        urdf_pkg_name = None
        urdf_file_name = None
        urdf_folder = "/urdf"

        # extra urdf args
        urdf_xacro_args = None

        """
        namespace of the robot
        """
        namespace = "/"

        """
        Launch the robot state publisher
        """
        # to launch robot state publisher
        launch_robot_state_pub = False

        # robot state publisher
        robot_state_publisher_max_freq = None
        new_robot_state_term = False

        """
        ROS Controllers
        """
        # loads controllers to parameter server
        load_controllers = False

        # controller (must be inside above pkg_name/config/)
        controllers_file = None
        controllers_list = None

        # Set if the controllers in "controller_list" will be reset at the beginning of each episode, default is False.
        reset_controllers = False

        # Whether to prompt the user before resetting the controllers.
        reset_controllers_prompt = False

        """
        kill rosmaster at the end of the env
        """
        kill_rosmaster = True

        """
        Clean ros Logs at the end of the env
        """
        clean_logs = False

        """
        Whether to prompt the user for resetting the environment
        """
        # uncomment if not defined as an arg in __init__
        # reset_env_prompt = False

        """
        Init GazeboBaseEnv.
        """
        super().__init__(
            load_robot=False, robot_pkg_name = None, robot_launch_file = None, robot_args = None,
            load_urdf=load_urdf, urdf_pkg_name=urdf_pkg_name, urdf_file_name=urdf_file_name,
            urdf_folder=urdf_folder, urdf_xacro_args=urdf_xacro_args, namespace=namespace,
            launch_robot_state_pub=launch_robot_state_pub,
            robot_state_publisher_max_freq=robot_state_publisher_max_freq, new_robot_state_term=new_robot_state_term,
            load_controllers=load_controllers, controllers_file=controllers_file, controllers_list=controllers_list,
            reset_controllers=reset_controllers, reset_controllers_prompt=reset_controllers_prompt,
            kill_rosmaster=kill_rosmaster, clean_logs=clean_logs, ros_port=ros_port, seed=seed,
            reset_env_prompt=reset_env_prompt)

        """
        Define ros publisher, subscribers and services for robot and sensors
        """
        # example: joint state
        # self.joint_state_sub = rospy.Subscriber(self.joint_state_topic, JointState, self.joint_state_callback)
        # self.joint_state = JointState()

        # example: moveit package
        # ros_common.ros_launch_launcher(pkg_name="interbotix_xsarm_moveit_interface",
        #                                launch_file_name="xsarm_moveit_interface.launch",
        #                                args=["robot_model:=rx200", "dof:=5", "use_python_interface:=true",
        #                                      "use_moveit_rviz:=false"])

        # example: object detection
        # ros_common.ros_launch_launcher(pkg_name="reactorx200_push_vision", launch_file_name="cube_detection.launch")
        #
        # self.simple_object_detection_sub = rospy.Subscriber('/extended_object_detection/simple_objects',
        #                                                     SimpleObjectArray, self.simple_object_detection_callback)

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
        #                                           ns="namespace")

        """
        Finished __init__ method
        """
        rospy.loginfo("End Init Custom Robot Env")

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
    #
    # # example:2
    # def check_goal(self, goal):
    #     """
    #     Check if the goal is reachable
    #     """
    #     result = self.moveit_robot_object.check_goal(goal)
    #     return result

    # ---------------------------------------------------
    #   Methods to override in Custom Robot Environment

    def _check_connection_and_readiness(self):
        """
        Function to check the connection status of subscribers, publishers and services, as well as the readiness of
        all systems.
        """
        return True

    # ---------------------------------------------------
    #    Methods to override in Custom Task Environment

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
        Get an observation from the environment.

        This method should be implemented by subclasses to return an observation representing the current state of
        the environment. The observation could be a sensor reading, a joint state, or any other type of observation
        that can be obtained from the environment.

        Returns:
            observation (Any): An observation representing the current state of the environment.
        """
        raise NotImplementedError()

    def _get_reward(self):
        """
        Function to get a reward from the environment.

        This method should be implemented by subclasses to return a scalar reward value representing how well the agent
        is doing in the current episode. The reward could be based on the distance to a goal, the amount of time taken
        to reach a goal, or any other metric that can be used to measure how well the agent is doing.

        Returns:
            A scalar reward value representing how well the agent is doing in the current episode.
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
