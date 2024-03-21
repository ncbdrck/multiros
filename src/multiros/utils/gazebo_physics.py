#! /usr/bin/env python

"""
This script is to specify all the common functions related to the handling physics of Gazebo Simulator.
It has the following functions,
  01. get_gazebo_physics_properties: Get the current physics properties of Gazebo.
  02. set_gazebo_physics_properties: Set the physics properties of Gazebo.
  03. get_gazebo_max_update_rate: Get the current maximum update rate for Gazebo.
  04. set_gazebo_max_update_rate: Set the maximum update rate for Gazebo in a real-time factor.
  05. get_gazebo_time_step: Get the current time step for Gazebo.
  06. set_gazebo_time_step: Set the time step for Gazebo.
  07. get_gazebo_gravity: Get the current gravity vector for Gazebo.
  08. set_gazebo_gravity: Set the gravity vector for Gazebo.
  09. get_gazebo_ode_physics: Get the current ODE physics properties of Gazebo.
  10. set_gazebo_ode_physics: Set the ODE physics properties of Gazebo.

  ROS Docs:
  http://docs.ros.org/en/diamondback/api/gazebo/html/srv/GetPhysicsProperties.html
  http://docs.ros.org/en/diamondback/api/gazebo/html/srv/SetPhysicsProperties.html
  http://docs.ros.org/en/diamondback/api/gazebo/html/msg/ODEPhysics.html
"""

import rospy
from gazebo_msgs.srv import GetPhysicsProperties, SetPhysicsProperties, SetPhysicsPropertiesRequest
from gazebo_msgs.msg import ODEPhysics
from multiros.utils import ros_common
from typing import List


def get_gazebo_physics_properties(ros_port: str = None, gazebo_port: str = None):
    """
    Function to get the current physics properties of Gazebo.

    Args:
        ros_port (str): The ROS_MASTER_URI port (optional).
        gazebo_port (str): The GAZEBO_MASTER_URI port (optional).

    Returns:
        physics_properties: The current physics properties of Gazebo.
    """

    # Change the ROS and Gazebo master environment variables if provided
    if ros_port is not None:
        ros_common.change_ros_gazebo_master(ros_port=ros_port, gazebo_port=gazebo_port)

    # Wait for the 'get_physics_properties' service to be available
    rospy.wait_for_service('/gazebo/get_physics_properties')

    try:
        # Create a service proxy for the 'get_physics_properties' service
        get_physics_properties = rospy.ServiceProxy('/gazebo/get_physics_properties', GetPhysicsProperties)

        # Call the 'get_physics_properties' service and get the current physics properties of Gazebo
        physics_properties = get_physics_properties()

        return physics_properties

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")


def set_gazebo_physics_properties(time_step: float = None, max_update_rate: float = None, gravity: list = None,
                                  ode_config: ODEPhysics = None, ros_port: str = None, gazebo_port: str = None) -> bool:
    """
    Function to set the physics properties of Gazebo.

    Args:
        time_step (float): The desired time step for Gazebo (optional).
        max_update_rate (float): The desired maximum update rate for Gazebo (optional).
        gravity (list): The desired gravity vector for Gazebo (optional).
        ode_config (ODEPhysics): The desired ODE physics properties for Gazebo (optional).
        ros_port (str): The ROS_MASTER_URI port (optional).
        gazebo_port (str): The GAZEBO_MASTER_URI port (optional).

    Returns:
        bool: True if the service call was successful, False otherwise.
    """

    # Change the ROS and Gazebo master environment variables if provided
    if ros_port is not None:
        ros_common.change_ros_gazebo_master(ros_port=ros_port, gazebo_port=gazebo_port)

    # Get the current physics properties of Gazebo
    physics_properties = get_gazebo_physics_properties()

    # Create a SetPhysicsPropertiesRequest object
    set_physics_properties_request = SetPhysicsPropertiesRequest()
    set_physics_properties_request.time_step = time_step if time_step is not None else physics_properties.time_step
    set_physics_properties_request.max_update_rate = max_update_rate if max_update_rate is not None else physics_properties.max_update_rate
    set_physics_properties_request.gravity = gravity if gravity is not None else physics_properties.gravity
    set_physics_properties_request.ode_config = ode_config if ode_config is not None else physics_properties.ode_config

    # Wait for the 'set_physics_properties' service to be available
    rospy.wait_for_service('/gazebo/set_physics_properties')
    try:
        # Create a service proxy for the 'set_physics_properties' service
        set_physics_properties = rospy.ServiceProxy('/gazebo/set_physics_properties', SetPhysicsProperties)

        # Call the 'set_physics_properties' service and set the physics properties for Gazebo
        set_physics_properties(set_physics_properties_request)

        return True

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False


def get_gazebo_max_update_rate(ros_port: str = None, gazebo_port: str = None) -> float:
    """
    Function to get the current maximum update rate for Gazebo.

    Args:
        ros_port (str): The ROS_MASTER_URI port (optional).
        gazebo_port (str): The GAZEBO_MASTER_URI port (optional).

    Returns:
        float: The current maximum update rate for Gazebo.
    """

    # Change the ROS and Gazebo master environment variables if provided
    if ros_port is not None:
        ros_common.change_ros_gazebo_master(ros_port=ros_port, gazebo_port=gazebo_port)

    # Get the current physics properties of Gazebo
    physics_properties = get_gazebo_physics_properties()

    # Return the current maximum update rate for Gazebo
    return physics_properties.max_update_rate


def set_gazebo_max_update_rate(real_time_factor: float, ros_port: str = None, gazebo_port: str = None) -> bool:
    """
    Function to set the maximum update rate for Gazebo in real-time factor.
    1 is real time, n is n times the rate of real time.

    Args:
        real_time_factor (float): The desired real-time factor for Gazebo.
        ros_port (str): The ROS_MASTER_URI port (optional).
        gazebo_port (str): The GAZEBO_MASTER_URI port (optional).

    Returns:
        bool: True if the service call was successful, False otherwise.
    """

    # Change the ROS and Gazebo master environment variables if provided
    if ros_port is not None:
        ros_common.change_ros_gazebo_master(ros_port=ros_port, gazebo_port=gazebo_port)

    # Get the current physics properties of Gazebo
    physics_properties = get_gazebo_physics_properties()

    # Calculate the maximum update rate for Gazebo in real-time factor
    max_update_rate = ((1.0 / physics_properties.time_step) * real_time_factor)

    # Create a SetPhysicsPropertiesRequest object
    set_physics_properties_request = SetPhysicsPropertiesRequest()
    set_physics_properties_request.time_step = physics_properties.time_step
    set_physics_properties_request.max_update_rate = max_update_rate
    set_physics_properties_request.gravity = physics_properties.gravity
    set_physics_properties_request.ode_config = physics_properties.ode_config

    # Wait for the 'set_physics_properties' service to be available
    rospy.wait_for_service('/gazebo/set_physics_properties')
    try:
        # Create a service proxy for the 'set_physics_properties' service
        set_physics_properties = rospy.ServiceProxy('/gazebo/set_physics_properties', SetPhysicsProperties)

        # Call the 'set_physics_properties' service and set the maximum update rate for Gazebo in real-time factor
        set_physics_properties(set_physics_properties_request)

        return True

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False


def get_gazebo_time_step(ros_port: str = None, gazebo_port: str = None) -> float:
    """
    Function to get the current time step for Gazebo.

    Args:
        ros_port (str): The ROS_MASTER_URI port (optional).
        gazebo_port (str): The GAZEBO_MASTER_URI port (optional).

    Returns:
        float: The current time step for Gazebo.
    """

    # Change the ROS and Gazebo master environment variables if provided
    if ros_port is not None:
        ros_common.change_ros_gazebo_master(ros_port=ros_port, gazebo_port=gazebo_port)

    # Get the current physics properties of Gazebo
    physics_properties = get_gazebo_physics_properties()

    # Return the current time step for Gazebo
    return physics_properties.time_step


def set_gazebo_time_step(time_step: float, ros_port: str = None, gazebo_port: str = None) -> bool:
    """
    Function to set the time step for Gazebo.

    Args:
        time_step (float): The desired time step for Gazebo.
        ros_port (str): The ROS_MASTER_URI port (optional).
        gazebo_port (str): The GAZEBO_MASTER_URI port (optional).

    Returns:
        bool: True if the service call was successful, False otherwise.
    """

    # Change the ROS and Gazebo master environment variables if provided
    if ros_port is not None:
        ros_common.change_ros_gazebo_master(ros_port=ros_port, gazebo_port=gazebo_port)

    # Get the current physics properties of Gazebo
    physics_properties = get_gazebo_physics_properties()

    # Create a SetPhysicsPropertiesRequest object
    set_physics_properties_request = SetPhysicsPropertiesRequest()
    set_physics_properties_request.time_step = time_step
    set_physics_properties_request.max_update_rate = physics_properties.max_update_rate
    set_physics_properties_request.gravity = physics_properties.gravity
    set_physics_properties_request.ode_config = physics_properties.ode_config

    # Wait for the 'set_physics_properties' service to be available
    rospy.wait_for_service('/gazebo/set_physics_properties')
    try:
        # Create a service proxy for the 'set_physics_properties' service
        set_physics_properties = rospy.ServiceProxy('/gazebo/set_physics_properties', SetPhysicsProperties)

        # Call the 'set_physics_properties' service and set the time step for Gazebo
        set_physics_properties(set_physics_properties_request)

        return True

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False


def get_gazebo_gravity(ros_port: str = None, gazebo_port: str = None) -> List[float]:
    """
    Function to get the current gravity vector for Gazebo.

    Args:
        ros_port (str): The ROS_MASTER_URI port (optional).
        gazebo_port (str): The GAZEBO_MASTER_URI port (optional).

    Returns:
        list[float]: The current gravity vector for Gazebo.
    """

    # Change the ROS and Gazebo master environment variables if provided
    if ros_port is not None:
        ros_common.change_ros_gazebo_master(ros_port=ros_port, gazebo_port=gazebo_port)

    # Get the current physics properties of Gazebo
    physics_properties = get_gazebo_physics_properties()

    # Return the current gravity vector for Gazebo
    return physics_properties.gravity


def set_gazebo_gravity(gravity: List[float], ros_port: str = None, gazebo_port: str = None) -> bool:
    """
    Function to set the gravity vector for Gazebo.

    Args:
        gravity (List[float]): The desired gravity vector for Gazebo.
        ros_port (str): The ROS_MASTER_URI port (optional).
        gazebo_port (str): The GAZEBO_MASTER_URI port (optional).

    Returns:
        bool: True if the service call was successful, False otherwise.
    """

    # Change the ROS and Gazebo master environment variables if provided
    if ros_port is not None:
        ros_common.change_ros_gazebo_master(ros_port=ros_port, gazebo_port=gazebo_port)

    # Get the current physics properties of Gazebo
    physics_properties = get_gazebo_physics_properties()

    # Create a SetPhysicsPropertiesRequest object
    set_physics_properties_request = SetPhysicsPropertiesRequest()
    set_physics_properties_request.time_step = physics_properties.time_step
    set_physics_properties_request.max_update_rate = physics_properties.max_update_rate
    set_physics_properties_request.gravity = gravity
    set_physics_properties_request.ode_config = physics_properties.ode_config

    # Wait for the 'set_physics_properties' service to be available
    rospy.wait_for_service('/gazebo/set_physics_properties')
    try:
        # Create a service proxy for the 'set_physics_properties' service
        set_physics_properties = rospy.ServiceProxy('/gazebo/set_physics_properties', SetPhysicsProperties)

        # Call the 'set_physics_properties' service and set the gravity vector for Gazebo
        set_physics_properties(set_physics_properties_request)

        return True

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False


def get_gazebo_ode_physics(ros_port: str = None, gazebo_port: str = None):
    """
    Function to get the current ODE physics properties of Gazebo.

    Args:
        ros_port (str): The ROS_MASTER_URI port (optional).
        gazebo_port (str): The GAZEBO_MASTER_URI port (optional).

    Returns:
        ode_config: The current ODE physics properties of Gazebo.
    """

    # Change the ROS and Gazebo master environment variables if provided
    if ros_port is not None:
        ros_common.change_ros_gazebo_master(ros_port=ros_port, gazebo_port=gazebo_port)

    # Get the current physics properties of Gazebo
    physics_properties = get_gazebo_physics_properties()

    return physics_properties.ode_config


def set_gazebo_ode_physics(ode_config: ODEPhysics, ros_port: str = None, gazebo_port: str = None) -> bool:
    """
    Function to set the ODE physics properties of Gazebo.

    Args:
        ode_config (ODEPhysics): The desired ODE physics properties for Gazebo.
        ros_port (str): The ROS_MASTER_URI port (optional).
        gazebo_port (str): The GAZEBO_MASTER_URI port (optional).

    Returns:
        bool: True if the service call was successful, False otherwise.
    """

    # Change the ROS and Gazebo master environment variables if provided
    if ros_port is not None:
        ros_common.change_ros_gazebo_master(ros_port=ros_port, gazebo_port=gazebo_port)

    # Get the current physics properties of Gazebo
    physics_properties = get_gazebo_physics_properties()

    # Create a SetPhysicsPropertiesRequest object
    set_physics_properties_request = SetPhysicsPropertiesRequest()
    set_physics_properties_request.time_step = physics_properties.time_step
    set_physics_properties_request.max_update_rate = physics_properties.max_update_rate
    set_physics_properties_request.gravity = physics_properties.gravity
    set_physics_properties_request.ode_config = ode_config

    # Wait for the 'set_physics_properties' service to be available
    rospy.wait_for_service('/gazebo/set_physics_properties')
    try:
        # Create a service proxy for the 'set_physics_properties' service
        set_physics_properties = rospy.ServiceProxy('/gazebo/set_physics_properties', SetPhysicsProperties)

        # Call the 'set_physics_properties' service and set the ODE physics properties for Gazebo
        set_physics_properties(set_physics_properties_request)

        return True

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False
