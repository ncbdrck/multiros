#! /usr/bin/env python

"""
Common functions for handling the physics of the MuJoCo simulator.

The MuJoCo server (mujoco_ros) exposes physics controls as ROS services under the server
node's private namespace and a dynamic_reconfigure server for the solver parameters. Unlike
Gazebo, the solver time step is primarily a property of the loaded model (the MJCF ``<option>``
element); it can be adjusted at run time through the dynamic_reconfigure interface.

Functions provided:

- ``get_mujoco_sim_info`` — get the current simulator status (real-time factor, paused, ...).
- ``set_mujoco_max_update_rate`` — set the real-time factor.
- ``get_mujoco_max_update_rate`` — get the configured real-time factor.
- ``set_mujoco_time_step`` — set the solver time step.
- ``get_mujoco_time_step`` — get the solver time step.
- ``set_mujoco_gravity`` — set the gravity vector.
- ``get_mujoco_gravity`` — get the gravity vector.
"""

import rospy
from typing import List, Optional
from multiros.utils import mujoco_core, ros_common

# These interfaces are only required for the MuJoCo backend. Import them lazily so that
# importing this module (and the wider package) does not fail on installations that only use
# the Gazebo backend.
try:
    import dynamic_reconfigure.client
    from mujoco_ros_msgs.srv import SetFloat, SetGravity, GetGravity, GetSimInfo

    _MUJOCO_MSGS_AVAILABLE = True
except ImportError:
    _MUJOCO_MSGS_AVAILABLE = False

DEFAULT_SERVER_NAME = "mujoco_server"


def _require_mujoco_msgs() -> None:
    """Raise a clear error if the mujoco_ros_pkgs Python interfaces are unavailable."""
    if not _MUJOCO_MSGS_AVAILABLE:
        raise ImportError(
            "mujoco_ros_msgs/dynamic_reconfigure could not be imported. Build mujoco_ros_pkgs "
            "in the workspace to use the MuJoCo backend."
        )


def get_mujoco_sim_info(server_name: str = DEFAULT_SERVER_NAME, ros_port: Optional[str] = None):
    """
    Function to get the current status of the MuJoCo simulator.

    Args:
        server_name (str): Graph name of the server node (optional).
        ros_port (str): The ROS_MASTER_URI port (optional).

    Returns:
        The current SimInfo of the simulator, or None on failure.
    """
    _require_mujoco_msgs()

    if ros_port is not None:
        ros_common.change_ros_master(ros_port=ros_port)

    service_name = f"/{server_name}/get_sim_info"
    try:
        rospy.wait_for_service(service_name, timeout=30.0)
    except rospy.ROSException as e:
        rospy.logerr(f"Timeout (30s) waiting for service '{service_name}': {e}")
        return None

    try:
        get_sim_info = rospy.ServiceProxy(service_name, GetSimInfo)
        response = get_sim_info()
        return response.state

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return None


def set_mujoco_max_update_rate(real_time_factor: float, server_name: str = DEFAULT_SERVER_NAME,
                               ros_port: Optional[str] = None) -> bool:
    """
    Function to set the real-time factor for MuJoCo.
    1 is real time, n is n times the rate of real time, and a negative value runs the
    simulation as fast as possible.

    Args:
        real_time_factor (float): The desired real-time factor for MuJoCo.
        server_name (str): Graph name of the server node (optional).
        ros_port (str): The ROS_MASTER_URI port (optional).

    Returns:
        bool: True if the service call was successful, False otherwise.
    """
    _require_mujoco_msgs()

    if ros_port is not None:
        ros_common.change_ros_master(ros_port=ros_port)

    service_name = f"/{server_name}/set_rt_factor"
    try:
        rospy.wait_for_service(service_name, timeout=30.0)
    except rospy.ROSException as e:
        rospy.logerr(f"Timeout (30s) waiting for service '{service_name}': {e}")
        return False

    try:
        set_rt_factor = rospy.ServiceProxy(service_name, SetFloat)
        response = set_rt_factor(value=real_time_factor, admin_hash=mujoco_core.get_admin_hash())
        return bool(response.success)

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False


def get_mujoco_max_update_rate(server_name: str = DEFAULT_SERVER_NAME,
                               ros_port: Optional[str] = None) -> Optional[float]:
    """
    Function to get the configured real-time factor for MuJoCo.

    Args:
        server_name (str): Graph name of the server node (optional).
        ros_port (str): The ROS_MASTER_URI port (optional).

    Returns:
        float: The configured real-time factor, or None on failure.
    """
    sim_info = get_mujoco_sim_info(server_name=server_name, ros_port=ros_port)
    if sim_info is None:
        return None
    return sim_info.rt_setting


def set_mujoco_time_step(time_step: float, server_name: str = DEFAULT_SERVER_NAME,
                         ros_port: Optional[str] = None) -> bool:
    """
    Function to set the solver time step for MuJoCo.

    The time step is adjusted through the simulator's dynamic_reconfigure interface.

    Args:
        time_step (float): The desired solver time step for MuJoCo.
        server_name (str): Graph name of the server node (optional).
        ros_port (str): The ROS_MASTER_URI port (optional).

    Returns:
        bool: True if the configuration was updated successfully, False otherwise.
    """
    _require_mujoco_msgs()

    if ros_port is not None:
        ros_common.change_ros_master(ros_port=ros_port)

    try:
        client = dynamic_reconfigure.client.Client(f"/{server_name}", timeout=30.0)
        client.update_configuration({"timestep": time_step})
        return True

    except Exception as e:
        rospy.logerr(f"Failed to set the MuJoCo time step: {e}")
        return False


def get_mujoco_time_step(server_name: str = DEFAULT_SERVER_NAME,
                         ros_port: Optional[str] = None) -> Optional[float]:
    """
    Function to get the solver time step for MuJoCo.

    Args:
        server_name (str): Graph name of the server node (optional).
        ros_port (str): The ROS_MASTER_URI port (optional).

    Returns:
        float: The current solver time step, or None on failure.
    """
    _require_mujoco_msgs()

    if ros_port is not None:
        ros_common.change_ros_master(ros_port=ros_port)

    try:
        client = dynamic_reconfigure.client.Client(f"/{server_name}", timeout=30.0)
        config = client.get_configuration(timeout=30.0)
        return config.get("timestep")

    except Exception as e:
        rospy.logerr(f"Failed to get the MuJoCo time step: {e}")
        return None


def set_mujoco_gravity(gravity: List[float], server_name: str = DEFAULT_SERVER_NAME,
                       ros_port: Optional[str] = None) -> bool:
    """
    Function to set the gravity vector for MuJoCo.

    Args:
        gravity (List[float]): The desired gravity vector (3 elements).
        server_name (str): Graph name of the server node (optional).
        ros_port (str): The ROS_MASTER_URI port (optional).

    Returns:
        bool: True if the service call was successful, False otherwise.
    """
    _require_mujoco_msgs()

    if ros_port is not None:
        ros_common.change_ros_master(ros_port=ros_port)

    service_name = f"/{server_name}/set_gravity"
    try:
        rospy.wait_for_service(service_name, timeout=30.0)
    except rospy.ROSException as e:
        rospy.logerr(f"Timeout (30s) waiting for service '{service_name}': {e}")
        return False

    try:
        set_gravity = rospy.ServiceProxy(service_name, SetGravity)
        response = set_gravity(gravity=gravity, admin_hash=mujoco_core.get_admin_hash())
        return bool(response.success)

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False


def get_mujoco_gravity(server_name: str = DEFAULT_SERVER_NAME,
                       ros_port: Optional[str] = None) -> Optional[List[float]]:
    """
    Function to get the gravity vector for MuJoCo.

    Args:
        server_name (str): Graph name of the server node (optional).
        ros_port (str): The ROS_MASTER_URI port (optional).

    Returns:
        list[float]: The current gravity vector, or None on failure.
    """
    _require_mujoco_msgs()

    if ros_port is not None:
        ros_common.change_ros_master(ros_port=ros_port)

    service_name = f"/{server_name}/get_gravity"
    try:
        rospy.wait_for_service(service_name, timeout=30.0)
    except rospy.ROSException as e:
        rospy.logerr(f"Timeout (30s) waiting for service '{service_name}': {e}")
        return None

    try:
        get_gravity = rospy.ServiceProxy(service_name, GetGravity)
        response = get_gravity()
        if response.success:
            return list(response.gravity)
        return None

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return None
