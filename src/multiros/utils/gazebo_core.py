#! /usr/bin/env python

"""
THE GAZEBO BRIDGE SCRIPT PROVIDES ALL THE MAIN FUNCTIONS FOR ROS TO COMMUNICATE WITH GAZEBO.

MOST OF THE ROS TOPICS & SERVICES FOR GAZEBO CAN BE FOUND HERE
http://wiki.ros.org/gazebo

AND HERE
https://classic.gazebosim.org/tutorials?tut=ros_comm&cat=connect_ros

WITH THE SCRIPT HAVE THE FOLLOWING FUNCTIONALITY,
  1. launch_gazebo: Launch Gazebo using the ROS
  2. close_gazebo: Close a gazebo instance. This function is to close both gzclient and the gzserver
  3. reset_gazebo: Reset the Gazebo simulation or world.
  4. pause_gazebo: Pause the Gazebo simulation.
  5. unpause_gazebo: Unpause the Gazebo simulation.
  6. gazebo_step: Step gazebo simulation multiple iteration.
"""

import rospy
import rospkg
import os
import subprocess
import time
from std_srvs.srv import Empty
from typing import Tuple
from multiros.utils import ros_common

"""
   1. launch_gazebo: Launch Gazebo using the ROS
"""


def launch_gazebo(launch_roscore=True, port=None, paused=False, use_sim_time=True, extra_gazebo_args=None, gui=False,
                  recording=False, debug=False, physics="ode", verbose=False, output='screen', respawn_gazebo=False,
                  pub_clock_frequency=100, server_required=False, gui_required=False, custom_world_path=None,
                  custom_world_pkg=None, custom_world_name=None,
                  launch_new_term=True) -> Tuple[str, str, subprocess.Popen]:
    """
        Launch Gazebo using the ROS.
        All the available options when launching gazebo with ros can be found in the
        /opt/ros/noetic/share/gazebo_ros/launch/empty_world.launch
        https://classic.gazebosim.org/tutorials?tut=ros_roslaunch&cat=connect_ros

        Args:
            launch_roscore (bool): if True, launch the roscore together with gazebo. Defaults to True.
            port (str): if the "launch_roscore" is "True", it launches using the the the given port.
            paused (bool):  Whether to start Gazebo in a paused state. Defaults to False.
            use_sim_time (bool): Gazebo-published simulation time, published over the ROS topic /clock (default true)
            extra_gazebo_args (str): Use this argument to add extra options to Gazebo
            gui (bool): Launch the user interface window of Gazebo (default true) (gzclient).
            recording (bool): if True, record the data from gazebo.
            debug (bool): Start gzserver (Gazebo Server) in debug mode using gdb (default false)
            physics (str): Set the physics engine (ode, dart, bullet, simbody). Currently, have support for "ode" only.
            verbose (bool): Printing errors and warnings to the terminal (default false)
            output (str): The output method for Gazebo (screen or log). Defaults to 'screen'.
            respawn_gazebo (bool): Whether to respawn Gazebo if it is killed. Defaults to False.
            pub_clock_frequency (int): The frequency of the clock publisher (in Hz). Defaults to 100.
            server_required (bool): Terminate launch script when gzserver (Gazebo Server) exits (default false)
            gui_required (bool): Terminate launch script when gzclient (user interface window) exits (default false)
            custom_world_path (str): Absolute path to the custom world file. (optional)
            custom_world_pkg (str): if the custom_world_path is None, package of the custom world file (optional)
            custom_world_name (str): if the custom_world_pkg is not None, name of the custom world file (optional)
            launch_new_term (bool): Launch the gazebo node in a new terminal (Xterm).

        Returns:
            Tuple: A tuple containing the ROS port, the Gazebo port, and the process object for the launched Gazebo instance.
        """

    rospack = rospkg.RosPack()
    try:
        rospack.get_path('gazebo_ros')
    except rospkg.common.ResourceNotFound:
        rospy.logerr("The package gazebo_ros was not found!")
        return None, None, None

    # Term command to start gazebo
    term_cmd = "roslaunch gazebo_ros empty_world.launch "

    # Start Gazebo in a paused
    term_cmd += " paused:=" + str(paused)

    # Tells ROS nodes asking for time to get the Gazebo-published simulation time, published over the ROS topic /clock
    term_cmd += " use_sim_time:=" + str(use_sim_time)

    # Add extra options to gazebo
    if extra_gazebo_args is not None:
        term_cmd += " extra_gazebo_args:=" + extra_gazebo_args

    # Launch gzclient
    term_cmd += " gui:=" + str(gui)

    # Record the data from gazebo
    term_cmd += " recording:=" + str(recording)

    # Start gzserver (Gazebo Server) in debug mode using gdb
    term_cmd += " debug:=" + str(debug)

    # change the physics engine. Currently, the gazebo_ros supports only "ode"
    if physics != "ode":
        term_cmd += " physics:=" + physics

    # if True, print debug information.
    term_cmd += " verbose:=" + str(verbose)

    # choose the output method for gazebo (screen, log).
    term_cmd += " output:=" + str(output)

    # if True, gazebo will be respawned if it is killed.
    term_cmd += " respawn_gazebo:=" + str(respawn_gazebo)

    # the frequency of the clock publisher (in Hz)
    term_cmd += " pub_clock_frequency:=" + str(pub_clock_frequency)

    # if True, the launch file will wait until gzserver is running.
    term_cmd += " server_required:=" + str(server_required)

    # if True, the launch file will wait until gzclient is running.
    term_cmd += " gui_required:=" + str(gui_required)

    # Select world
    if custom_world_path is not None:
        if os.path.exists(custom_world_path) is False:
            print("Custom World file in " + custom_world_path + " does not exists!")
            return None, None, None
        term_cmd += " world_name:=" + str(custom_world_path)

    elif custom_world_pkg and custom_world_name is not None:
        try:
            world_pkg_path = rospack.get_path(custom_world_pkg)
        except rospkg.common.ResourceNotFound:
            rospy.logwarn("Package where world file is located was NOT FOUND!")
            return None, None, None

        world_file_path = world_pkg_path + "/worlds/" + custom_world_name
        if os.path.exists(world_file_path) is False:
            print("Custom World file in " + world_file_path + " does not exists!")
            return None, None, None
        term_cmd += " world_name:=" + str(world_file_path)

    # initializing the var for "launch_roscore"
    ros_port = None
    gazebo_port = None

    if launch_roscore:
        if port is not None:
            ros_port, gazebo_port = ros_common.launch_roscore(port=int(port))
        else:
            ros_port, gazebo_port = ros_common.launch_roscore()

    # Launch gazebo
    if launch_new_term:
        term_cmd = f"xterm -e '{term_cmd}'"
        # print("term_cmd:", term_cmd)
        process = subprocess.Popen(term_cmd, shell=True)

    else:
        # print("term_cmd:", term_cmd)
        process = subprocess.Popen(term_cmd, shell=True)

    time.sleep(5.0)

    # waiting for gazebo to done launching
    rospy.wait_for_service('/gazebo/pause_physics')

    # if launch_roscore in False, ignore the first two returns
    return ros_port, gazebo_port, process


"""
   2. close_gazebo: close a gazebo instance. This function is to close both gzclient and the gzserver
"""


def close_gazebo(process: subprocess.Popen, ros_port=None, gazebo_port=None) -> bool:
    """
    Function to close a gazebo instance. This function is to close both gzclient and the gzserver

    Args:
        process: A subprocess.Popen object representing the running Gazebo instance.
        ros_port (str): The ROS port used by the Gazebo instance. Defaults to None. (optional).
        gazebo_port (str): The Gazebo port used by the Gazebo instance. Defaults to None. (optional).

    Returns:
        bool: True if Gazebo was closed successfully, False otherwise.
    """

    # change the rosmaster
    if ros_port is not None:
        ros_common.change_ros_gazebo_master(ros_port=ros_port, gazebo_port=gazebo_port)

    process.terminate()

    rospy.logdebug("Closing Gazebo!")

    return True


"""
   3. reset_gazebo: Reset the Gazebo simulation or world. 
"""


def reset_gazebo(reset_type: str = "simulation", max_tries: int = 5, ros_port: str = None,
                 gazebo_port: str = None) -> bool:
    """
    Function to reset the Gazebo simulation or world.

    Args:
        reset_type (str): The type of reset to perform ("simulation" or "world") (optional).
        max_tries (int): The maximum number of tries to reset the simulation or world (optional).
        ros_port (str): The ROS_MASTER_URI port (optional).
        gazebo_port (str): The GAZEBO_MASTER_URI port (optional).

    Returns:
        bool: True if the service call was successful, False otherwise.
    """

    # Change the ROS and Gazebo master environment variables if provided
    if ros_port is not None:
        ros_common.change_ros_gazebo_master(ros_port=ros_port, gazebo_port=gazebo_port)

    # Determine the service name based on the reset_type
    service_name = "/gazebo/reset_simulation" if reset_type == "simulation" else "/gazebo/reset_world"

    # Wait for the service to be available
    rospy.wait_for_service(service_name)

    # Try to reset the simulation or world up to max_tries times
    for i in range(max_tries):
        try:
            # Create a service proxy for the service
            reset_service = rospy.ServiceProxy(service_name, Empty)

            # Call the service to reset the simulation or world
            reset_service()

            rospy.loginfo(f"Reset {reset_type} successful!")
            return True

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    return False


"""
   4. pause_gazebo: Pause the Gazebo simulation.
"""


def pause_gazebo(max_tries: int = 5, ros_port: str = None, gazebo_port: str = None) -> bool:
    """
    Function to pause the Gazebo simulation.

    Args:
        max_tries (int): The maximum number of tries to pause the simulation (optional).
        ros_port (str): The ROS_MASTER_URI port (optional).
        gazebo_port (str): The GAZEBO_MASTER_URI port (optional).

    Returns:
        bool: True if the service call was successful, False otherwise.
    """

    # Change the ROS and Gazebo master environment variables if provided
    if ros_port is not None:
        ros_common.change_ros_gazebo_master(ros_port=ros_port, gazebo_port=gazebo_port)

    # Wait for the 'pause_physics' service to be available
    rospy.wait_for_service('/gazebo/pause_physics')

    # Try to pause the simulation up to max_tries times
    for i in range(max_tries):
        try:
            # Create a service proxy for the 'pause_physics' service
            pause_physics = rospy.ServiceProxy('/gazebo/pause_physics', Empty)

            # Call the 'pause_physics' service to pause the simulation
            pause_physics()
            rospy.logdebug(f"Pause successful!")
            return True

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    return False


"""
   5. unpause_gazebo: Unpause the Gazebo simulation.
"""


def unpause_gazebo(max_tries: int = 5, ros_port: str = None, gazebo_port: str = None) -> bool:
    """
    Function to unpause the Gazebo simulation.

    Args:
        max_tries (int): The maximum number of tries to unpause the simulation (optional).
        ros_port (str): The ROS_MASTER_URI port (optional).
        gazebo_port (str): The GAZEBO_MASTER_URI port (optional).

    Returns:
        bool: True if the service call was successful, False otherwise.
    """

    # Change the ROS and Gazebo master environment variables if provided
    if ros_port is not None:
        ros_common.change_ros_gazebo_master(ros_port=ros_port, gazebo_port=gazebo_port)

    # Wait for the 'unpause_physics' service to be available
    rospy.wait_for_service('/gazebo/unpause_physics')

    # Try to unpause the simulation up to max_tries times
    for i in range(max_tries):
        try:
            # Create a service proxy for the 'unpause_physics' service
            unpause_physics = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)

            # Call the 'unpause_physics' service to unpause the simulation
            unpause_physics()
            rospy.logdebug(f"Unpause successful!")
            return True

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    return False


def gazebo_step(steps: int, ros_port: str = None, gazebo_port: str = None) -> bool:
    """
    Function to Step gazebo simulation multiple iteration.

    https://manpages.ubuntu.com/manpages/focal/man1/gz.1.html

    Args:
        steps (int): The number of steps to advance the simulation.
        ros_port (str): The ROS_MASTER_URI port (optional).
        gazebo_port (str): The GAZEBO_MASTER_URI port (optional).

    Returns:
        bool: True if the task is completed successfully, False otherwise.
    """

    # Change the ROS and Gazebo master environment variables if provided
    if ros_port is not None:
        ros_common.change_ros_gazebo_master(ros_port=ros_port, gazebo_port=gazebo_port)

    try:
        term_cmd = "gz world -m " + str(steps)
        subprocess.Popen(term_cmd, shell=True).wait()
        return True
    except Exception as e:
        rospy.logerr(f"An error occurred while stepping the physics in Gazebo: {e}")
        return False
