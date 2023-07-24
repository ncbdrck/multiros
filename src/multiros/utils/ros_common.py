#! /usr/bin/env python

"""
This script is to specify all the common functions related to the handling of ROS.
It has the following functions,
    01. launch_roscore: Launch a rocore with a given or random (no overlapping) port
    02. change_ros_gazebo_master: Change the current ROS and Gazebo Master Environment Variables
    03. get_all_the_ros_masters: Get all the current ros master ports
    04. add_to_rosmaster_list: Add a port to the current ros master ports list
    05. remove_from_rosmaster_list: remove a port from the current ros master ports list
    06. kill_all_ros_and_gazebo: Kill all the ros and gazebo related instances.
    07. kill_all_roslaunch_process: Kill all roslaunch processes.
    08. kill_all_ros_nodes: Kill all running ROS nodes of a specified or the current ROS master.
    09. kill_ros_node: Kill a single ROS node of a given or current ROS master.
    10. ros_kill_master: Kill a ROS master
    11. clean_ros_logs: Clean all the ros logs
    12. source_workspace: Source a ros workspace
    13. ros_launch_launcher: Execute a roslaunch with args
    14. ros_node_launcher: Launch a ROS node from a package
    15. ros_load_yaml: Fetch a YAML file from a package or a path and load it into the ROS Parameter Server.
    16. load_urdf: Load a URDF into the parameter server or a string containing the processed URDF data
    17. is_roscore_running: Check if a roscore is currently running
    18. change_ros_master: Set the current ROS Master Environment Variable.
    19. change_gazebo_master: Set the current Gazebo Master Environment Variable
    20. init_robot_state_publisher: Initialize the robot state publisher.
    21. remove_all_from_rosmaster_list: Remove all ports from the Multiros rosmaster port list.

"""
import rosparam
import rospy
import rospkg
import os
import subprocess
import time
import random
import xacro
from typing import Tuple, Union

"""
    01. launch_roscore: To launch a rocore with a given or random (no overlapping) port
"""


def launch_roscore(port: int = None, set_new_master_vars: bool = True) -> Tuple[str, str]:
    """
    Function to launch a roscore. It launches a roscore and sets a temporary variable in a file to track all the
    current python program's opened roscores. This allows the script to launch multiple gazebo instances in parallel and
    communicate with them without interference.

    Args:
        port (int): A port for the ROS_MASTER_URI (optional).
        set_new_master_vars (bool): change the current ROS_MASTER anb the GAZEBO_MASTER to the selected one

    Returns:
        str: Ports selected for the ROS_MASTER_URI and GAZEBO_MASTER_URI.
    """

    # Define the path to the file where the opened ROS_MASTER ports are will be stored
    env_file_path = '/tmp/ros_master_ports_multiros.txt'

    # Get the currently opened rosmaster ports from the temp file
    if os.path.exists(env_file_path):
        with open(env_file_path, 'r') as f:
            ROSMASTER_LIST = f.read().strip()
        all_ros_masters = list(ROSMASTER_LIST.split(" "))
        print("Currently opened ROSMASTER ports: ", all_ros_masters)
        rospy.loginfo("Currently opened ROSMASTER ports: " + str(ROSMASTER_LIST))

    # if there is no such a file, create one
    else:
        print("No existing ROSMASTER ports!")
        ROSMASTER_LIST = '11311'  # Default port for ROS. So better to not select it
        all_ros_masters = list(ROSMASTER_LIST.split(" "))
        with open(env_file_path, 'w') as f:
            f.write(ROSMASTER_LIST)
        rospy.loginfo("Created /tmp/ros_master_ports_multiros.txt!")

    # done is True if the selected or given port is not in the temp file
    done = False

    # Find and launch the ros master with the given port if it is not already in use
    if port is not None:
        ports = [str(port), str(port - 1), str(port + 1)]
        if not any(item in ports for item in all_ros_masters):
            done = True
            ros_port = str(port)
            gazebo_port = str(port + 1)

            # add the new port to the temp file
            ROSMASTER_LIST = ROSMASTER_LIST + ' ' + ros_port
            with open(env_file_path, 'w') as f:
                f.write(ROSMASTER_LIST)
            rospy.logdebug(f"Updated the ROSMASTER port list with port {ros_port}!")

        else:
            rospy.logwarn("The port already exists! So launching rosmaster with a new port")
            print("The port already exists! So launching with a new port")

    # If not done, try to find an unused port by generating random port numbers until one is found that is
    # not already in use
    while not done:
        randn = random.randint(11300, 12400)

        # Since we are allocating the gazebo port just after the ros port, we need to check
        values = [str(randn), str(randn - 1), str(randn + 1)]

        if not any(item in values for item in all_ros_masters):
            done = True
            ros_port = str(randn)
            gazebo_port = str(randn + 1)

            # add the new port to the variable
            ROSMASTER_LIST = ROSMASTER_LIST + ' ' + ros_port
            with open(env_file_path, 'w') as f:
                f.write(ROSMASTER_LIST)
            rospy.logdebug(f"updated the ROSMASTER port list with port {ros_port}!")

    # launch roscore as a term command
    term_cmd = "roscore -p " + ros_port
    term_cmd = "xterm -e ' " + term_cmd + "'"
    subprocess.Popen(term_cmd, shell=True)
    time.sleep(5.0)
    rospy.loginfo("Roscore launched! with port: " + ros_port)

    # change the current ROS_MASTER anb the GAZEBO_MASTER to the selected one
    if set_new_master_vars:
        change_ros_gazebo_master(ros_port, gazebo_port)

    return ros_port, gazebo_port


"""
    02. change_ros_gazebo_master: Change the current ROS and Gazebo Master Environment Variables
"""


def change_ros_gazebo_master(ros_port: str, gazebo_port: str = None) -> bool:
    """
    Function to set the current ROS and Gazebo Master Environment Variables.
    This is for situations where we need to spawn multiple gazebo instances
    :website: https://wiki.ros.org/ROS/EnvironmentVariables

    Args:
        ros_port (str): The ROS_MASTER_URI port.
        gazebo_port (str): The GAZEBO_MASTER_URI port (optional).

    Returns:
        bool: True if ROS and Gazebo Master Environment Variables are set to new values.
    """

    # only update the ros master if gazebo_port is not given
    if gazebo_port is None:
        return change_ros_master(ros_port)

    else:
        # Temporarily assigning values for the ROS_MASTER_URI and GAZEBO_MASTER_URI environment variables
        os.environ["ROS_MASTER_URI"] = f"http://localhost:{ros_port}"
        os.environ["GAZEBO_MASTER_URI"] = f"http://localhost:{gazebo_port}"

        rospy.logdebug(f"Changed ROS_MASTER_URI to port: {ros_port}")
        rospy.logdebug(f"Changed GAZEBO_MASTER_URI to port: {gazebo_port}")

    return True


"""
    03. get_all_the_ros_masters: Get all the current ros master ports
"""


def get_all_the_ros_masters() -> str:
    """
    Function to get all the active Rosmasters that were made during the execution of the script.

    Returns:
        str: List of all the active rosmaster ports.
    """

    # Define the path to the file where the all the active rosmaster ports will be stored
    env_file_path = '/tmp/ros_master_ports_multiros.txt'

    # Get the currently opened rosmaster ports from the temp file
    if os.path.exists(env_file_path):
        with open(env_file_path, 'r') as f:
            rosmaster_list = f.read().strip()

    else:
        print("No existing Multiros ROSMASTER ports!")
        rosmaster_list = '11311'  # Default port for ROS. So better to not select it
        with open(env_file_path, 'w') as f:
            f.write(rosmaster_list)
        rospy.loginfo("Created /tmp/ros_master_ports_multiros.txt!")

    return rosmaster_list


"""
    04. add_to_rosmaster_list: Add a port to the current ros master ports list
"""


def add_to_rosmaster_list(ros_port: str) -> bool:
    """
    Function to add the specified port to the Multiros rosmaster port list.

    Args:
        ros_port (str): The ROS_MASTER_URI port that needs to be added.

    Returns:
        bool: True if the port was added successfully.
    """

    # Define the path to the file where the all the active rosmaster ports will be stored
    env_file_path = '/tmp/ros_master_ports_multiros.txt'

    rosmaster_list = get_all_the_ros_masters()

    ROSMASTER_LIST = rosmaster_list + ' ' + ros_port

    with open(env_file_path, 'w') as f:
        f.write(ROSMASTER_LIST)

    rospy.logdebug("Updated the temp file with the port: " + ros_port)

    return True


"""
    05. remove_from_rosmaster_list: remove a port from the current ros master ports list
"""


def remove_from_rosmaster_list(ros_port: str) -> bool:
    """
    Function to remove the specified port from the Multiros rosmaster port list.

    Args:
        ros_port (str): The ROS_MASTER_URI port that needs to be removed.

    Returns:
        bool: True if the port was removed successfully, False otherwise.
    """

    # Define the path to the file where the all the active rosmaster ports will be stored
    env_file_path = '/tmp/ros_master_ports_multiros.txt'

    ROSMASTER_LIST = get_all_the_ros_masters()

    if ros_port in ROSMASTER_LIST:
        edited_ros_master_list = ROSMASTER_LIST.replace(ros_port, '')

        with open(env_file_path, 'w') as f:
            f.write(edited_ros_master_list)

        rospy.loginfo("Removed the port: " + ros_port + " from the Multiros rosmaster port list!")

        return True
    else:
        print("Given port: " + ros_port + " doesn't exist in the active rosmaster port list!")
        return False


"""
    06. kill_all_ros_and_gazebo: Kill all the ros and gazebo related instances.
"""


def kill_all_ros_and_gazebo() -> bool:
    """
    Function to kill all the ros-related processes, including all the gazebo and moveit processes.
    Note: This process kills all Rosmasters and gazebo instances

    Returns:
        bool: True if all the applications were closed.
    """

    term_cmd = "killall -9 rosmaster roslaunch gzserver rosout gzclient robot_state_publisher nodelet"
    subprocess.Popen("xterm -e ' " + term_cmd + "'", shell=True).wait()

    # term_cmd = "pkill -f ros"
    # subprocess.Popen("xterm -e ' " + term_cmd + "'", shell=True).wait()

    # remove all the rosmasters
    remove_all_from_rosmaster_list()

    rospy.logdebug("Successfully killed all the active ROS related instances!")

    return True


"""
    07. kill_all_roslaunch_process: Kill all roslaunch processes.
"""


def kill_all_roslaunch_process() -> bool:
    """
    Function to kill all roslaunch processes.
    Note: Kills all the launchers not on one rosmaster

    Returns:
        bool: True if all the roslaunch processes were killed.
    """

    term_cmd = "killall -9 roslaunch"
    subprocess.Popen("xterm -e ' " + term_cmd + "'", shell=True).wait()
    rospy.logdebug("Successfully killed all the active roslaunch instances!")

    return True


"""
    08. kill_all_ros_nodes: Kill all running ROS nodes of a specified or the current ROS master.
"""


def kill_all_ros_nodes(ros_port=None, gazebo_port=None) -> bool:
    """
    Function to kill all running ROS nodes of the specified or current Rosmaster.

    Args:
        ros_port (str): The ROS_MASTER_URI port (optional).
        gazebo_port (str): The GAZEBO_MASTER_URI port (optional).

    Returns:
        bool: True if all nodes were killed, False otherwise.
    """

    if ros_port is not None:
        change_ros_gazebo_master(ros_port=ros_port, gazebo_port=gazebo_port)

    term_cmd = "rosnode kill -a"
    subprocess.Popen("xterm -e ' " + term_cmd + "'", shell=True).wait()

    rospy.logdebug("Successfully killed all the active nodes instances!")

    # Remove the current ros master from the ROS_MASTER port list
    if ros_port is not None:
        remove_from_rosmaster_list(ros_port)

    return True


"""
    09. kill_ros_node: Kill a single ROS node of a given or current ROS master.
"""


def kill_ros_node(node_name, ros_port=None, gazebo_port=None) -> bool:
    """
    Function to kill a ROS node of the given or current Rosmaster.

    Args:
        node_name (str): Name of the node to kill.
        ros_port (str): The ROS_MASTER_URI port (optional).
        gazebo_port (str): The GAZEBO_MASTER_URI port (optional).

    Returns:
        bool: True if the node was killed, False otherwise.
    """

    if ros_port is not None:
        change_ros_gazebo_master(ros_port=ros_port, gazebo_port=gazebo_port)

    term_cmd = "rosnode kill " + node_name
    subprocess.Popen("xterm -e ' " + term_cmd + "'", shell=True).wait()

    rospy.logdebug(f"Successfully killed the node: {node_name}!")

    # Remove the current ros master from the ROS_MASTER port list
    if node_name == "/rosout" and ros_port is not None:
        remove_from_rosmaster_list(ros_port)

    return True


"""
    10. ros_kill_master: To kill a ROS master
"""


def ros_kill_master(ros_port) -> bool:
    """
    Function to kill a ROS master.

    Args:
        ros_port (str): The ROS_MASTER_URI port.

    Returns:
        bool: True if the master was killed, False otherwise.
    """

    term_cmd = "pkill -f 'roscore -p " + ros_port + "'"
    subprocess.Popen("xterm -e ' " + term_cmd + "'", shell=True).wait()
    time.sleep(0.5)

    rospy.logdebug(f"Successfully killed ROS master: {ros_port}!")

    # Remove the ros master from the ROS_MASTER port list
    remove_from_rosmaster_list(ros_port)

    return True


"""
    11. clean_ros_logs: To clean all the ros logs
"""


def clean_ros_logs() -> bool:
    """
    Function to clean all the ros logs.

    Returns:
        bool: True if all the ros logs were closed.
    """

    term_cmd = "rosclean purge -y"
    subprocess.Popen("xterm -e ' " + term_cmd + "'", shell=True).wait()
    rospy.logdebug("successfully cleaned all the ROS logs")

    return True


"""
    12. source_workspace: To source a ros workspace
"""


def source_workspace(abs_path) -> bool:
    """
    Function to source the ros workspace.

    Args:
        abs_path (str): Absolute path of the ros workspace.

    Returns:
        bool: True if all the ros logs were closed and False otherwise.
    """

    if os.path.exists(abs_path) is False:
        rospy.logwarn("A ROS workspace does not exists in the: " + abs_path + "\n" +
                      "This should be the *absolute* path of the workspace!")
        return False

    elif os.path.exists(abs_path + "/devel/") is False:
        rospy.logwarn("devel folder does not exists in the: " + abs_path)
        return False

    term_cmd = "cd " + abs_path + ";"
    term_cmd = term_cmd + "source devel/setup.bash"
    subprocess.Popen("xterm -e ' " + term_cmd + "'", shell=True).wait()

    term_cmd = "rospack profile"
    subprocess.Popen("xterm -e ' " + term_cmd + "'", shell=True).wait()

    rospy.loginfo("successfully sourced the ROS workspace!")

    return True


"""
    13. ros_launch_launcher: To execute a roslaunch with args
"""


def ros_launch_launcher(pkg_name=None, launch_file_name=None, launch_file_abs_path=None, args=None,
                        launch_new_term=True, ros_port=None, gazebo_port=None) -> bool:
    """
    Function to execute a roslaunch with args.

    Args:
        pkg_name (str): Name of the package where the launch file is located.
        launch_file_name (str): Name of the launch file.
        launch_file_abs_path (str): Absolute path of the launch file
        args (list of str): Args to pass to the launch file.
        launch_new_term (bool): Launch the process in a new terminal (Xterm).
        ros_port (str): The ROS_MASTER_URI port (optional).
        gazebo_port (str): The GAZEBO_MASTER_URI port (optional).

    Returns:
        bool: True if the launch file was launched and False otherwise.
    """

    # change the rosmaster
    if ros_port is not None:
        change_ros_gazebo_master(ros_port=ros_port, gazebo_port=gazebo_port)

    term_cmd = construct_roslaunch_command(pkg_name, launch_file_name, launch_file_abs_path)

    if term_cmd is None:
        print("Launch Failed! Requires either the absolute path or the pkg_name and the launch_file_name as input!")
        return False

    if args is not None:
        term_cmd += " " + " ".join(args)

    if launch_new_term:
        term_cmd = "xterm -e ' " + term_cmd + "'"

    subprocess.Popen(term_cmd, shell=True)
    time.sleep(5.0)

    return True


# helper fn for ros_launch_launcher
def construct_roslaunch_command(pkg_name, launch_file_name, launch_file_abs_path):
    """
    Constructs a roslaunch command using either a package name and launch file name or an absolute path to a launch file.

    Args:
        pkg_name (str): Name of the package where the launch file is located.
        launch_file_name (str): Name of the launch file.
        launch_file_abs_path (str): Absolute path of the launch file

    Returns:
        str: The constructed roslaunch command or None if no valid inputs were provided.
    """

    # roslaunch from package
    if pkg_name and launch_file_name is not None:
        rospack = rospkg.RosPack()
        try:
            pkg_path = rospack.get_path(pkg_name)
            rospy.logdebug("Package found!")
        except rospkg.common.ResourceNotFound:
            rospy.logerr("Package NOT FOUND! Recheck the Package name and Source the proper ROS Workspace!")
            return None

        file_path = os.path.join(pkg_path, "launch", launch_file_name)
        if os.path.exists(file_path) is False:
            print(f"Launch file {launch_file_name} in {file_path} does not exist!")
            return None

        return f"roslaunch {pkg_name} {launch_file_name}"

    # or roslaunch from a path
    elif launch_file_abs_path is not None:
        if os.path.exists(launch_file_abs_path) is False:
            print(f"Launch file {launch_file_abs_path} does not exist!")
            return None

        return f"roslaunch {launch_file_abs_path}"

    else:
        return None


"""
    14. ros_node_launcher: To launch a ROS node from a package
"""


def ros_node_launcher(pkg_name, node_name, launch_master=False, launch_new_term=True, name=None, ns="/", output="log",
                      ros_port=None, gazebo_port=None, args=None) -> Tuple[str, str, bool]:
    """
    Function to launch a ROS node from a package. If "launch_master" is "True", it will also launch a ROSCORE with the
    given "ros_port" or with a random ROS master port if "ros_port" is not specified.

    Args:
        pkg_name (str): Name of the package to launch the node from.
        node_name (str): Name of the node to launch.
        launch_master (bool): If ROSMASTER is not running launch it.
        launch_new_term (bool): Launch the process in a new terminal (Xterm).
        name (str): Name to give the node to be launched.
        ns (str): Namespace to give the node to be launched.
        output (str): log, screen, or None
        ros_port (str): The ROS_MASTER_URI port (optional).
        gazebo_port (str): The GAZEBO_MASTER_URI port (optional).
        args (list of str): Args to pass.

    Returns:
        tuple[str, str, bool]: ROS Master port, GAZEBO Master port and True if the node was launched successfully,
                               False otherwise.
    """

    # change the rosmaster
    if ros_port is not None and not launch_master:
        change_ros_gazebo_master(ros_port=ros_port, gazebo_port=gazebo_port)

    # verify if the given package is available
    if not check_package_exists(pkg_name):
        return "", "", False

    # only if launch_master is True
    rs_port = ""
    gz_port = ""

    # launching the roscore
    if launch_master:
        print("Launching ROS Master")
        if ros_port is not None:
            rs_port, gz_port = launch_roscore(port=int(ros_port))
        else:
            rs_port, gz_port = launch_roscore()

    # check if the rosmaster is running
    try:
        rospy.get_master().getPid()
    except ConnectionRefusedError:
        print("ROS Master not running!")
        return rs_port, gz_port, False
    else:
        print("ROS Master is running!")

    term_cmd = construct_rosrun_command(pkg_name, node_name, name=name, ns=ns, output=output)

    if args is not None:
        term_cmd += " " + " ".join(args)

    if launch_new_term:
        term_cmd = f"xterm -e '{term_cmd}'"

    subprocess.Popen(term_cmd, shell=True)
    time.sleep(5.0)

    return rs_port, gz_port, True


# helper fn for ros_node_launcher
def construct_rosrun_command(pkg_name, node_name, name=None, ns="/", output="log"):
    """
    Constructs a rosrun command using a package name and node name.

    Args:
        pkg_name (str): Name of the package to launch the node from.
        node_name (str): Name of the node to launch.
        name (str): Name to give the node to be launched.
        ns (str): Namespace to give the node to be launched.
        output (str): log, screen, or None

    Returns:
        str: The constructed rosrun command.
    """

    term_cmd = f"rosrun {pkg_name} {node_name}"

    if name is not None:
        term_cmd += f" __name:={name}"

    term_cmd += f" __ns:={ns}"
    term_cmd += f" __log:={output}"

    return term_cmd


# # helper fn for ros_node_launcher
def check_package_exists(pkg_name):
    """
    Checks if a given package exists.

    Args:
        pkg_name (str): Name of the package.

    Returns:
        bool: True if the package exists and False otherwise.
    """

    rospack = rospkg.RosPack()
    try:
        rospack.get_path(pkg_name)
        rospy.logdebug("Package FOUND...!")
        return True
    except rospkg.common.ResourceNotFound:
        rospy.logerr("Package NOT FOUND! Recheck the Package name and Source the proper ROS Workspace!")
        return False


"""
    15. ros_load_yaml: Fetch a YAML file from a package or a path and load it into the ROS Parameter Server.
"""


def ros_load_yaml(pkg_name=None, file_name=None, file_abs_path=None, ns='/', ros_port=None, gazebo_port=None) -> bool:
    """
    Fetch a YAML file from a package or an abs path and load it into the ROS Parameter Server.

    Args:
        pkg_name (str): name of package.
        file_name (str): name of file.
        file_abs_path (str): Absolute path of the YAML file.
        ns (str): namespace to load parameters into.
        ros_port (str): The ROS_MASTER_URI port (optional).
        gazebo_port (str): The GAZEBO_MASTER_URI port (optional).

    Returns:
        bool: True if file was loaded, false otherwise.
    """

    # Change the rosmaster if ros_port is not None
    if ros_port is not None:
        change_ros_gazebo_master(ros_port=ros_port, gazebo_port=gazebo_port)

    # If pkg_name and file_name are not None, try to locate the package and check if the YAML file exists within it
    if pkg_name and file_name is not None:
        rospack = rospkg.RosPack()
        try:
            pkg_path = rospack.get_path(pkg_name)
            rospy.logdebug(f"Package {pkg_name} located.")
        except rospkg.common.ResourceNotFound:
            rospy.logerr(f"Package {pkg_name} not found.")
            return False

        file_abs_path = pkg_path + "/config/" + file_name
        if os.path.exists(pkg_path + "/config/" + file_name) is False:
            print(f"Config file {file_name} in {file_abs_path} does not exist")
            return False

    # If pkg_name and file_name are both None but file_abs_path is not None,
    # check if the YAML file exists at the given absolute path
    elif file_abs_path is not None:
        if os.path.exists(file_abs_path) is False:
            print(f"Config file {file_abs_path} does not exist!")
            return False

    # If none of these conditions are met, return False
    else:
        print("Load Failed! Requires either the absolute path or the pkg_name and the file_name as input!")
        return False

    # Load the parameters from the YAML file and upload them to the ROS Parameter Server under the given namespace
    param_list = rosparam.load_file(file_abs_path)
    for params, namespace in param_list:
        rosparam.upload_params(ns, params)

    return True


"""
    16. load_urdf: Load a URDF into the parameter server or a string containing the processed URDF data
"""


def load_urdf(model_path=None, pkg_name=None, file_name=None, folder="/urdf", ns=None, args_xacro=None,
              param_name=None, ros_port=None, gazebo_port=None) -> Tuple[bool, Union[str, None]]:
    """
    Function to load a URDF from a ROS package to the parameter server or a string containing the processed URDF data.

    Args:
        model_path (str): The path to the URDF file. Defaults to None.
        pkg_name (str): The name of the ROS package containing the URDF file. Defaults to None.
        file_name (str): The name of the URDF file. Defaults to None.
        folder (str): The folder within the ROS package containing the URDF file. Defaults to "/urdf".
        ns (str): The namespace to use when adding the URDF to the parameter server. Defaults to None.
        args_xacro (list): Additional arguments to pass to xacro when processing the URDF file. Defaults to None.
        param_name (str): The name of the parameter to use when adding the URDF to the parameter server.
        ros_port (str): The ROS_MASTER_URI port (optional). Defaults to None.
        gazebo_port (str): The GAZEBO_MASTER_URI port (optional). Defaults to None.

    Returns:
        Tuple[bool, Union[str, None]]: A tuple containing a boolean value indicating whether the loading was
                                        successful and a string containing the processed URDF data. If an error
                                        occurred while loading or processing the URDF file, the first element of
                                        the tuple will be False and the second element will be None.
    """

    # Change the rosmaster
    if ros_port is not None:
        change_ros_gazebo_master(ros_port=ros_port, gazebo_port=gazebo_port)

    # Determine the path of the URDF file
    if pkg_name and file_name is not None:
        rospack = rospkg.RosPack()
        try:
            pkg_path = rospack.get_path(pkg_name)
            rospy.logdebug(f"Package '{pkg_name}' found at path '{pkg_path}'")
        except rospkg.common.ResourceNotFound:
            rospy.logerr(f"Package '{pkg_name}' not found")
            return False, None

        urdf_path = pkg_path + folder + "/" + file_name

    elif model_path is not None:
        urdf_path = model_path
    else:
        rospy.logerr("Invalid input: model_path and pkg_name/file_name are both None")
        return False, None

    # Check if the URDF file exists
    if os.path.exists(urdf_path) is False:
        rospy.logerr(f"Model path '{urdf_path}' does not exist")
        return False, None

    # Process the URDF file using xacro
    list_args = [urdf_path]
    if args_xacro is not None:
        list_args = list_args + args_xacro

    opts, input_file_name = xacro.process_args(list_args)
    model = xacro.process_file(input_file_name, **vars(opts))
    encoding = {}
    model_string = model.toprettyxml(indent=' ', **encoding)

    # Add the processed URDF data to the parameter server
    if param_name is not None:

        if ns is not None and ns != "/":
            ns = ns.rstrip('/')
            final_param_name = ns + "/" + param_name
            rospy.logdebug(f"URDF data added to parameter server with name '{final_param_name}' in namespace '{ns}'")
        else:
            final_param_name = param_name
            rospy.logdebug(f"URDF data added to parameter server with name '{final_param_name}'")

        rospy.set_param(final_param_name, model_string)

    return True, model_string


def is_roscore_running() -> bool:
    """
    Function to check if a roscore is currently running.

    Returns:
        bool: True if a roscore is running, False otherwise.
    """

    try:
        rospy.get_master().getPid()
    except ConnectionRefusedError:
        rospy.logerr("ROS Master not running!")
        return False
    else:
        rospy.loginfo("ROS Master is running!")
        return True


def change_ros_master(ros_port: str) -> bool:
    """
    Function to set the current ROS Master Environment Variable.

    Args:
        ros_port (str): The ROS_MASTER_URI port.

    Returns:
        bool: True if ROS Master Environment Variables are set to new values.
    """

    # Temporarily assigning values for the ROS_MASTER_URI environment variable
    os.environ["ROS_MASTER_URI"] = f"http://localhost:{ros_port}"

    rospy.logdebug(f"Changed ROS_MASTER_URI to port: {ros_port}")

    return True


def change_gazebo_master(gazebo_port: str) -> bool:
    """
    Function to set the current Gazebo Master Environment Variable.

    Args:
        gazebo_port (str): The GAZEBO_MASTER_URI port.

    Returns:
        bool: True if Gazebo Master Environment Variables are set to new values.
    """

    # Temporarily assigning values for the GAZEBO_MASTER_URI environment variable
    os.environ["GAZEBO_MASTER_URI"] = f"http://localhost:{gazebo_port}"

    rospy.logdebug(f"Changed GAZEBO_MASTER_URI to port: {gazebo_port}")

    return True


def init_robot_state_publisher(ns: str = "/", max_pub_freq: float = None, launch_new_term: bool = False) -> bool:
    """
    Function to initialize the robot state publisher.

    Args:
        ns (str): Namespace to give the node to be launched.
        max_pub_freq (float): Maximum frequency at which to publish the robot state.
        launch_new_term (bool): Launch the process in a new terminal (Xterm).

    Returns:
        bool: True if the node was launched successfully, False otherwise.
    """
    if max_pub_freq is not None:
        # Remove any trailing slashes from ns

        if ns != "/":
            ns = ns.rstrip('/')
            rospy.set_param(ns + "/rob_st_pub/publish_frequency", max_pub_freq)
        else:
            rospy.set_param("/rob_st_pub/publish_frequency", max_pub_freq)

    _, _, launch_done = ros_node_launcher(pkg_name="robot_state_publisher",
                                          node_name="robot_state_publisher",
                                          launch_new_term=launch_new_term, ns=ns)

    if launch_done:
        rospy.logdebug("Successfully initialised the Robot State Publisher!")
    else:
        rospy.logerr("Robot State Publisher initialisation Failed!")

    return launch_done


def remove_all_from_rosmaster_list() -> bool:
    """
    Function to remove all ports from the Multiros rosmaster port list.

    Returns:
        bool: True if all ports were removed successfully, False otherwise.
    """

    # Define the path to the file where the all the active rosmaster ports will be stored
    env_file_path = '/tmp/ros_master_ports_multiros.txt'

    with open(env_file_path, 'w') as f:
        f.write('')

    rospy.loginfo("Removed all ports from the Multiros rosmaster port list!")

    return True
