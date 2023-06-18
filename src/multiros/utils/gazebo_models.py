#! /usr/bin/env python

"""
This script is to specify all the common functions related to the handling of models in Gazebo.
It has the following functions,
   01. gazebo_spawn_urdf: Spwan a URDF model in Gazebo
   02. spawn_model_in_gazebo: Spawn a model from an SDF file in Gazebo
   03. spawn_sdf_model_gazebo: Spawn an SDF model in Gazebo. (Recommended)
   04. spawn_robot_in_gazebo: Spawn a robot in Gazebo.
   05. gazebo_get_world_properties: Get properties of the Gazebo world. (spawned model names)
   06. gazebo_delete_model: Delete a model from Gazebo
   07. remove_model_gazebo: Delete a model from Gazebo. (Recommended)
   08. gazebo_get_model_state: Get the state of a model in Gazebo (Header, Pose, Twist)
   09. gazebo_set_model_state: Set the state of a model in Gazebo.
"""

import rospy
import rospkg
import os
import time
from rospy.service import ServiceException

from gazebo_msgs.srv import DeleteModel, SpawnModel, GetWorldProperties
from gazebo_msgs.srv import SpawnModelRequest
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion, Twist

from multiros.utils import ros_common, ros_controllers, gazebo_core
from typing import Tuple, List


def gazebo_spawn_urdf(model_string=None, param_name=None, model_name="robot_0", robot_namespace="/",
                      reference_frame="world", pos_x=0.0, pos_y=0.0, pos_z=0.0,
                      ori_x=0.0, ori_y=0.0, ori_z=0.0, ori_w=1.0,
                      ros_port=None, gazebo_port=None) -> Tuple[bool, str]:
    """
    Function to spawn a URDF model in Gazebo. Spwan using either the model_string or the ros param_name.

    Args:
        model_string (str): A string containing the URDF data for the model to spawn. Defaults to None.
        param_name (str): The name of the parameter containing the URDF data for the model to spawn. Defaults to None.
        model_name (str): The name to assign to the spawned model. Defaults to "robot_0".
        robot_namespace (str): The namespace to use when spawning the model. Defaults to "/".
        reference_frame (str): The reference frame in which to spawn the model. Defaults to "world".
        pos_x (float): The x-coordinate of the position at which to spawn the model. Defaults to 0.0.
        pos_y (float): The y-coordinate of the position at which to spawn the model. Defaults to 0.0.
        pos_z (float): The z-coordinate of the position at which to spawn the model. Defaults to 0.0.
        ori_x (float): The x-component of the quaternion representing the orientation at which to spawn the model.
                       Defaults to 0.0.
        ori_y (float): The y-component of the quaternion representing the orientation at which to spawn the model.
                       Defaults to 0.0.
        ori_z (float): The z-component of the quaternion representing the orientation at which to spawn the model.
                       Defaults to 0.0.
        ori_w (float): The w-component of the quaternion representing the orientation at which to spawn the model.
                       Defaults to 1.0.
        ros_port (str): The ROS_MASTER_URI port (optional). Defaults to None.
        gazebo_port (str): The GAZEBO_MASTER_URI port (optional). Defaults to None.

    Returns:
        Tuple[bool, str]: A tuple containing a boolean value indicating whether spawning was successful and
                          a string containing a status message.
    """

    # Change the rosmaster
    if ros_port is not None:
        ros_common.change_ros_gazebo_master(ros_port=ros_port, gazebo_port=gazebo_port)

    # Wait for the Gazebo spawn service
    rospy.wait_for_service("/gazebo/spawn_urdf_model")
    client_srv = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)

    # Get the URDF data from a parameter if it was not provided directly
    if model_string is None and param_name is not None:

        # Remove any trailing slashes from robot_namespace
        robot_namespace = robot_namespace.rstrip('/')
        final_param_name = robot_namespace + "/" + param_name

        if rospy.has_param(final_param_name) is False:
            rospy.logerr(f"Parameter '{final_param_name}' does not exist")
            return False, f"Error: Parameter '{final_param_name}' does not exist"

        model_string = rospy.get_param(final_param_name)

    # Create a request object for spawning the model
    srv_model = SpawnModelRequest(model_name=model_name,
                                  model_xml=model_string,
                                  robot_namespace=robot_namespace,
                                  initial_pose=Pose(position=Point(x=pos_x, y=pos_y, z=pos_z),
                                                    orientation=Quaternion(x=ori_x, y=ori_y, z=ori_z, w=ori_w)),
                                  reference_frame=reference_frame)

    # Call the Gazebo spawn service
    try:
        result = client_srv(srv_model)
        return result.success, result.status_message
    except ServiceException:
        rospy.logerr("Package used in model was not found; source the workspace in all terminals")
        return False, "Error: Package used in model was not found; source the workspace in all terminals"


def spawn_model_in_gazebo(model_path=None, pkg_name=None, file_name=None, model_folder="/model", model_name="model_0",
                          robot_namespace="/", reference_frame="world", pos_x=0.0, pos_y=0.0, pos_z=0.0,
                          ori_x=0.0, ori_y=0.0, ori_z=0.0, ori_w=1.0, ros_port=None, gazebo_port=None) -> bool:
    """
    Function to spawn a model from an SDF file. The user need to give either absolute model_path or pkg_name/file_name.

    Args:
        model_path (str): The path to the SDF file. Defaults to None.
        pkg_name (str): The name of the ROS package containing the SDF file. Defaults to None.
        file_name (str): The name of the SDF file. Defaults to None.
        model_folder (str): The folder within the ROS package containing the SDF file. Defaults to "/model".
        model_name (str): The name to assign to the spawned model. Defaults to "model_0".
        robot_namespace (str): The namespace to use when spawning the model. Defaults to "/".
        reference_frame (str): The reference frame in which to spawn the model. Defaults to "world".
        pos_x (float): The x-coordinate of the position at which to spawn the model. Defaults to 0.0.
        pos_y (float): The y-coordinate of the position at which to spawn the model. Defaults to 0.0.
        pos_z (float): The z-coordinate of the position at which to spawn the model. Defaults to 0.0.
        ori_x (float): The x-component of the quaternion representing the orientation at which to spawn the model.
                       Defaults to 0.0.
        ori_y (float): The y-component of the quaternion representing the orientation at which to spawn the model.
                       Defaults to 0.0.
        ori_z (float): The z-component of the quaternion representing the orientation at which to spawn the model.
                       Defaults to 0.0.
        ori_w (float): The w-component of the quaternion representing the orientation at which to spawn the model.
                       Defaults to 1.0.
        ros_port (str): The ROS_MASTER_URI port (optional). Defaults to None.
        gazebo_port (str): The GAZEBO_MASTER_URI port (optional). Defaults to None.

    Returns:
        bool: True if spawning was successful, False otherwise.
    """

    # Change the rosmaster
    if ros_port is not None:
        ros_common.change_ros_gazebo_master(ros_port=ros_port, gazebo_port=gazebo_port)

    # Wait for the Gazebo spawn service
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    client_srv = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

    # Determine the path of the SDF file
    if pkg_name and file_name is not None:
        rospack = rospkg.RosPack()
        try:
            pkg_path = rospack.get_path(pkg_name)
            rospy.logdebug(f"Package '{pkg_name}' found at path '{pkg_path}'")
        except rospkg.common.ResourceNotFound:
            rospy.logerr(f"Package '{pkg_name}' not found")
            return False

        sdf_path = pkg_path + model_folder + "/" + file_name

    elif model_path is not None:
        sdf_path = model_path
    else:
        rospy.logerr("Invalid input: model_path and pkg_name/file_name are both None")
        return False

    # Check if the SDF file exists
    if os.path.exists(sdf_path) is False:
        rospy.logerr(f"Model path '{sdf_path}' does not exist")
        return False

    # Read and process the SDF file
    with open(sdf_path, 'r') as f:
        sdf_string = f.read()

    # Create a request object for spawning the model
    srv_model = SpawnModelRequest(model_name=model_name,
                                  model_xml=sdf_string,
                                  robot_namespace=robot_namespace,
                                  initial_pose=Pose(position=Point(x=pos_x, y=pos_y, z=pos_z),
                                                    orientation=Quaternion(x=ori_x, y=ori_y, z=ori_z, w=ori_w)),
                                  reference_frame=reference_frame)

    # Call the Gazebo spawn service
    try:
        result = client_srv(srv_model)
        return result.success
    except ServiceException:
        rospy.logerr("Package used in model was not found; source the workspace in all terminals")
        return False


def spawn_sdf_model_gazebo(model_path: str = None, pkg_name: str = None, file_name: str = None,
                           model_folder: str = "/model", model_name: str = "model_0",
                           namespace: str = "/", reference_frame: str = "world",
                           pos_x: float = 0.0, pos_y: float = 0.0, pos_z: float = 0.0,
                           ori_x: float = 0.0, ori_y: float = 0.0, ori_z: float = 0.0, ori_w: float = 1.0,
                           ros_port=None, gazebo_port=None, max_tries: int = 5) -> bool:
    """
    Function to spawn an SDF model in Gazebo. This function make sure that model was spawned.

    Args:
        model_path (str): The absolute path to the SDF file. Defaults to None.
        pkg_name (str): The name of the ROS package containing the SDF file. Defaults to None.
        file_name (str): The name of the SDF file. Defaults to None.
        model_folder (str): The folder within the ROS package containing the SDF file. Defaults to "/model".
        model_name (str): The name to give the model when spawning it in Gazebo. Defaults to "model_0".
        namespace (str): The namespace to use when adding the SDF to the parameter server and when launching nodes. Defaults to "/".
        reference_frame (str): The reference frame to use when spawning the model in Gazebo. Defaults to "world".
        pos_x (float): The x position of the model in Gazebo. Defaults to 0.0.
        pos_y (float): The y position of the model in Gazebo. Defaults to 0.0.
        pos_z (float): The z position of the model in Gazebo. Defaults to 0.0.
        ori_x (float): The x orientation of the model in Gazebo. Defaults to 0.0.
        ori_y (float): The y orientation of the model in Gazebo. Defaults to 0.0.
        ori_z (float): The z orientation of the model in Gazebo. Defaults to 0.0.
        ori_w (float): The w orientation of the model in Gazebo. Defaults to 1.0.
        ros_port (str): The ROS_MASTER_URI port (optional). Defaults to None.
        gazebo_port (str): The GAZEBO_MASTER_URI port (optional). Defaults to None.
        max_tries (int): The maximum number of times to attempt to spawn the model. Defaults to 5.

    Returns:
        bool: True if all operations were successful, False otherwise.
    """
    # Change the rosmaster
    if ros_port is not None:
        ros_common.change_ros_gazebo_master(ros_port=ros_port, gazebo_port=gazebo_port)

    spawn_done = False  # flag to check if the object has been spawned successfully

    # there is an issue of this becoming an infinite loop. So we use tries
    while not spawn_done and max_tries != 0:

        # let's spawn the object
        spawn_model_in_gazebo(model_path=model_path, pkg_name=pkg_name, file_name=file_name, model_folder=model_folder,
                              model_name=model_name, robot_namespace=namespace, reference_frame=reference_frame,
                              pos_x=pos_x, pos_y=pos_y, pos_z=pos_z, ori_x=ori_x, ori_y=ori_y, ori_z=ori_z, ori_w=ori_w)

        # we need to unpause before ropy.sleep
        gazebo_core.unpause_gazebo()
        rospy.sleep(5)  # wait for 5 seconds

        _, _, model_names = gazebo_get_world_properties()  # get the names of all the models in the world

        # check if the object is in the world
        if model_name in model_names:
            # we have the object in gazebo
            rospy.loginfo(f"Successfully spawned {model_name}!")
            spawn_done = True
        else:
            rospy.loginfo(f"{model_name} not found in Gazebo. Respawning!")
            # to avoid infinite loop
            max_tries -= 1

    # let's pause the gazebo after the loop
    gazebo_core.pause_gazebo()

    return spawn_done


def spawn_robot_in_gazebo(pkg_name: str, model_urdf_file: str, model_urdf_folder: str = "/urdf", ns: str = "/",
                          args_xacro: list = None, pub_freq: float = None, rob_st_term: bool = False,
                          gazebo_name: str = "robot", gz_ref_frame: str = "world",
                          pos_x: float = 0.0, pos_y: float = 0.0, pos_z: float = 0.0,
                          ori_w: float = 0.0, ori_x: float = 0.0, ori_y: float = 0.0, ori_z: float = 0.0,
                          controllers_file: str = None, controllers_list: list = None,
                          ros_port: str = None, gazebo_port: str = None) -> bool:
    """
    Function to spawn a robot in Gazebo.

    Args:
        pkg_name (str): The name of the ROS package containing the URDF/ Controllers files.
        model_urdf_file (str): The name of the URDF file.
        model_urdf_folder (str): The folder within the ROS package containing the URDF file. Defaults to "/urdf".
        ns (str): The namespace to use when adding the URDF to the parameter server and when launching nodes. Defaults to "/".
        args_xacro (list): Additional arguments to pass to xacro when processing the URDF file. Defaults to None.
        pub_freq (float): The maximum frequency at which the robot_state_publisher should publish the robot's state. Defaults to None.
        rob_st_term (bool): Whether to launch the robot_state_publisher node in a new terminal window. Defaults to False.
        gazebo_name (str): The name to give the robot model when spawning it in Gazebo. Defaults to "robot".
        gz_ref_frame (str): The reference frame to use when spawning the robot model in Gazebo. Defaults to "world".
        pos_x (float): The x position of the robot model in Gazebo. Defaults to 0.0.
        pos_y (float): The y position of the robot model in Gazebo. Defaults to 0.0.
        pos_z (float): The z position of the robot model in Gazebo. Defaults to 0.0.
        ori_w (float): The w orientation of the robot model in Gazebo. Defaults to 0.0.
        ori_x (float): The x orientation of the robot model in Gazebo. Defaults to 0.0.
        ori_y (float): The y orientation of the robot model in Gazebo. Defaults to 0.0.
        ori_z (float): The z orientation of the robot model in Gazebo. Defaults to 0.0.
        controllers_file (str): The name of a YAML file containing controller configurations. Defaults to None.
        controllers_list (list): A list of controller names to spawn. Defaults to None.
        ros_port (str): The ROS_MASTER_URI port (optional). Defaults to None.
        gazebo_port (str): The GAZEBO_MASTER_URI port (optional). Defaults to None.

    Returns:
        bool: True if all operations were successful, False otherwise.
    """
    if controllers_list is None:
        controllers_list = []

    # Change the rosmaster
    if ros_port is not None:
        ros_common.change_ros_gazebo_master(ros_port=ros_port, gazebo_port=gazebo_port)

    # Load the model URDF in the parameter server
    try:
        load_done, model_string = ros_common.load_urdf(pkg_name=pkg_name,
                                                       file_name=model_urdf_file,
                                                       folder=model_urdf_folder, ns=ns, args_xacro=args_xacro,
                                                       param_name="robot_description")

        rospy.loginfo("URDF file loaded successfully")
    except ServiceException:
        rospy.logerr("Error while loading URDF file")
        return False

    time.sleep(0.1)

    # Initialize the Robot State Publisher node set the publishing freq
    # check for publish frequency
    if pub_freq is not None:
        # Remove any trailing slashes from ns

        if ns != "/":
            ns = ns.rstrip('/')
            rospy.set_param(ns + "/robot_state_publisher/publish_frequency", pub_freq)
        else:
            rospy.set_param("/robot_state_publisher/publish_frequency", pub_freq)

    # we don't need the first two since we are not launching a new roscore
    _, _, launch_done = ros_common.ros_node_launcher(pkg_name="robot_state_publisher",
                                                     node_name="robot_state_publisher",
                                                     launch_new_term=rob_st_term, ns=ns)

    if launch_done:
        rospy.loginfo("Robot state publisher initialized")
    else:
        rospy.logerr("Error while initializing robot state publisher")
        return False

    time.sleep(0.1)

    # Spawn the model in gazebo
    result_spawn, message = gazebo_spawn_urdf(model_string=model_string, model_name=gazebo_name,
                                              robot_namespace=ns, reference_frame=gz_ref_frame,
                                              pos_x=pos_x, pos_y=pos_y, pos_z=pos_z,
                                              ori_x=ori_x, ori_y=ori_y, ori_z=ori_z, ori_w=ori_w)
    if result_spawn:
        rospy.loginfo("Model spawned successfully")
    else:
        rospy.logerr("Error while spawning model")
        rospy.logerr(message)
        return False

    time.sleep(0.1)

    # Launch controllers
    if controllers_file is not None:
        # Load the robot controllers from YAML files in the parameter server
        if ros_common.ros_load_yaml(pkg_name=pkg_name, file_name=controllers_file, ns=ns):
            rospy.loginfo("Robot controllers loaded successfully")
        else:
            rospy.logerr("Error while loading robot controllers")
            return False

        time.sleep(0.1)

        # Spawn the controllers
        if ros_controllers.spawn_controllers(controllers_list, ns=ns):
            rospy.loginfo("Controllers spawned successfully")
        else:
            rospy.logerr("Error while spawning controllers")
            return False

    return True


def gazebo_get_world_properties(ros_port=None, gazebo_port=None) -> Tuple[bool, float, List[str]]:
    """
    Function to get properties of the Gazebo world.

    Args:
        ros_port (str): The ROS_MASTER_URI port (optional). Defaults to None.
        gazebo_port (str): The GAZEBO_MASTER_URI port (optional). Defaults to None.

    Returns:
        Tuple[bool, float, List[str]]: A tuple containing a boolean value indicating whether the operation was
                                       successful, the current simulation time, and a list of model names in
                                       the world.

    http://docs.ros.org/en/diamondback/api/gazebo/html/srv/GetWorldProperties.html
    """
    # Change the rosmaster
    if ros_port is not None:
        ros_common.change_ros_gazebo_master(ros_port=ros_port, gazebo_port=gazebo_port)

    # Wait for the /gazebo/get_world_properties service
    rospy.wait_for_service("/gazebo/get_world_properties")
    client_get_world_properties = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)

    # Call the /gazebo/get_world_properties service
    world_specs = client_get_world_properties()

    # Return the success status, simulation time, and list of model names
    return world_specs.success, world_specs.sim_time, world_specs.model_names


def gazebo_delete_model(model_name: str, ros_port=None, gazebo_port=None) -> Tuple[bool, str]:
    """
    Function to delete a model from Gazebo.

    Args:
        model_name (str): The name of the model to delete.
        ros_port (str): The ROS_MASTER_URI port (optional). Defaults to None.
        gazebo_port (str): The GAZEBO_MASTER_URI port (optional). Defaults to None.

    Returns:
        Tuple[bool, str]: A tuple containing a boolean value indicating whether the operation was successful and
                          a string containing an error message if the operation was not successful.

    http://docs.ros.org/en/diamondback/api/gazebo/html/srv/DeleteModel.html
    """
    # Change the rosmaster
    if ros_port is not None:
        ros_common.change_ros_gazebo_master(ros_port=ros_port, gazebo_port=gazebo_port)

    # Wait for the /gazebo/delete_model service
    rospy.wait_for_service("/gazebo/delete_model")
    client_delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)

    # Call the /gazebo/delete_model service
    result = client_delete_model(model_name=model_name)

    # Return the success status
    return result.success


def remove_model_gazebo(model_name: str, max_tries: int = 5, ros_port=None, gazebo_port=None) -> bool:
    """
    Function to make sure if a model is deleted from Gazebo.

    Args:
        model_name (str): The name of the model to delete.
        max_tries (int): The maximum number of times to attempt to delete the model. Defaults to 5.
        ros_port (str): The ROS_MASTER_URI port (optional). Defaults to None.
        gazebo_port (str): The GAZEBO_MASTER_URI port (optional). Defaults to None.

    Returns:
        bool: True if the operation was successful, False otherwise.
    """
    # Change the rosmaster
    if ros_port is not None:
        ros_common.change_ros_gazebo_master(ros_port=ros_port, gazebo_port=gazebo_port)

    remove_done = False  # flag to check if the model has been removed successfully

    # there is an issue of this becoming an infinite loop. So we use tries
    while not remove_done and max_tries != 0:
        # we need to unpause before rospy.sleep
        gazebo_core.unpause_gazebo()
        rospy.sleep(5)  # wait for 5 seconds

        # check if the object already in the sim
        _, _, model_names = gazebo_get_world_properties()  # get the names of all the models in the world

        if model_name in model_names:
            # remove if the object in the sim
            gazebo_delete_model(model_name)
        else:
            # object is not in the sim. The task is done
            rospy.loginfo(f"Successfully removed {model_name}!")
            remove_done = True

        max_tries -= 1

    # let's pause the gazebo after the loop
    gazebo_core.pause_gazebo()

    return remove_done


def gazebo_get_model_state(model_name: str, relative_entity_name: str = 'world',
                           ros_port=None, gazebo_port=None) -> Tuple[Header, Pose, Twist, bool]:
    """
    Function to get the state of a model in Gazebo.

    Args:
        model_name (str): The name of the model to get the state of.
        relative_entity_name (str): The name of the entity to which the returned pose is relative. Defaults to 'world'
        ros_port (str): The ROS_MASTER_URI port (optional). Defaults to None.
        gazebo_port (str): The GAZEBO_MASTER_URI port (optional). Defaults to None.

    Returns:
        Tuple[Header, Pose, Twist, bool]: A tuple containing a Header message with the timestamp of the returned state,
                                          a Pose message with the position and orientation of the model,
                                          a Twist message with the linear and angular velocity of the model,
                                          and a boolean value indicating whether the operation was successful.

    http://docs.ros.org/en/diamondback/api/gazebo/html/srv/GetModelState.html
    """
    # Change the rosmaster
    if ros_port is not None:
        ros_common.change_ros_gazebo_master(ros_port=ros_port, gazebo_port=gazebo_port)

    # Wait for the /gazebo/get_model_state service
    rospy.wait_for_service("/gazebo/get_model_state")
    client_get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

    # Call the /gazebo/get_model_state service
    result = client_get_model_state(model_name=model_name, relative_entity_name=relative_entity_name)

    # Return the header, pose, twist, and success status
    return result.header, result.pose, result.twist, result.success


def gazebo_set_model_state(model_name: str, reference_frame: str = "world",
                           pos_x: float = 0.0, pos_y: float = 0.0, pos_z: float = 0.0,
                           ori_x: float = 0.0, ori_y: float = 0.0, ori_z: float = 0.0, ori_w: float = 0.0,
                           lin_vel_x: float = 0.0, lin_vel_y: float = 0.0, lin_vel_z: float = 0.0,
                           ang_vel_x: float = 0.0, ang_vel_y: float = 0.0, ang_vel_z: float = 0.0,
                           sleep_time: float = 0.05,
                           ros_port=None, gazebo_port=None) -> bool:
    """
    Function to set the state of a model in Gazebo.

    Args:
        model_name (str): The name of the model to set the state of.
        reference_frame (str): The name of the reference frame to use when setting the pose of the model. Defaults to "world".
        pos_x (float): The x position of the model in Gazebo. Defaults to 0.0.
        pos_y (float): The y position of the model in Gazebo. Defaults to 0.0.
        pos_z (float): The z position of the model in Gazebo. Defaults to 0.0.
        ori_x (float): The x orientation of the model in Gazebo. Defaults to 0.0.
        ori_y (float): The y orientation of the model in Gazebo. Defaults to 0.0.
        ori_z (float): The z orientation of the model in Gazebo. Defaults to 0.0.
        ori_w (float): The w orientation of the model in Gazebo. Defaults to 1.0.
        lin_vel_x (float): The x component of the linear velocity of the model in Gazebo. Defaults to 0.0.
        lin_vel_y (float): The y component of the linear velocity of the model in Gazebo. Defaults to 0.0.
        lin_vel_z (float): The z component of the linear velocity of the model in Gazebo. Defaults to 0.0.
        ang_vel_x (float): The x component of the angular velocity of the model in Gazebo. Defaults to 0.0.
        ang_vel_y (float): The y component of the angular velocity of the model in Gazebo. Defaults to 0.0.
        ang_vel_z (float): The z component of the angular velocity of the model in Gazebo. Defaults to 0.0.
        sleep_time (float): The amount of time to sleep after setting the state of the model (optional). Defaults to 0.05 seconds.
        ros_port (str): The ROS_MASTER_URI port (optional). Defaults to None.
        gazebo_port (str): The GAZEBO_MASTER_URI port (optional). Defaults to None.

    Returns:
        bool: True if the operation was successful, False otherwise.

    http://docs.ros.org/en/diamondback/api/gazebo/html/srv/SetModelState.html
    """
    # Change the rosmaster
    if ros_port is not None:
        ros_common.change_ros_gazebo_master(ros_port=ros_port, gazebo_port=gazebo_port)

    # Create a ModelState message
    model_state = ModelState()
    model_state.model_name = model_name
    model_state.reference_frame = reference_frame
    model_state.pose.position.x = pos_x
    model_state.pose.position.y = pos_y
    model_state.pose.position.z = pos_z
    model_state.pose.orientation.x = ori_x
    model_state.pose.orientation.y = ori_y
    model_state.pose.orientation.z = ori_z
    model_state.pose.orientation.w = ori_w
    model_state.twist.linear.x = lin_vel_x
    model_state.twist.linear.y = lin_vel_y
    model_state.twist.linear.z = lin_vel_z
    model_state.twist.angular.x = ang_vel_x
    model_state.twist.angular.y = ang_vel_y
    model_state.twist.angular.z = ang_vel_z

    # Wait for the /gazebo/set_model_state service
    rospy.wait_for_service("/gazebo/set_model_state")
    client_set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

    # Call the /gazebo/set_model_state service
    result = client_set_model_state(model_state=model_state)

    # Sleep for the specified amount of time
    rospy.sleep(sleep_time)

    # Return the success status
    return result.success
