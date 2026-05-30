#! /usr/bin/env python

"""
Common functions for handling models in MuJoCo.

In MuJoCo the world and the robot are defined by a single MJCF/XML scene that is loaded when
the server starts. There is no per-model spawn/delete service as in Gazebo; the scene can be
replaced at run time with the ``reload`` service, and individual free-jointed bodies that are
already present in the scene can be repositioned with the ``set_body_state`` service.

Functions provided:

- ``mujoco_reload`` — load or replace the whole scene model at run time.
- ``mujoco_get_body_state`` — get the state of a body (Header, Pose, Twist, success).
- ``mujoco_set_body_state`` — set the state of a body (pose / twist / qpos reset).
- ``filter_urdf_transmissions`` — strip ``<transmission>`` elements for joints not in a keep-list
  (needed because mujoco_ros_control aborts on transmissions whose joint is absent from the MJCF).
- ``spawn_robot_in_mujoco`` — bring up a robot's ROS interfaces (description, state
  publisher, controllers). The robot geometry itself is provided by the loaded MJCF scene.

Classes provided:

- ``MujocoSceneManager`` — maintain an authoritative scene graph (a base scene plus a set of
  added objects), regenerate the merged MJCF, and apply it with the ``reload`` service. This
  provides spawn/delete semantics analogous to the Gazebo model services. For repositioning a
  free-jointed object that is already present in the scene, ``mujoco_set_body_state`` is faster
  than a reload.
"""

import os
import tempfile
import xml.etree.ElementTree as ET

import rospy
import time
from typing import Dict, List, Optional, Sequence, Tuple

from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseStamped, Twist, TwistStamped, Point, Quaternion, Vector3
from multiros.utils import ros_common, ros_controllers

# These interfaces are only required for the MuJoCo backend. Import them lazily so that
# importing this module (and the wider package) does not fail on installations that only use
# the Gazebo backend.
try:
    from mujoco_ros_msgs.srv import Reload, SetBodyState, GetBodyState
    from mujoco_ros_msgs.msg import BodyState

    _MUJOCO_MSGS_AVAILABLE = True
except ImportError:
    _MUJOCO_MSGS_AVAILABLE = False

DEFAULT_SERVER_NAME = "mujoco_server"


def _require_mujoco_msgs() -> None:
    """Raise a clear error if the mujoco_ros_pkgs Python interfaces are unavailable."""
    if not _MUJOCO_MSGS_AVAILABLE:
        raise ImportError(
            "mujoco_ros_msgs could not be imported. Build mujoco_ros_pkgs in the workspace to "
            "use the MuJoCo backend."
        )


def mujoco_reload(model_path: Optional[str] = None, model_string: Optional[str] = None,
                  server_name: str = DEFAULT_SERVER_NAME, ros_port: Optional[str] = None) -> Tuple[bool, str]:
    """
    Function to load or replace the whole scene model in MuJoCo at run time.

    Either an absolute path to an MJCF/XML file or a raw MJCF string can be supplied. Loading a
    file path is recommended when the scene references external assets (meshes, textures).

    Args:
        model_path (str): Absolute path to an MJCF/XML file to load. Defaults to None.
        model_string (str): A raw MJCF string to load (used if model_path is None). Defaults to None.
        server_name (str): Graph name of the server node (optional).
        ros_port (str): The ROS_MASTER_URI port (optional).

    Returns:
        Tuple[bool, str]: A tuple containing a boolean indicating whether the reload succeeded
                          and a status message.
    """
    _require_mujoco_msgs()

    if ros_port is not None:
        ros_common.change_ros_master(ros_port=ros_port)

    if model_path is not None:
        model = model_path
    elif model_string is not None:
        model = model_string
    else:
        rospy.logerr("Invalid input: both model_path and model_string are None")
        return False, "Error: no model supplied"

    service_name = f"/{server_name}/reload"
    try:
        rospy.wait_for_service(service_name, timeout=30.0)
    except rospy.ROSException as e:
        rospy.logerr(f"Timeout (30s) waiting for service '{service_name}': {e}")
        return False, "Error: timeout waiting for reload service"

    try:
        reload_service = rospy.ServiceProxy(service_name, Reload)
        result = reload_service(model=model)
        return result.success, result.status_message

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False, f"Error: {e}"


def mujoco_get_body_state(body_name: str, server_name: str = DEFAULT_SERVER_NAME,
                          ros_port: Optional[str] = None) -> Tuple[Header, Pose, Twist, bool]:
    """
    Function to get the state of a body in MuJoCo.

    Args:
        body_name (str): The name of the body (or a geom belonging to the body).
        server_name (str): Graph name of the server node (optional).
        ros_port (str): The ROS_MASTER_URI port (optional).

    Returns:
        Tuple[Header, Pose, Twist, bool]: A tuple containing the Header of the returned state,
                                          the Pose of the body, the Twist of the body, and a
                                          boolean indicating whether the operation was successful.
    """
    _require_mujoco_msgs()

    if ros_port is not None:
        ros_common.change_ros_master(ros_port=ros_port)

    service_name = f"/{server_name}/get_body_state"
    try:
        rospy.wait_for_service(service_name, timeout=30.0)
    except rospy.ROSException as e:
        rospy.logerr(f"Timeout (30s) waiting for service '{service_name}': {e}")
        return Header(), Pose(), Twist(), False

    try:
        get_body_state = rospy.ServiceProxy(service_name, GetBodyState)
        result = get_body_state(name=body_name)
        return result.state.pose.header, result.state.pose.pose, result.state.twist.twist, result.success

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return Header(), Pose(), Twist(), False


def mujoco_set_body_state(body_name: str, reference_frame: str = "world",
                          pos_x: float = 0.0, pos_y: float = 0.0, pos_z: float = 0.0,
                          ori_x: float = 0.0, ori_y: float = 0.0, ori_z: float = 0.0, ori_w: float = 1.0,
                          lin_vel_x: float = 0.0, lin_vel_y: float = 0.0, lin_vel_z: float = 0.0,
                          ang_vel_x: float = 0.0, ang_vel_y: float = 0.0, ang_vel_z: float = 0.0,
                          set_pose: bool = True, set_twist: bool = False, reset_qpos: bool = False,
                          sleep_time: float = 0.05,
                          server_name: str = DEFAULT_SERVER_NAME,
                          ros_port: Optional[str] = None) -> bool:
    """
    Function to set the state of a body in MuJoCo.

    Only bodies with a single free joint can be repositioned. The pose is interpreted in the
    given reference frame.

    Args:
        body_name (str): The name of the body to set the state of.
        reference_frame (str): The reference frame of the pose. Defaults to "world".
        pos_x (float): The x position of the body. Defaults to 0.0.
        pos_y (float): The y position of the body. Defaults to 0.0.
        pos_z (float): The z position of the body. Defaults to 0.0.
        ori_x (float): The x orientation of the body. Defaults to 0.0.
        ori_y (float): The y orientation of the body. Defaults to 0.0.
        ori_z (float): The z orientation of the body. Defaults to 0.0.
        ori_w (float): The w orientation of the body. Defaults to 1.0.
        lin_vel_x (float): The x component of the linear velocity. Defaults to 0.0.
        lin_vel_y (float): The y component of the linear velocity. Defaults to 0.0.
        lin_vel_z (float): The z component of the linear velocity. Defaults to 0.0.
        ang_vel_x (float): The x component of the angular velocity. Defaults to 0.0.
        ang_vel_y (float): The y component of the angular velocity. Defaults to 0.0.
        ang_vel_z (float): The z component of the angular velocity. Defaults to 0.0.
        set_pose (bool): Whether to apply the pose. Defaults to True.
        set_twist (bool): Whether to apply the twist. Defaults to False.
        reset_qpos (bool): Whether to reset the body's joint positions to their defaults. Defaults to False.
        sleep_time (float): The amount of time to sleep after setting the state. Defaults to 0.05 seconds.
        server_name (str): Graph name of the server node (optional).
        ros_port (str): The ROS_MASTER_URI port (optional).

    Returns:
        bool: True if the operation was successful, False otherwise.
    """
    _require_mujoco_msgs()

    if ros_port is not None:
        ros_common.change_ros_master(ros_port=ros_port)

    # Build the body state message
    body_state = BodyState()
    body_state.name = body_name
    body_state.pose = PoseStamped(
        header=Header(frame_id=reference_frame),
        pose=Pose(position=Point(x=pos_x, y=pos_y, z=pos_z),
                  orientation=Quaternion(x=ori_x, y=ori_y, z=ori_z, w=ori_w)))
    body_state.twist = TwistStamped()
    body_state.twist.twist.linear = Vector3(x=lin_vel_x, y=lin_vel_y, z=lin_vel_z)
    body_state.twist.twist.angular = Vector3(x=ang_vel_x, y=ang_vel_y, z=ang_vel_z)

    service_name = f"/{server_name}/set_body_state"
    try:
        rospy.wait_for_service(service_name, timeout=30.0)
    except rospy.ROSException as e:
        rospy.logerr(f"Timeout (30s) waiting for service '{service_name}': {e}")
        return False

    try:
        set_body_state = rospy.ServiceProxy(service_name, SetBodyState)
        result = set_body_state(state=body_state, set_pose=set_pose, set_twist=set_twist,
                                set_mass=False, reset_qpos=reset_qpos)
        rospy.sleep(sleep_time)
        return bool(result.success)

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False


def filter_urdf_transmissions(urdf_string: str, keep_joints: List[str]) -> str:
    """
    Return a copy of a URDF string keeping only the ``<transmission>`` elements whose actuated
    joint is in ``keep_joints``; all other transmissions are removed.

    Why this exists: mujoco_ros_control's ``DefaultRobotHWSim`` walks every ``<transmission>`` in
    ``robot_description`` and aborts (``Could not initialize robot simulation interface``) if a
    transmission joint is not present in the loaded MuJoCo model. Manufacturer URDFs commonly carry
    more transmissions than the MJCF exposes (grippers, mimic fingers), so a MuJoCo env that drives
    only a subset of joints must strip the rest. All links and joints are left untouched (so forward
    kinematics and the robot_state_publisher still see the full tree); only ``<transmission>``
    elements are removed.

    Args:
        urdf_string (str): The processed URDF/XML as a string.
        keep_joints (list): Names of the joints whose transmissions should be kept.

    Returns:
        str: The filtered URDF as a string.
    """
    keep = set(keep_joints)
    root = ET.fromstring(urdf_string)
    removed = []
    for trans in list(root.findall("transmission")):
        joint = trans.find("joint")
        name = joint.get("name") if joint is not None else None
        if name not in keep:
            root.remove(trans)
            removed.append(name)
    if removed:
        rospy.loginfo(f"filter_urdf_transmissions: stripped transmissions for {removed}; "
                      f"kept {sorted(keep)}")
    return ET.tostring(root, encoding="unicode")


def spawn_robot_in_mujoco(pkg_name: str, model_urdf_file: str, model_urdf_folder: str = "/urdf", ns: str = "/",
                          args_xacro: Optional[List[str]] = None,
                          pub_freq: Optional[float] = None,
                          rob_st_term: bool = False,
                          controllers_file: Optional[str] = None,
                          controllers_list: Optional[List[str]] = None,
                          ros_port: Optional[str] = None,
                          controller_package_name: Optional[str] = None,
                          controlled_joints: Optional[List[str]] = None) -> bool:
    """
    Function to bring up a robot's ROS interfaces for MuJoCo.

    The robot geometry is provided by the MJCF scene loaded by the server, so this function does
    not spawn the robot into the simulation. It loads the robot description onto the parameter
    server (used by ros_control, the robot_state_publisher and the kinematics utilities),
    launches the robot_state_publisher, and loads and spawns the requested controllers.

    Args:
        pkg_name (str): The name of the ROS package containing the URDF / controller files.
        model_urdf_file (str): The name of the URDF file.
        model_urdf_folder (str): The folder within the ROS package containing the URDF file. Defaults to "/urdf".
        ns (str): The namespace to use when adding the URDF to the parameter server and when launching nodes. Defaults to "/".
        args_xacro (list): Additional arguments to pass to xacro when processing the URDF file. Defaults to None.
        pub_freq (float): The maximum frequency at which the robot_state_publisher should publish. Defaults to None.
        rob_st_term (bool): Whether to launch the robot_state_publisher node in a new terminal. Defaults to False.
        controllers_file (str): The name of a YAML file containing controller configurations. Defaults to None.
        controllers_list (list): A list of controller names to spawn. Defaults to None.
        ros_port (str): The ROS_MASTER_URI port (optional). Defaults to None.
        controller_package_name (str): The name of the package containing the controllers. Defaults to None.
        controlled_joints (list): If given, only the ``<transmission>`` elements for these joints are
            kept in the loaded ``robot_description``; transmissions for any other joints are stripped.
            Use this when the URDF declares more transmissions than the MuJoCo model exposes (e.g. a
            gripper/fingers absent from the MJCF), which would otherwise abort the
            ``mujoco_ros_control`` plugin. Defaults to None (load the URDF unchanged).

    Returns:
        bool: True if all operations were successful, False otherwise.
    """
    if controllers_list is None:
        controllers_list = []

    # Change the rosmaster
    if ros_port is not None:
        ros_common.change_ros_master(ros_port=ros_port)

    # Load the model URDF onto the parameter server. When controlled_joints is given, fetch the
    # processed URDF string (param_name=None -> do not set the param yet), strip the transmissions
    # for non-controlled joints, then set robot_description ourselves under the namespace.
    try:
        if controlled_joints:
            _, urdf_string = ros_common.load_urdf(pkg_name=pkg_name,
                                                  file_name=model_urdf_file,
                                                  folder=model_urdf_folder, args_xacro=args_xacro,
                                                  param_name=None)
            urdf_string = filter_urdf_transmissions(urdf_string, controlled_joints)
            if ns is not None and ns != "/":
                param_name = ns.rstrip('/') + "/robot_description"
            else:
                param_name = "robot_description"
            rospy.set_param(param_name, urdf_string)
        else:
            load_done, _ = ros_common.load_urdf(pkg_name=pkg_name,
                                                 file_name=model_urdf_file,
                                                 folder=model_urdf_folder, ns=ns, args_xacro=args_xacro,
                                                 param_name="robot_description")

        rospy.loginfo("URDF file loaded successfully")
    except Exception:
        rospy.logerr("Error while loading URDF file")
        return False

    time.sleep(0.1)

    # Initialize the Robot State Publisher node and set the publishing frequency
    if pub_freq is not None:
        _, _, launch_done = ros_common.ros_node_launcher(pkg_name="robot_state_publisher",
                                                         node_name="robot_state_publisher",
                                                         launch_new_term=rob_st_term,
                                                         ns=ns,
                                                         args=[f"publish_frequency:={pub_freq}"]
                                                         )
    else:
        _, _, launch_done = ros_common.ros_node_launcher(pkg_name="robot_state_publisher",
                                                         node_name="robot_state_publisher",
                                                         launch_new_term=rob_st_term,
                                                         ns=ns)

    if launch_done:
        rospy.loginfo("Robot state publisher initialized")
    else:
        rospy.logerr("Error while initializing robot state publisher")
        return False

    time.sleep(0.1)

    # Launch controllers
    if controllers_file is not None:
        # Load the robot controllers from YAML files onto the parameter server
        if controller_package_name is None:
            if ros_common.ros_load_yaml(pkg_name=pkg_name, file_name=controllers_file, ns=ns):
                rospy.loginfo("Robot controllers loaded successfully")
            else:
                rospy.logerr("Error while loading robot controllers")
                return False
        else:
            if ros_common.ros_load_yaml(pkg_name=controller_package_name, file_name=controllers_file, ns=ns):
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


def _ros_to_mjcf_quat(ori_x: float, ori_y: float, ori_z: float, ori_w: float) -> str:
    """Convert a ROS quaternion (x, y, z, w) to an MJCF quaternion string (w x y z)."""
    return f"{ori_w} {ori_x} {ori_y} {ori_z}"


def _primitive_body_xml(model_name: str, geom_type: str, size: Sequence[float],
                        pos_x: float, pos_y: float, pos_z: float,
                        quat_str: str, mass: Optional[float],
                        rgba: Optional[Sequence[float]], free_joint: bool) -> str:
    """Build an MJCF ``<body>`` snippet for a primitive geom."""
    size_str = " ".join(str(s) for s in size)
    free = f'<freejoint name="{model_name}_freejoint"/>' if free_joint else ''
    mass_attr = f' mass="{mass}"' if mass is not None else ''
    rgba_attr = f' rgba="{" ".join(str(c) for c in rgba)}"' if rgba is not None else ''
    return (f'<body name="{model_name}" pos="{pos_x} {pos_y} {pos_z}" quat="{quat_str}">'
            f'{free}'
            f'<geom name="{model_name}_geom" type="{geom_type}" size="{size_str}"{mass_attr}{rgba_attr}/>'
            f'</body>')


class MujocoSceneManager:
    """
    Maintain an authoritative MuJoCo scene graph and apply it with the ``reload`` service.

    The scene is composed of a fixed base scene (typically the robot and the static world) plus a
    set of objects added at run time. Adding or removing an object regenerates the merged MJCF and
    reloads it, which provides spawn/delete semantics analogous to the Gazebo model services.

    A reload re-initialises the whole simulation, so it is suited to per-episode changes in the set
    of objects rather than per-step updates. To reposition an object that is already present in the
    scene (declared with a free joint), ``mujoco_set_body_state`` is faster than a reload.

    When the base scene is provided as a file, its referenced assets (meshes, textures) remain
    resolvable after composition: the compiler asset directories are resolved to absolute paths so
    the regenerated scene can be written to any working directory.
    """

    def __init__(self, base_scene_path: Optional[str] = None, base_scene_string: Optional[str] = None,
                 server_name: str = DEFAULT_SERVER_NAME, ros_port: Optional[str] = None,
                 work_dir: Optional[str] = None):
        """
        Initialize the scene manager.

        Args:
            base_scene_path (str): Absolute path to the base MJCF/XML scene. Recommended when the
                scene references external assets.
            base_scene_string (str): A raw MJCF string for the base scene (used if base_scene_path
                is None). Best suited to self-contained scenes that use only primitive geoms.
            server_name (str): Graph name of the server node.
            ros_port (str): The ROS_MASTER_URI port (optional).
            work_dir (str): Directory for the regenerated scene files. Defaults to the system
                temporary directory.
        """
        _require_mujoco_msgs()

        if base_scene_path is not None:
            with open(base_scene_path, 'r') as f:
                self._base_xml = f.read()
            self._base_dir = os.path.dirname(os.path.abspath(base_scene_path))
        elif base_scene_string is not None:
            self._base_xml = base_scene_string
            self._base_dir = None
        else:
            raise ValueError("Either base_scene_path or base_scene_string must be provided")

        self.server_name = server_name
        self.ros_port = ros_port
        self.work_dir = work_dir if work_dir is not None else tempfile.gettempdir()

        # name -> {"body_xml": str, "asset_xml": Optional[str]}
        self._objects: Dict[str, Dict[str, Optional[str]]] = {}
        self._last_written: Optional[str] = None
        self._counter = 0

    def spawn_primitive(self, model_name: str, geom_type: str = "box",
                        size: Sequence[float] = (0.025, 0.025, 0.025),
                        pos_x: float = 0.0, pos_y: float = 0.0, pos_z: float = 0.0,
                        ori_x: float = 0.0, ori_y: float = 0.0, ori_z: float = 0.0, ori_w: float = 1.0,
                        mass: Optional[float] = 0.1, rgba: Optional[Sequence[float]] = (1.0, 0.0, 0.0, 1.0),
                        free_joint: bool = True, reload: bool = True) -> Tuple[bool, str]:
        """
        Add a primitive-geom object to the scene.

        Args:
            model_name (str): The unique name of the object.
            geom_type (str): The MuJoCo geom type (box, sphere, cylinder, capsule, ...). Defaults to "box".
            size (Sequence[float]): The geom size, following MuJoCo conventions (for a box these are
                half-extents). Defaults to (0.025, 0.025, 0.025).
            pos_x (float): The x position of the object. Defaults to 0.0.
            pos_y (float): The y position of the object. Defaults to 0.0.
            pos_z (float): The z position of the object. Defaults to 0.0.
            ori_x (float): The x component of the orientation quaternion. Defaults to 0.0.
            ori_y (float): The y component of the orientation quaternion. Defaults to 0.0.
            ori_z (float): The z component of the orientation quaternion. Defaults to 0.0.
            ori_w (float): The w component of the orientation quaternion. Defaults to 1.0.
            mass (float): The object mass. Defaults to 0.1.
            rgba (Sequence[float]): The object colour. Defaults to (1.0, 0.0, 0.0, 1.0).
            free_joint (bool): Whether to add a free joint so the object can move / be repositioned. Defaults to True.
            reload (bool): Whether to reload the scene immediately. Defaults to True.

        Returns:
            Tuple[bool, str]: The reload result if reload is True, otherwise (True, "queued").
        """
        quat_str = _ros_to_mjcf_quat(ori_x, ori_y, ori_z, ori_w)
        body_xml = _primitive_body_xml(model_name, geom_type, size, pos_x, pos_y, pos_z,
                                       quat_str, mass, rgba, free_joint)
        self._objects[model_name] = {"body_xml": body_xml, "asset_xml": None}
        if reload:
            return self.reload_scene()
        return True, "queued"

    def spawn_model(self, model_name: str, body_string: Optional[str] = None,
                    body_path: Optional[str] = None, asset_string: Optional[str] = None,
                    reload: bool = True) -> Tuple[bool, str]:
        """
        Add an object to the scene from a raw MJCF ``<body>`` snippet.

        Args:
            model_name (str): The unique name of the object.
            body_string (str): An MJCF ``<body>...</body>`` snippet for the object.
            body_path (str): Path to a file containing the ``<body>`` snippet (used if body_string is None).
            asset_string (str): Optional MJCF asset elements (meshes, materials, ...) required by the body.
            reload (bool): Whether to reload the scene immediately. Defaults to True.

        Returns:
            Tuple[bool, str]: The reload result if reload is True, otherwise (True, "queued").
        """
        if body_string is None and body_path is not None:
            with open(body_path, 'r') as f:
                body_string = f.read()
        if body_string is None:
            rospy.logerr("Invalid input: both body_string and body_path are None")
            return False, "Error: no body snippet supplied"

        self._objects[model_name] = {"body_xml": body_string, "asset_xml": asset_string}
        if reload:
            return self.reload_scene()
        return True, "queued"

    def remove_model(self, model_name: str, reload: bool = True) -> Tuple[bool, str]:
        """
        Remove an object from the scene.

        Args:
            model_name (str): The name of the object to remove.
            reload (bool): Whether to reload the scene immediately. Defaults to True.

        Returns:
            Tuple[bool, str]: The reload result if reload is True, otherwise (True, "queued").
        """
        if model_name in self._objects:
            del self._objects[model_name]
        else:
            rospy.logwarn(f"Object '{model_name}' is not in the scene.")
        if reload:
            return self.reload_scene()
        return True, "queued"

    def get_model_names(self) -> List[str]:
        """
        Get the names of the objects currently added to the scene.

        Returns:
            list[str]: The object names managed by this scene manager.
        """
        return list(self._objects.keys())

    def clear(self, reload: bool = False) -> Tuple[bool, str]:
        """
        Remove all added objects, restoring the base scene.

        Args:
            reload (bool): Whether to reload the scene immediately. Defaults to False.

        Returns:
            Tuple[bool, str]: The reload result if reload is True, otherwise (True, "queued").
        """
        self._objects.clear()
        if reload:
            return self.reload_scene()
        return True, "queued"

    def reload_scene(self) -> Tuple[bool, str]:
        """
        Regenerate the merged MJCF from the base scene plus the added objects and reload it.

        Returns:
            Tuple[bool, str]: A tuple containing a boolean indicating whether the reload succeeded
                              and a status message.
        """
        composed_xml = self._compose()

        self._counter += 1
        out_path = os.path.join(self.work_dir, f"multiros_scene_{os.getpid()}_{self._counter}.xml")
        try:
            with open(out_path, 'w') as f:
                f.write(composed_xml)
        except OSError as e:
            rospy.logerr(f"Failed to write the composed scene to {out_path}: {e}")
            return False, f"Error: {e}"

        # Remove the previously written scene file (best effort).
        if self._last_written is not None and os.path.exists(self._last_written):
            try:
                os.remove(self._last_written)
            except OSError:
                pass
        self._last_written = out_path

        return mujoco_reload(model_path=out_path, server_name=self.server_name, ros_port=self.ros_port)

    def close(self) -> None:
        """
        Remove the last generated scene file from disk.

        Call this when the scene manager is no longer needed (for example when the environment
        closes) so that repeated training sessions do not accumulate generated MJCF files in the
        working directory.
        """
        if self._last_written is not None and os.path.exists(self._last_written):
            try:
                os.remove(self._last_written)
            except OSError:
                pass
        self._last_written = None

    def _compose(self) -> str:
        """Build the merged MJCF string from the base scene and the added objects."""
        root = ET.fromstring(self._base_xml)

        # When the base scene is a file, resolve the compiler asset directories to absolute paths so
        # the regenerated scene resolves meshes / textures regardless of where it is written.
        if self._base_dir is not None:
            compiler = root.find('compiler')
            if compiler is None:
                compiler = ET.SubElement(root, 'compiler')
            for attr in ('meshdir', 'texturedir'):
                val = compiler.get(attr)
                if val is None:
                    compiler.set(attr, self._base_dir)
                elif not os.path.isabs(val):
                    compiler.set(attr, os.path.abspath(os.path.join(self._base_dir, val)))
            assetdir = compiler.get('assetdir')
            if assetdir is not None and not os.path.isabs(assetdir):
                compiler.set('assetdir', os.path.abspath(os.path.join(self._base_dir, assetdir)))

        worldbody = root.find('worldbody')
        if worldbody is None:
            worldbody = ET.SubElement(root, 'worldbody')
        asset = root.find('asset')

        for spec in self._objects.values():
            worldbody.append(ET.fromstring(spec["body_xml"]))
            asset_xml = spec.get("asset_xml")
            if asset_xml:
                if asset is None:
                    asset = ET.SubElement(root, 'asset')
                for child in ET.fromstring(f"<asset>{asset_xml}</asset>"):
                    asset.append(child)

        return ET.tostring(root, encoding='unicode')
