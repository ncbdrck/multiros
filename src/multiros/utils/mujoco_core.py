#! /usr/bin/env python

"""
THE MUJOCO BRIDGE SCRIPT PROVIDES ALL THE MAIN FUNCTIONS FOR ROS TO COMMUNICATE WITH MUJOCO.

The MuJoCo simulation is driven through the mujoco_ros_pkgs server node (mujoco_ros), which
exposes its controls as ROS services and an actionlib action under the node's private
namespace (by default ``/mujoco_server/``):

https://github.com/ubi-agni/mujoco_ros_pkgs

This script provides the following functionality:
  1. launch_mujoco: Launch the MuJoCo server using ROS.
  2. close_mujoco: Close a running MuJoCo server instance.
  3. reset_mujoco: Reset the MuJoCo simulation to its initial state.
  4. pause_mujoco: Pause the MuJoCo simulation.
  5. unpause_mujoco: Unpause the MuJoCo simulation.
  6. mujoco_step: Step the MuJoCo simulation a fixed number of iterations.
"""

import rospy
import rospkg
import os
import signal
import subprocess
import time
from std_srvs.srv import Empty
from typing import Optional, Tuple
from multiros.utils import ros_common

# The message and action types live in mujoco_ros_pkgs, which is only required for the
# MuJoCo backend. Import them lazily so that importing this module (and the wider package)
# does not fail on installations that only use the Gazebo backend.
try:
    import actionlib
    from mujoco_ros_msgs.srv import SetPause, SetFloat, SetGravity, GetSimInfo
    from mujoco_ros_msgs.msg import StepAction, StepGoal

    _MUJOCO_MSGS_AVAILABLE = True
except ImportError:
    _MUJOCO_MSGS_AVAILABLE = False

# The graph name of the mujoco_ros server node. All services and the step action are
# advertised under this node's private namespace.
DEFAULT_SERVER_NAME = "mujoco_server"

# Cache of step action clients keyed by server name (one ROS master per process).
_step_clients: dict = {}


def _require_mujoco_msgs() -> None:
    """Raise a clear error if the mujoco_ros_pkgs Python interfaces are unavailable."""
    if not _MUJOCO_MSGS_AVAILABLE:
        raise ImportError(
            "mujoco_ros_msgs/actionlib could not be imported. Build mujoco_ros_pkgs in the "
            "workspace to use the MuJoCo backend."
        )


def _bool_arg(value: bool) -> str:
    """Format a boolean as a roslaunch argument value ('true'/'false')."""
    return str(bool(value)).lower()


"""
   1. launch_mujoco: Launch the MuJoCo server using ROS.
"""


def launch_mujoco(launch_roscore: bool = True,
                  port: Optional[int] = None,
                  paused: bool = False,
                  use_sim_time: bool = True,
                  model_path: Optional[str] = None,
                  model_pkg: Optional[str] = None,
                  model_name: Optional[str] = None,
                  headless: bool = True,
                  no_render: bool = False,
                  realtime: Optional[str] = None,
                  mujoco_plugin_config: Optional[str] = None,
                  initial_joint_states: Optional[str] = None,
                  server_name: str = DEFAULT_SERVER_NAME,
                  ns: str = "",
                  verbose: bool = False,
                  output: str = 'screen',
                  launch_new_term: bool = True) -> Tuple[str, subprocess.Popen]:
    """
    Launch the MuJoCo server using ROS.

    The available options map onto the arguments of the mujoco_ros launch file
    (mujoco_ros/launch/launch_server.launch).

    Args:
        launch_roscore (bool): If True, launch a roscore together with MuJoCo. Defaults to True.
        port (int): If "launch_roscore" is True, launch the roscore on the given port.
        paused (bool): Whether to start the simulation paused. Defaults to False.
        use_sim_time (bool): Publish simulation time over /clock. The server requires this to be
            explicitly set, so it is always forwarded to the launch file. Defaults to True.
        model_path (str): Absolute path to the MuJoCo MJCF/XML scene to load (optional).
        model_pkg (str): If "model_path" is None, the package containing the scene file (optional).
        model_name (str): If "model_pkg" is set, the scene file path relative to that package.
        headless (bool): Run without the interactive viewer window. Defaults to True.
        no_render (bool): Disable on- and off-screen rendering entirely. Defaults to False.
        realtime (str): Fraction of real time in (0, 1], or "-1" to run as fast as possible.
            None uses the value defined in the model.
        mujoco_plugin_config (str): Path to a YAML file with plugin configurations to load
            (for example mujoco_ros_control or sensor plugins) (optional).
        initial_joint_states (str): Path to a YAML file with initial joint states (optional).
        server_name (str): Graph name of the server node, used to resolve the service and action
            names. The bundled mujoco_ros launch file always names the node "mujoco_server"; a
            different value is only valid when attaching to a server started by a custom launch
            file that renames the node. Defaults to "mujoco_server".
        ns (str): Value of the server node's "ns" parameter. This is read by plugins and does
            NOT wrap the node's services in a ROS namespace. Defaults to "".
        verbose (bool): Print additional debug output. Defaults to False.
        output (str): The output method for the node (screen or log). Defaults to 'screen'.
        launch_new_term (bool): Launch the node in a new terminal (Xterm). Defaults to True.

    Returns:
        Tuple: A tuple containing the ROS port and the process object for the launched instance.
    """

    rospack = rospkg.RosPack()
    try:
        rospack.get_path('mujoco_ros')
    except rospkg.common.ResourceNotFound:
        rospy.logerr("The package mujoco_ros was not found!")
        return None, None

    # The bundled launch file always names the node "mujoco_server". Warn if the caller
    # expects a different name, since the readiness wait and all later service calls resolve
    # names from server_name and would otherwise target a node that does not exist.
    if server_name != DEFAULT_SERVER_NAME:
        rospy.logwarn(f"launch_mujoco was given server_name='{server_name}', but the bundled "
                      f"launch file names the node '{DEFAULT_SERVER_NAME}'. The launched server "
                      f"will be '{DEFAULT_SERVER_NAME}'; service resolution may fail.")

    # Term command to start the MuJoCo server
    term_cmd = "roslaunch mujoco_ros launch_server.launch "

    # The server node requires /use_sim_time to be set explicitly.
    term_cmd += " use_sim_time:=" + _bool_arg(use_sim_time)

    # Start the simulation paused or running (the launch file's "unpause" argument).
    term_cmd += " unpause:=" + _bool_arg(not paused)

    # Run without the interactive viewer window.
    term_cmd += " headless:=" + _bool_arg(headless)

    # Disable rendering completely (overrides headless/offscreen rendering).
    term_cmd += " no_render:=" + _bool_arg(no_render)

    # Print additional debug output.
    term_cmd += " verbose:=" + _bool_arg(verbose)

    # Forward the namespace.
    if ns:
        term_cmd += " ns:=" + str(ns)

    # Real-time factor (e.g. "-1" runs as fast as possible).
    if realtime is not None:
        term_cmd += " realtime:=" + str(realtime)

    # Plugin configuration (ros_control, sensors, ...).
    if mujoco_plugin_config is not None:
        term_cmd += " mujoco_plugin_config:=" + str(mujoco_plugin_config)

    # Initial joint states.
    if initial_joint_states is not None:
        term_cmd += " initial_joint_states:=" + str(initial_joint_states)

    # Select the scene model
    if model_path is not None:
        if os.path.exists(model_path) is False:
            rospy.logerr("Model file in " + model_path + " does not exist!")
            return None, None
        term_cmd += " modelfile:=" + str(model_path)

    elif model_pkg and model_name is not None:
        try:
            model_pkg_path = rospack.get_path(model_pkg)
        except rospkg.common.ResourceNotFound:
            rospy.logwarn("Package where the model file is located was NOT FOUND!")
            return None, None

        model_file_path = model_pkg_path + "/" + model_name
        if os.path.exists(model_file_path) is False:
            rospy.logerr("Model file in " + model_file_path + " does not exist!")
            return None, None
        term_cmd += " modelfile:=" + str(model_file_path)

    # initializing the var for "launch_roscore"
    ros_port = None

    if launch_roscore:
        if port is not None:
            ros_port, _ = ros_common.launch_roscore(port=int(port))
        else:
            ros_port, _ = ros_common.launch_roscore()

    # Snapshot existing server PIDs so we can identify the ones THIS launch creates
    # (and only kill those on Ctrl+C).
    pre_mujoco_pids = _mujoco_pids()

    # Launch the MuJoCo server
    if launch_new_term:
        term_cmd = f"xterm -e '{term_cmd}'"
        process = subprocess.Popen(term_cmd, shell=True)
    else:
        process = subprocess.Popen(term_cmd, shell=True)

    # Register the launch process for cleanup immediately, before the readiness wait, so a
    # Ctrl+C or a failed launch does not leave the wrapper (or the server it spawned) behind.
    ros_common.register_managed_process(
        process,
        kind="mujoco",
    )

    time.sleep(5.0)

    # waiting for the server to finish launching
    readiness_service = f"/{server_name}/get_sim_info"
    try:
        rospy.wait_for_service(readiness_service, timeout=30.0)
    except rospy.ROSException as e:
        rospy.logerr(f"Timeout (30s) waiting for service '{readiness_service}' "
                     f"after launching MuJoCo: {e}")
        # Tear down the partially-launched server so it does not orphan.
        new_mujoco_pids = sorted(_mujoco_pids() - pre_mujoco_pids)
        for pid in new_mujoco_pids:
            try:
                os.kill(int(pid), signal.SIGTERM)
            except (ProcessLookupError, PermissionError, ValueError):
                pass
        try:
            process.terminate()
        except Exception:
            pass
        return None, None

    # Identify the server PIDs spawned by THIS launch (set diff vs. the snapshot taken before
    # Popen) and register them so SIGINT / atexit can clean them up even if the real node
    # detached from the launch wrapper.
    new_mujoco_pids = sorted(_mujoco_pids() - pre_mujoco_pids)
    if new_mujoco_pids:
        ros_common.register_managed_process(
            process,
            mujoco_pids=new_mujoco_pids,
            kind="mujoco",
        )

    # if launch_roscore is False, ignore the first return
    return ros_port, process


def _mujoco_pids() -> set:
    """Return PIDs of currently running mujoco_node processes."""
    pids: set = set()
    for name in ("mujoco_node",):
        try:
            out = subprocess.run(
                ["pgrep", "-x", name],
                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                timeout=5,
            )
            pids.update(int(p) for p in out.stdout.decode().split() if p)
        except Exception:
            pass
    return pids


"""
   2. close_mujoco: Close a running MuJoCo server instance.
"""


def close_mujoco(process: subprocess.Popen, ros_port: Optional[str] = None,
                 server_name: str = DEFAULT_SERVER_NAME) -> bool:
    """
    Function to close a MuJoCo server instance.

    Args:
        process: A subprocess.Popen object representing the running MuJoCo instance.
        ros_port (str): The ROS port used by the MuJoCo instance. Defaults to None. (optional).
        server_name (str): Graph name of the server node, used to call its shutdown service.
            Defaults to "mujoco_server".

    Returns:
        bool: True if the instance was closed successfully, False otherwise.
    """

    # change the rosmaster
    if ros_port is not None:
        ros_common.change_ros_master(ros_port=ros_port)

    # Ask the server to shut down cleanly first. This is scoped to this instance's ROS master
    # (selected above), so it does not affect other concurrent servers. Best effort: fall back
    # to terminating the launch process if the service is unavailable.
    shutdown_service = f"/{server_name}/shutdown"
    try:
        rospy.wait_for_service(shutdown_service, timeout=2.0)
        rospy.ServiceProxy(shutdown_service, Empty)()
    except Exception:
        pass

    process.terminate()

    rospy.logdebug("Closing MuJoCo!")

    return True


"""
   3. reset_mujoco: Reset the MuJoCo simulation to its initial state.
"""


def reset_mujoco(reset_type: str = "simulation", max_tries: int = 5,
                 server_name: str = DEFAULT_SERVER_NAME, ros_port: Optional[str] = None) -> bool:
    """
    Function to reset the MuJoCo simulation.

    The MuJoCo server provides a single reset that restores the model's default configuration
    and re-applies the configured initial joint states. The "reset_type" argument is accepted
    for interface compatibility with the Gazebo backend; both values map to the same reset.

    Args:
        reset_type (str): Accepted for interface compatibility ("simulation" or "world").
        max_tries (int): The maximum number of tries to reset the simulation (optional).
        server_name (str): Graph name of the server node (optional).
        ros_port (str): The ROS_MASTER_URI port (optional).

    Returns:
        bool: True if the service call was successful, False otherwise.
    """

    # Change the ROS master environment variable if provided
    if ros_port is not None:
        ros_common.change_ros_master(ros_port=ros_port)

    service_name = f"/{server_name}/reset"

    # Wait for the service to be available
    try:
        rospy.wait_for_service(service_name, timeout=30.0)
    except rospy.ROSException as e:
        rospy.logerr(f"Timeout (30s) waiting for service '{service_name}': {e}")
        return False

    # Try to reset the simulation up to max_tries times
    for i in range(max_tries):
        try:
            reset_service = rospy.ServiceProxy(service_name, Empty)
            reset_service()

            rospy.loginfo("Reset successful!")
            return True

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    return False


"""
   4. pause_mujoco: Pause the MuJoCo simulation.
"""


def pause_mujoco(max_tries: int = 5, server_name: str = DEFAULT_SERVER_NAME,
                 ros_port: Optional[str] = None) -> bool:
    """
    Function to pause the MuJoCo simulation.

    Args:
        max_tries (int): The maximum number of tries to pause the simulation (optional).
        server_name (str): Graph name of the server node (optional).
        ros_port (str): The ROS_MASTER_URI port (optional).

    Returns:
        bool: True if the service call was successful, False otherwise.
    """
    _require_mujoco_msgs()

    # Change the ROS master environment variable if provided
    if ros_port is not None:
        ros_common.change_ros_master(ros_port=ros_port)

    service_name = f"/{server_name}/set_pause"

    # Wait for the 'set_pause' service to be available
    try:
        rospy.wait_for_service(service_name, timeout=30.0)
    except rospy.ROSException as e:
        rospy.logerr(f"Timeout (30s) waiting for service '{service_name}': {e}")
        return False

    # Try to pause the simulation up to max_tries times
    for i in range(max_tries):
        try:
            set_pause = rospy.ServiceProxy(service_name, SetPause)
            response = set_pause(paused=True)
            if response.success:
                rospy.logdebug("Pause successful!")
                return True

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    return False


"""
   5. unpause_mujoco: Unpause the MuJoCo simulation.
"""


def unpause_mujoco(max_tries: int = 5, server_name: str = DEFAULT_SERVER_NAME,
                   ros_port: Optional[str] = None) -> bool:
    """
    Function to unpause the MuJoCo simulation.

    Args:
        max_tries (int): The maximum number of tries to unpause the simulation (optional).
        server_name (str): Graph name of the server node (optional).
        ros_port (str): The ROS_MASTER_URI port (optional).

    Returns:
        bool: True if the service call was successful, False otherwise.
    """
    _require_mujoco_msgs()

    # Change the ROS master environment variable if provided
    if ros_port is not None:
        ros_common.change_ros_master(ros_port=ros_port)

    service_name = f"/{server_name}/set_pause"

    # Wait for the 'set_pause' service to be available
    try:
        rospy.wait_for_service(service_name, timeout=30.0)
    except rospy.ROSException as e:
        rospy.logerr(f"Timeout (30s) waiting for service '{service_name}': {e}")
        return False

    # Try to unpause the simulation up to max_tries times
    for i in range(max_tries):
        try:
            set_pause = rospy.ServiceProxy(service_name, SetPause)
            response = set_pause(paused=False)
            if response.success:
                rospy.logdebug("Unpause successful!")
                return True

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    return False


"""
   6. mujoco_step: Step the MuJoCo simulation a fixed number of iterations.
"""


def mujoco_step(steps: int, server_name: str = DEFAULT_SERVER_NAME,
                ros_port: Optional[str] = None, timeout: float = 30.0) -> bool:
    """
    Function to step the MuJoCo simulation a fixed number of iterations.

    The server only executes stepping while the simulation is paused, so the caller must pause
    the simulation before requesting steps.

    Args:
        steps (int): The number of steps to advance the simulation.
        server_name (str): Graph name of the server node (optional).
        ros_port (str): The ROS_MASTER_URI port (optional).
        timeout (float): Seconds to wait for the action server and the result. Defaults to 30.0.

    Returns:
        bool: True if the steps completed successfully, False otherwise.
    """
    _require_mujoco_msgs()

    # The Step action's num_steps field is a uint16; reject values outside its range early.
    if not isinstance(steps, int) or isinstance(steps, bool) or not (1 <= steps <= 65535):
        raise ValueError(f"steps must be an integer in [1, 65535], got {steps!r}")

    # Change the ROS master environment variable if provided
    if ros_port is not None:
        ros_common.change_ros_master(ros_port=ros_port)

    try:
        client = _step_clients.get(server_name)
        if client is None:
            client = actionlib.SimpleActionClient(f"/{server_name}/step", StepAction)
            if not client.wait_for_server(rospy.Duration(timeout)):
                rospy.logerr(f"Timeout ({timeout}s) waiting for the MuJoCo step action server.")
                return False
            _step_clients[server_name] = client

        client.send_goal(StepGoal(num_steps=steps))
        client.wait_for_result(rospy.Duration(timeout))
        result = client.get_result()
        return bool(result is not None and result.success)

    except Exception as e:
        rospy.logerr(f"An error occurred while stepping the physics in MuJoCo: {e}")
        return False
