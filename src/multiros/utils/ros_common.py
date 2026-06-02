#! /usr/bin/env python

"""
Common helpers for handling ROS from inside the multiros framework.

Functions provided:

- ``launch_roscore`` — launch a roscore with a given or random (non-overlapping) port.
- ``change_ros_gazebo_master`` — change the current ROS and Gazebo master environment variables.
- ``get_all_the_ros_masters`` — get all the currently-known rosmaster ports (diagnostic).
- ``add_to_rosmaster_list`` — add a port to the diagnostic port log.
- ``remove_from_rosmaster_list`` — remove a port from the diagnostic port log.
- ``kill_all_host_ros_and_gazebo`` — kill ALL ROS and Gazebo processes on this host.
- ``kill_all_host_roslaunch_processes`` — kill ALL roslaunch processes on this host.
- ``kill_all_ros_nodes`` — kill every node attached to the current rosmaster.
- ``kill_ros_node`` — kill a single named ROS node.
- ``ros_kill_master`` — kill a specific rosmaster by port.
- ``clean_ros_logs`` — purge ROS logs.
- ``source_workspace`` — deprecated no-op.
- ``ros_launch_launcher`` — execute a roslaunch with args.
- ``ros_node_launcher`` — launch a ROS node from a package.
- ``ros_load_yaml`` — fetch and load a YAML file onto the ROS parameter server.
- ``load_urdf`` — load a URDF onto the parameter server.
- ``is_roscore_running`` — check whether a roscore is reachable.
- ``change_ros_master`` — set ROS_MASTER_URI for this process.
- ``change_ros_master_multi_device`` — set ROS_MASTER_URI for cross-machine use.
- ``change_gazebo_master`` — set GAZEBO_MASTER_URI for this process.
- ``init_robot_state_publisher`` — start the robot_state_publisher node.
- ``remove_all_from_rosmaster_list`` — truncate the diagnostic port log.
"""
import rosparam
import rospy
import rospkg
import os
import subprocess
import time
import random
import socket
import warnings
import fcntl
import atexit
import signal
import threading
import xacro
from typing import Any, Dict, List, Optional, Tuple, Union

# Best-effort log of ports we've launched roscores on. Not load-bearing
# for allocation correctness: the kernel decides what's free via
# socket.bind(0); this file is just for diagnostics ("which roscores did
# my multiros/realros scripts spawn?"). flock-protected so two parallel
# scripts can both append safely.
_PORT_LOG_PATH = '/tmp/ros_master_ports_multiros.txt'

# ----------------------------------------------------------------------
# Managed-process registry: track the roscore / Gazebo processes this
# Python process spawned so they can be cleaned up on Ctrl+C or
# normal exit. Scoped to processes the calling script started — the
# host-wide ``kill_all_host_*`` helpers below are a separate tool.
# ----------------------------------------------------------------------
_managed_lock = threading.Lock()
_managed_processes: List[Dict[str, Any]] = []
_cleanup_done = False
_handlers_installed = False
_prev_sigint_handler: Any = None


def register_managed_process(popen, **selectors) -> None:
    """
    Register a process spawned by this script for automatic cleanup
    on Ctrl+C or normal interpreter exit.

    Cleanup is scoped: only processes registered here are touched.
    Pre-existing ROS/Gazebo sessions on the host are NOT affected
    (unlike ``kill_all_host_ros_and_gazebo``).

    Args:
        popen: A ``subprocess.Popen`` (typically the xterm wrapper
            shell). ``.terminate()`` will be called on cleanup.
        **selectors: Optional fallback identifiers used by cleanup
            when terminating the Popen alone is not enough (e.g. the
            real child detached from the wrapper). Recognised keys
            are ``roscore_port`` (str | int), which triggers
            ``pkill -f "roscore -p <port>"``; ``gazebo_pids`` and
            ``mujoco_pids`` (list[int]), PIDs that will be SIGTERM'd
            then SIGKILL'd; and ``kind`` (str), a free-form label used
            in log lines.
    """
    global _handlers_installed, _prev_sigint_handler
    with _managed_lock:
        _managed_processes.append({"popen": popen, "selectors": dict(selectors)})
        already_installed = _handlers_installed
        _handlers_installed = True

    if not already_installed:
        atexit.register(_cleanup_managed_processes)
        try:
            _prev_sigint_handler = signal.signal(signal.SIGINT, _sigint_handler)
        except ValueError:
            # signal.signal can only be called from the main thread;
            # atexit will still fire on normal interpreter shutdown.
            _prev_sigint_handler = None

        # Also register with rospy's shutdown machinery. This matters
        # because ``rospy.init_node`` (typically called by the user's
        # training script AFTER launch_roscore / launch_gazebo)
        # installs its OWN SIGINT handler that overwrites ours — so
        # our SIGINT handler would never fire on Ctrl+C. rospy's
        # shutdown callback list is iterated regardless of who owns
        # the signal handler, so wiring our cleanup in here is what
        # actually makes Ctrl+C tear down roscore/Gazebo in the
        # typical "import multiros; rospy.init_node; train" flow.
        try:
            rospy.on_shutdown(_cleanup_managed_processes)
        except Exception:
            # rospy unavailable / not initialised yet / etc. Atexit
            # + SIGINT handler still cover the non-rospy cases.
            pass


def _sigint_handler(signum, frame):
    """
    On Ctrl+C: tear down managed processes once, then chain to the
    previously-installed SIGINT handler (or KeyboardInterrupt) so the
    user's normal interrupt semantics still apply. A second Ctrl+C
    during cleanup will use the chained handler and exit immediately.
    """
    # Restore the previous handler BEFORE doing cleanup so a second
    # Ctrl+C during cleanup hits the default and bails out hard.
    try:
        signal.signal(signal.SIGINT, _prev_sigint_handler if _prev_sigint_handler else signal.SIG_DFL)
    except (ValueError, TypeError):
        signal.signal(signal.SIGINT, signal.SIG_DFL)

    try:
        rospy.loginfo("SIGINT received; cleaning up spawned roscore/Gazebo...")
    except Exception:
        print("[multiros] SIGINT received; cleaning up spawned roscore/Gazebo...")

    _cleanup_managed_processes()

    # Chain: call the previous handler or raise KeyboardInterrupt
    if callable(_prev_sigint_handler) and _prev_sigint_handler not in (signal.SIG_DFL, signal.SIG_IGN):
        try:
            _prev_sigint_handler(signum, frame)
            return
        except Exception:
            pass
    raise KeyboardInterrupt


def _cleanup_managed_processes() -> None:
    """
    Tear down every registered managed process. Idempotent — safe to
    call from both the SIGINT handler and the atexit hook.
    """
    global _cleanup_done
    with _managed_lock:
        if _cleanup_done:
            return
        _cleanup_done = True
        to_cleanup = list(_managed_processes)
        _managed_processes.clear()

    if not to_cleanup:
        return

    # Phase 1: targeted pkill / SIGTERM
    for entry in to_cleanup:
        popen = entry["popen"]
        selectors = entry["selectors"]

        # Kill specific gzserver/gzclient PIDs we captured at launch.
        # Done first so the roslaunch wrapper doesn't try to restart
        # them as we kill it.
        for pid in selectors.get("gazebo_pids", []) or []:
            try:
                os.kill(int(pid), signal.SIGTERM)
            except (ProcessLookupError, PermissionError, ValueError):
                pass

        # Kill specific mujoco_node PIDs we captured at launch, for the
        # same reason as the gzserver/gzclient PIDs above.
        for pid in selectors.get("mujoco_pids", []) or []:
            try:
                os.kill(int(pid), signal.SIGTERM)
            except (ProcessLookupError, PermissionError, ValueError):
                pass

        # Kill the wrapper Popen (xterm/shell). If the popen was started
        # with start_new_session=True (i.e. is its own pgrp leader),
        # killpg the entire group so children like roslaunch + move_group
        # + moveit_python_interface die alongside the xterm. Otherwise
        # plain terminate() suffices (and killpg would risk hitting the
        # parent's pgrp).
        try:
            if popen is not None and popen.poll() is None:
                try:
                    pgid = os.getpgid(popen.pid)
                    is_session_leader = pgid == popen.pid
                except (ProcessLookupError, PermissionError):
                    is_session_leader = False
                if is_session_leader:
                    try:
                        os.killpg(pgid, signal.SIGTERM)
                    except (ProcessLookupError, PermissionError):
                        pass
                else:
                    popen.terminate()
        except Exception:
            pass

        # Targeted pkill by roscore port — uniquely identifies OUR
        # roscore even if xterm detached.
        port = selectors.get("roscore_port")
        if port:
            try:
                subprocess.run(
                    ["pkill", "-f", f"roscore -p {port}"],
                    timeout=5,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
            except Exception:
                pass

    # Give children a moment to exit gracefully.
    time.sleep(0.5)

    # Phase 2: SIGKILL anything still alive.
    for entry in to_cleanup:
        popen = entry["popen"]
        selectors = entry["selectors"]
        try:
            if popen is not None and popen.poll() is None:
                try:
                    pgid = os.getpgid(popen.pid)
                    is_session_leader = pgid == popen.pid
                except (ProcessLookupError, PermissionError):
                    is_session_leader = False
                if is_session_leader:
                    try:
                        os.killpg(pgid, signal.SIGKILL)
                    except (ProcessLookupError, PermissionError):
                        pass
                else:
                    popen.kill()
        except Exception:
            pass
        for pid in selectors.get("gazebo_pids", []) or []:
            try:
                os.kill(int(pid), signal.SIGKILL)
            except (ProcessLookupError, PermissionError, ValueError):
                pass
        for pid in selectors.get("mujoco_pids", []) or []:
            try:
                os.kill(int(pid), signal.SIGKILL)
            except (ProcessLookupError, PermissionError, ValueError):
                pass

    # Restore SIGINT to the default handler. Useful for the common
    # case where this cleanup runs via rospy.on_shutdown: rospy may
    # have installed its own SIGINT handler that doesn't terminate
    # the script, so the user has to Ctrl+C multiple times. Resetting
    # to SIG_DFL here means any further Ctrl+C kills the process
    # immediately rather than re-entering rospy's handler.
    try:
        signal.signal(signal.SIGINT, signal.SIG_DFL)
    except (ValueError, OSError):
        pass


"""
    01. launch_roscore: To launch a rocore with a given or random (no overlapping) port
"""


def _port_is_free(port: int) -> bool:
    """
    Return True iff ``port`` can be bound on localhost right now.

    Uses SO_REUSEADDR so the TIME_WAIT state of a previous bind on the
    same port doesn't falsely report it as occupied.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind(('127.0.0.1', port))
        except OSError:
            return False
        return True
    finally:
        sock.close()


def _master_is_reachable(port: int, timeout: float = 1.0) -> bool:
    """
    Return True iff a TCP connection to ``localhost:port`` succeeds
    within ``timeout`` seconds.

    The ROS master is an XMLRPC server listening on the master URI port,
    so a successful connect() is a strong signal that rosmaster is up
    and accepting client registrations. This is the cheap counterpart
    to ``rosgraph.Master().is_online()`` — we don't want to import
    rosgraph just for the readiness check, and connecting directly
    avoids any side effect on the current process's ROS_MASTER_URI.

    Used by ``launch_roscore`` to verify the spawned roscore actually
    came up before we point ROS_MASTER_URI at it. Without this check,
    a silent xterm/roscore failure leaves the caller pointed at a
    phantom master and every downstream ``wait_for_service`` blocks
    for its full 30 s timeout.
    """
    try:
        with socket.create_connection(("127.0.0.1", int(port)), timeout=timeout):
            return True
    except OSError:
        return False


# Track ports we've issued via _reserve_free_port within this Python
# process. The kernel only guarantees port uniqueness while a socket is
# bound — once we close() to read the port back, the kernel can re-issue
# the same port to a subsequent bind(0). Keeping a per-process set lets
# us detect that and pick a different port. Cross-process races aren't
# covered (they're rare in practice because the kernel allocates
# different ephemeral ranges per process under typical settings).
_port_claims_lock = threading.Lock()
_port_claims: set = set()


def _reserve_free_port(_max_retries: int = 20) -> int:
    """
    Ask the kernel for a free ephemeral port via bind(0).

    Returns the port number after closing the socket. The kernel
    guarantees the returned port is free *at this moment*; the
    process-local set ``_port_claims`` ensures we never hand out the
    same port twice within this Python process even if the kernel
    re-issues it after the previous claimant's socket was closed.

    There is still a tiny TOCTOU window between this function
    returning and the caller's subsequent ``roscore -p <port>``
    bind. Mitigation in production: roscore launches quickly after
    the port is picked, well before the kernel would normally
    rotate back to the same ephemeral port.
    """
    for _ in range(_max_retries):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(('127.0.0.1', 0))
            port = sock.getsockname()[1]
        finally:
            sock.close()
        with _port_claims_lock:
            if port not in _port_claims:
                _port_claims.add(port)
                return port
        # Otherwise the kernel re-issued a port we already claimed; loop.
    raise RuntimeError(
        f"Could not reserve a unique free port after {_max_retries} attempts; "
        f"this typically means something is wrong with the ephemeral port pool."
    )


def _append_port_log(ros_port: str) -> None:
    """
    Append ``ros_port`` to the diagnostic port log under flock.

    Best-effort: log-only. Allocation correctness does not depend on
    this file. Failures are logged but not raised.
    """
    try:
        with open(_PORT_LOG_PATH, 'a') as f:
            try:
                fcntl.flock(f.fileno(), fcntl.LOCK_EX)
                f.write(ros_port + ' ')
                f.flush()
            finally:
                try:
                    fcntl.flock(f.fileno(), fcntl.LOCK_UN)
                except OSError:
                    pass
    except OSError as e:
        rospy.logwarn(f"Could not write to port log {_PORT_LOG_PATH}: {e}")


def _remove_from_port_log(ros_port: str) -> None:
    """Best-effort removal of ``ros_port`` from the diagnostic log."""
    try:
        if not os.path.exists(_PORT_LOG_PATH):
            return
        with open(_PORT_LOG_PATH, 'r+') as f:
            try:
                fcntl.flock(f.fileno(), fcntl.LOCK_EX)
                contents = f.read()
                tokens = [t for t in contents.split() if t and t != ros_port]
                f.seek(0)
                f.truncate()
                if tokens:
                    f.write(' '.join(tokens) + ' ')
            finally:
                try:
                    fcntl.flock(f.fileno(), fcntl.LOCK_UN)
                except OSError:
                    pass
    except OSError as e:
        rospy.logwarn(f"Could not edit port log {_PORT_LOG_PATH}: {e}")


def launch_roscore(
    port: Optional[int] = None,
    set_new_master_vars: bool = True,
    allocate_gazebo_port: bool = True,
) -> Tuple[str, Optional[str]]:
    """
    Launch a roscore on a free port and (optionally) point this process's
    ``ROS_MASTER_URI`` / ``GAZEBO_MASTER_URI`` at it.

    Ports are allocated via ``socket.bind(('127.0.0.1', 0))`` so the
    kernel picks free ports and serializes between parallel callers
    on the same host — there's no shared file to race on, and
    anything else squatting on a port in the historical range is
    naturally avoided.

    Args:
        port (int): A specific desired port for ``ROS_MASTER_URI``.
            If the port is unavailable on this host right now, falls
            back to a kernel-allocated free port and logs a warning.
        set_new_master_vars (bool): change the current
            ``ROS_MASTER_URI`` and ``GAZEBO_MASTER_URI`` environment
            variables to the selected ones.
        allocate_gazebo_port (bool): If False, do not reserve a
            ``GAZEBO_MASTER_URI`` port and do not export the var. Set
            this to False from MuJoCo callers — MuJoCo never reads
            ``GAZEBO_MASTER_URI``, so the second port reservation is
            wasted and lengthens the launch path.

    Returns:
        Tuple[str, Optional[str]]: ``(ros_port, gazebo_port)`` as strings.
        ``gazebo_port`` is ``None`` when ``allocate_gazebo_port=False``.
    """

    # Try the caller's requested port first if specified and free.
    if port is not None and _port_is_free(port) and (
        not allocate_gazebo_port or _port_is_free(port + 1)
    ):
        ros_port = str(port)
        gazebo_port = str(port + 1) if allocate_gazebo_port else None
    else:
        if port is not None:
            rospy.logwarn(
                f"Requested port {port} (or {port + 1}) is unavailable; "
                f"falling back to a kernel-allocated free port."
            )
        # Allocate free ports from the kernel. We don't require
        # them to be consecutive — the original "ros_port + 1 for
        # gazebo" convention was just a convenience.
        ros_port = str(_reserve_free_port())
        gazebo_port = str(_reserve_free_port()) if allocate_gazebo_port else None

    # Diagnostic log only; not load-bearing.
    _append_port_log(ros_port)

    # Launch roscore in xterm with verify+retry. The xterm wrapper
    # sometimes returns success (no error from Popen) while the
    # roscore inside it never binds — TIME_WAIT collision from a stale
    # run, xterm DISPLAY issue, or rosout startup failure. Without
    # verification, ``set_new_master_vars`` below points this process
    # at a phantom master and every downstream wait_for_service blocks
    # for its 30 s timeout (and roslaunch inside any subsequent
    # ``launch_gazebo`` xterm starts its OWN rosmaster on an
    # auto-allocated port, so gzserver registers there and the env
    # never finds the /gazebo/* services it expects).
    _MAX_TRIES = 3
    _WAIT_AFTER_LAUNCH = 5.0
    _VERIFY_TIMEOUT = 10.0
    roscore_proc = None
    for _attempt in range(_MAX_TRIES):
        term_cmd = "xterm -e ' " + "roscore -p " + ros_port + "'"
        roscore_proc = subprocess.Popen(term_cmd, shell=True)
        time.sleep(_WAIT_AFTER_LAUNCH)
        if _master_is_reachable(int(ros_port), timeout=_VERIFY_TIMEOUT):
            rospy.loginfo("Roscore launched! with port: " + ros_port)
            break
        rospy.logwarn(
            f"Roscore on port {ros_port} did not come up after "
            f"{_WAIT_AFTER_LAUNCH:.0f}s (attempt {_attempt + 1}/{_MAX_TRIES}); "
            "picking a fresh port and retrying."
        )
        try:
            roscore_proc.terminate()
        except Exception:
            pass
        ros_port = str(_reserve_free_port())
        gazebo_port = str(_reserve_free_port()) if allocate_gazebo_port else None
    else:
        raise RuntimeError(
            f"Could not launch a verifiable roscore after {_MAX_TRIES} "
            "attempts. Check ~/.ros/log/ and that xterm + roscore work "
            "in this terminal (DISPLAY set, no stale rosmaster on the "
            "selected port)."
        )

    # Register for Ctrl+C / atexit cleanup. The xterm wrapper may
    # detach, so we also record the port so cleanup can issue a
    # targeted ``pkill -f "roscore -p <port>"`` that is scoped to
    # OUR roscore (the port is unique to this Python process).
    register_managed_process(roscore_proc, roscore_port=ros_port, kind="roscore")

    # change the current ROS_MASTER anb the GAZEBO_MASTER to the selected one
    if set_new_master_vars:
        change_ros_gazebo_master(ros_port, gazebo_port)

    return ros_port, gazebo_port


"""
    02. change_ros_gazebo_master: Change the current ROS and Gazebo Master Environment Variables
"""


def change_ros_gazebo_master(ros_port: str, gazebo_port: Optional[str] = None) -> bool:
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
    Return a space-separated string of ports recorded in the diagnostic
    port log. Best-effort and diagnostic-only.

    DEPRECATED for allocation use. Port allocation is now handled by
    ``launch_roscore`` via ``socket.bind(0)``; this helper only reflects
    the best-effort log written for human debugging.
    """
    warnings.warn(
        "get_all_the_ros_masters() is deprecated and now reads a "
        "best-effort diagnostic log only. Port allocation is handled "
        "directly by the kernel via socket.bind(0).",
        DeprecationWarning, stacklevel=2,
    )
    if not os.path.exists(_PORT_LOG_PATH):
        return ''
    try:
        with open(_PORT_LOG_PATH, 'r') as f:
            try:
                fcntl.flock(f.fileno(), fcntl.LOCK_SH)
                return f.read().strip()
            finally:
                try:
                    fcntl.flock(f.fileno(), fcntl.LOCK_UN)
                except OSError:
                    pass
    except OSError:
        return ''


"""
    04. add_to_rosmaster_list: Add a port to the current ros master ports list
"""


def add_to_rosmaster_list(ros_port: str) -> bool:
    """
    DEPRECATED. Appends to the diagnostic port log only.

    Port allocation is now handled by ``launch_roscore`` directly via
    ``socket.bind(0)``. Callers do not need to maintain a shared list.
    """
    warnings.warn(
        "add_to_rosmaster_list() is deprecated and now only writes "
        "to a diagnostic log. Port allocation is handled by "
        "socket.bind(0) inside launch_roscore().",
        DeprecationWarning, stacklevel=2,
    )
    _append_port_log(ros_port)
    return True


"""
    05. remove_from_rosmaster_list: remove a port from the current ros master ports list
"""


def remove_from_rosmaster_list(ros_port: str) -> bool:
    """
    DEPRECATED. Removes ``ros_port`` from the diagnostic log only.

    Port allocation is now handled by ``launch_roscore`` directly via
    ``socket.bind(0)``. Callers do not need to maintain a shared list.
    """
    warnings.warn(
        "remove_from_rosmaster_list() is deprecated and now only edits "
        "a diagnostic log. Port allocation is handled by socket.bind(0) "
        "inside launch_roscore().",
        DeprecationWarning, stacklevel=2,
    )
    _remove_from_port_log(ros_port)
    return True


"""
    06. kill_all_host_ros_and_gazebo: Kill ALL ROS and Gazebo instances on this host.
"""


def kill_all_host_ros_and_gazebo() -> bool:
    """
    Kill EVERY rosmaster/roslaunch/gzserver/gzclient/rosout/
    robot_state_publisher/nodelet process on this host (not just the
    ones this script spawned).

    Implemented via ``killall -9`` so it is host-scoped: if another
    user or another script on the same machine is running ROS or
    Gazebo, this will kill their processes too. Use with care.

    Renamed from ``kill_all_ros_and_gazebo`` to make the host-wide
    scope explicit. The old name is kept as a DeprecationWarning-
    emitting alias for backwards compatibility.

    Returns:
        bool: True if the kill command ran (does not verify each
              process actually died).
    """

    term_cmd = "killall -9 rosmaster roslaunch gzserver rosout gzclient robot_state_publisher nodelet"
    subprocess.Popen("xterm -e ' " + term_cmd + "'", shell=True).wait()

    # clear the diagnostic port log
    try:
        if os.path.exists(_PORT_LOG_PATH):
            with open(_PORT_LOG_PATH, 'w') as f:
                try:
                    fcntl.flock(f.fileno(), fcntl.LOCK_EX)
                    f.write('')
                finally:
                    try:
                        fcntl.flock(f.fileno(), fcntl.LOCK_UN)
                    except OSError:
                        pass
    except OSError:
        pass

    rospy.logdebug("Killed all host-wide ROS and Gazebo processes.")

    return True


def kill_all_ros_and_gazebo() -> bool:
    """
    DEPRECATED. Use :func:`kill_all_host_ros_and_gazebo` instead.

    The old name was misleading: it sounds like it cleans up only the
    instances this script spawned, but it actually issues ``killall -9``
    against rosmaster/roslaunch/gzserver/etc. host-wide.
    """
    warnings.warn(
        "kill_all_ros_and_gazebo() is deprecated; use "
        "kill_all_host_ros_and_gazebo() instead. The behaviour is "
        "unchanged but the name now makes the host-wide scope explicit.",
        DeprecationWarning, stacklevel=2,
    )
    return kill_all_host_ros_and_gazebo()


"""
    07. kill_all_host_roslaunch_processes: Kill ALL roslaunch processes on this host.
"""


def kill_all_host_roslaunch_processes() -> bool:
    """
    Kill EVERY ``roslaunch`` process on this host via ``killall -9``.

    Host-scoped: not limited to this script's rosmaster. Renamed from
    ``kill_all_roslaunch_process`` to make the host-wide scope
    explicit. The old name is kept as a DeprecationWarning-emitting
    alias for backwards compatibility.

    Returns:
        bool: True if the kill command ran.
    """

    term_cmd = "killall -9 roslaunch"
    subprocess.Popen("xterm -e ' " + term_cmd + "'", shell=True).wait()
    rospy.logdebug("Killed all host-wide roslaunch processes.")

    return True


def kill_all_roslaunch_process() -> bool:
    """
    DEPRECATED. Use :func:`kill_all_host_roslaunch_processes` instead.
    """
    warnings.warn(
        "kill_all_roslaunch_process() is deprecated; use "
        "kill_all_host_roslaunch_processes() instead. The behaviour is "
        "unchanged but the name now makes the host-wide scope explicit.",
        DeprecationWarning, stacklevel=2,
    )
    return kill_all_host_roslaunch_processes()


"""
    08. kill_all_ros_nodes: Kill all running ROS nodes of a specified or the current ROS master.
"""


def kill_all_ros_nodes(ros_port: Optional[str] = None, gazebo_port: Optional[str] = None) -> bool:
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

    # Remove from diagnostic port log
    if ros_port is not None:
        _remove_from_port_log(ros_port)

    return True


"""
    09. kill_ros_node: Kill a single ROS node of a given or current ROS master.
"""


def kill_ros_node(node_name: str, ros_port: Optional[str] = None, gazebo_port: Optional[str] = None) -> bool:
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

    # Remove from diagnostic port log
    if node_name == "/rosout" and ros_port is not None:
        _remove_from_port_log(ros_port)

    return True


"""
    10. ros_kill_master: To kill a ROS master
"""


def ros_kill_master(ros_port: str) -> bool:
    """
    Function to kill a ROS master.

    Args:
        ros_port (str): The ROS_MASTER_URI port.

    Returns:
        bool: True if the master was killed, False otherwise.
    """

    # term_cmd = "pkill -f 'roscore -p " + ros_port + "'"
    # subprocess.Popen("xterm -e ' " + term_cmd + "'", shell=True).wait()
    # time.sleep(0.5)
    #
    # rospy.logdebug(f"Successfully killed ROS master: {ros_port}!")
    #
    # # Remove the ros master from the ROS_MASTER port list
    # remove_from_rosmaster_list(ros_port)
    #
    # return True

    term_cmd = "pkill -f 'roscore -p " + ros_port + "'"
    result = os.system(term_cmd)
    time.sleep(0.5)

    if result == 0:

        rospy.logdebug(f"Successfully killed ROS master: {ros_port}!")

        # Remove from diagnostic port log
        _remove_from_port_log(ros_port)
        return True

    else:
        rospy.logdebug(f"Failed to kill ROS master: {ros_port}!")
        return False


"""
    11. clean_ros_logs: To clean all the ros logs
"""


def clean_ros_logs() -> bool:
    """
    Function to clean all the ros logs.

    Returns:
        bool: True if all the ros logs were closed.
    """

    # term_cmd = "rosclean purge -y"
    # subprocess.Popen("xterm -e ' " + term_cmd + "'", shell=True).wait()
    # rospy.logdebug("successfully cleaned all the ROS logs")
    # return True

    term_cmd = "rosclean purge -y"

    result = os.system(term_cmd)

    if result == 0:
        rospy.logdebug("Successfully cleaned all the ROS logs")
        return True
    else:
        rospy.logdebug("Failed to clean ROS logs")
        return False


"""
    12. source_workspace: To source a ros workspace
"""


def source_workspace(abs_path: str) -> bool:
    """
    DEPRECATED no-op.

    This function used to launch an xterm that ran
    ``source devel/setup.bash`` plus ``rospack profile``. That approach
    cannot work: a ``source`` in a child shell mutates only the child's
    environment, and the child dies with the subprocess. The calling
    Python process's ``os.environ`` / ``PYTHONPATH`` are unaffected, so
    importing packages from the workspace will not work.

    To use a ROS workspace from a Python process, source it in the
    shell that launches Python (i.e. ``source devel/setup.bash`` in the
    terminal, then run ``python``). If you only need to refresh
    rospack's cache, run ``rospack profile`` directly.

    This function is preserved for backwards compatibility but is now
    a no-op that emits a DeprecationWarning and a rospy logwarn. It
    will be removed in a future release.

    Args:
        abs_path (str): Unused. Kept for signature compatibility.

    Returns:
        bool: Always False.
    """
    import warnings
    msg = (
        "source_workspace() cannot mutate the calling Python process's "
        "environment. Source the ROS workspace in the shell that "
        "launches Python instead. This function is a no-op and will "
        "be removed in a future release."
    )
    warnings.warn(msg, DeprecationWarning, stacklevel=2)
    rospy.logwarn(msg)
    return False


"""
    13. ros_launch_launcher: To execute a roslaunch with args
"""


def ros_launch_launcher(pkg_name: Optional[str] = None,
                        launch_file_name: Optional[str] = None,
                        launch_file_abs_path: Optional[str] = None,
                        args: Optional[List[str]] = None,
                        launch_new_term: bool = True,
                        ros_port: Optional[str] = None,
                        gazebo_port: Optional[str] = None) -> bool:
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
        rospy.logerr("Launch Failed! Requires either the absolute path or the pkg_name and the launch_file_name as input!")
        return False

    if args is not None:
        term_cmd += " " + " ".join(args)

    if launch_new_term:
        term_cmd = "xterm -e ' " + term_cmd + "'"

    # Register the spawned xterm/shell with the managed-process registry
    # so atexit + SIGINT cleanup tears it down when the calling env exits.
    # Without this, the launched roslaunch (e.g. MoveIt bring-up) is
    # orphaned on env.close() and lingers as a zombie xterm + python +
    # move_group stack. start_new_session=True makes the xterm a
    # process-group leader so cleanup can ``killpg`` the whole subtree
    # (xterm → roslaunch → move_group → moveit_python_interface) in one
    # signal — SIGTERM on the xterm alone wouldn't propagate to its
    # grandchildren.
    _launch_proc = subprocess.Popen(term_cmd, shell=True, start_new_session=True)
    time.sleep(5.0)
    register_managed_process(_launch_proc, kind=f"ros_launch:{pkg_name or 'abs_path'}")

    return True


# helper fn for ros_launch_launcher
def construct_roslaunch_command(pkg_name: Optional[str],
                                launch_file_name: Optional[str],
                                launch_file_abs_path: Optional[str]) -> List[str]:
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
            rospy.logerr(f"Launch file {launch_file_name} in {file_path} does not exist!")
            return None

        return f"roslaunch {pkg_name} {launch_file_name}"

    # or roslaunch from a path
    elif launch_file_abs_path is not None:
        if os.path.exists(launch_file_abs_path) is False:
            rospy.logerr(f"Launch file {launch_file_abs_path} does not exist!")
            return None

        return f"roslaunch {launch_file_abs_path}"

    else:
        return None


"""
    14. ros_node_launcher: To launch a ROS node from a package
"""


def ros_node_launcher(pkg_name: str, node_name: str,
                      launch_master: bool = False,
                      launch_new_term: bool = True,
                      name: Optional[str] = None,
                      ns: str = "/",
                      output: str = "log",
                      ros_port: Optional[str] = None,
                      gazebo_port: Optional[str] = None,
                      args: Optional[List[str]] = None) -> Tuple[str, str, bool]:
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
        args (List[str]): List of additional arguments to pass to the rosrun command.

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
        rospy.loginfo("Launching ROS Master")
        if ros_port is not None:
            rs_port, gz_port = launch_roscore(port=int(ros_port))
        else:
            rs_port, gz_port = launch_roscore()

    # check if the rosmaster is running
    try:
        rospy.get_master().getPid()
    except ConnectionRefusedError:
        rospy.loginfo("ROS Master not running!")
        return rs_port, gz_port, False
    else:
        rospy.loginfo("ROS Master is running!")

    term_cmd = construct_rosrun_command(pkg_name, node_name, name=name, ns=ns, output=output)

    if args is not None:
        term_cmd += " " + " ".join(args)

    if launch_new_term:
        term_cmd = f"xterm -e '{term_cmd}'"

    # See ros_launch_launcher: start_new_session makes the xterm wrapper
    # a process-group leader so atexit/SIGINT cleanup can killpg the
    # whole subtree (xterm → rosrun → node) in one signal.
    _node_proc = subprocess.Popen(term_cmd, shell=True, start_new_session=True)
    time.sleep(5.0)
    register_managed_process(_node_proc, kind=f"ros_node:{pkg_name}/{node_name}")

    return rs_port, gz_port, True


# helper fn for ros_node_launcher
def construct_rosrun_command(pkg_name: str, node_name: str,
                             name: Optional[str] = None,
                             ns: str = "/",
                             output: str = "log") -> List[str]:
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
def check_package_exists(pkg_name: str) -> bool:
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


def ros_load_yaml(pkg_name: Optional[str] = None,
                  file_name: Optional[str] = None,
                  file_abs_path: Optional[str] = None,
                  ns: str = '/',
                  ros_port: Optional[str] = None,
                  gazebo_port: Optional[str] = None) -> bool:
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
            rospy.logerr(f"Config file {file_name} in {file_abs_path} does not exist")
            return False

    # If pkg_name and file_name are both None but file_abs_path is not None,
    # check if the YAML file exists at the given absolute path
    elif file_abs_path is not None:
        if os.path.exists(file_abs_path) is False:
            rospy.logerr(f"Config file {file_abs_path} does not exist!")
            return False

    # If none of these conditions are met, return False
    else:
        rospy.logerr("Load Failed! Requires either the absolute path or the pkg_name and the file_name as input!")
        return False

    # Load the parameters from the YAML file and upload them to the ROS Parameter Server under the given namespace
    param_list = rosparam.load_file(file_abs_path)
    for params, namespace in param_list:
        rosparam.upload_params(ns, params)

    return True


"""
    16. load_urdf: Load a URDF into the parameter server or a string containing the processed URDF data
"""


def load_urdf(model_path: Optional[str] = None,
              pkg_name: Optional[str] = None,
              file_name: Optional[str] = None,
              folder: str = "/urdf",
              ns: Optional[str] = None,
              args_xacro: Optional[List[str]] = None,
              param_name: Optional[str] = None,
              ros_port: Optional[str] = None,
              gazebo_port: Optional[str] = None) -> Tuple[bool, Optional[str]]:
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


def change_ros_master_multi_device(remote_ip: str, local_ip: str, remote_ros_port: str = '11311') -> bool:
    """
    Function to set the current ROS Master Environment Variable for multi-device communication.

    Args:
        remote_ip (str): The remote IP address.
        local_ip (str): The local IP address.
        remote_ros_port (str): The remote ROS_MASTER_URI port. Defaults ros port 11311.

    Returns:
        bool: True if ROS Master Environment Variables are set to new values.
    """

    # Temporarily assigning values for the ROS_MASTER_URI environment variable
    os.environ["ROS_MASTER_URI"] = f"http://{remote_ip}:{remote_ros_port}"

    # Temporarily assigning values for the ROS_HOSTNAME environment variable
    os.environ["ROS_HOSTNAME"] = f"{local_ip}"

    rospy.logdebug(f"Changed ROS_MASTER_URI to: http://{remote_ip}:{remote_ros_port}")

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
        _, _, launch_done = ros_node_launcher(pkg_name="robot_state_publisher",
                                              node_name="robot_state_publisher",
                                              launch_new_term=launch_new_term,
                                              ns=ns,
                                              args=[f"publish_frequency:={max_pub_freq}"]
                                              )
    else:
        # we don't need the first two since we are not launching a new roscore
        _, _, launch_done = ros_node_launcher(pkg_name="robot_state_publisher",
                                              node_name="robot_state_publisher",
                                              launch_new_term=launch_new_term,
                                              ns=ns)

    if launch_done:
        rospy.logdebug("Successfully initialised the Robot State Publisher!")
    else:
        rospy.logerr("Robot State Publisher initialisation Failed!")

    return launch_done


def remove_all_from_rosmaster_list() -> bool:
    """
    DEPRECATED. Truncates the diagnostic port log only.

    Port allocation is now handled by ``launch_roscore`` directly via
    ``socket.bind(0)``. Callers do not need to maintain a shared list.
    """
    warnings.warn(
        "remove_all_from_rosmaster_list() is deprecated and now only "
        "truncates a diagnostic log. Port allocation is handled by "
        "socket.bind(0) inside launch_roscore().",
        DeprecationWarning, stacklevel=2,
    )
    try:
        if os.path.exists(_PORT_LOG_PATH):
            with open(_PORT_LOG_PATH, 'w') as f:
                try:
                    fcntl.flock(f.fileno(), fcntl.LOCK_EX)
                    f.write('')
                finally:
                    try:
                        fcntl.flock(f.fileno(), fcntl.LOCK_UN)
                    except OSError:
                        pass
    except OSError as e:
        rospy.logwarn(f"Could not truncate port log {_PORT_LOG_PATH}: {e}")
        return False
    return True
