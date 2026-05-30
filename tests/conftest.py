"""
Shared pytest fixtures and stubs for the multiros test suite.

Important: the rospy / message-package stubs are installed at this
module's LOAD time, NOT inside an autouse fixture. The reason is
test files import ``multiros.utils.ros_common`` at top level, which
in turn does ``import rospy``. If the real rospy (from a system ROS
install on the developer's machine, or a CI installation) is on the
path, that real import wins and our autouse fixture would never get
to install its stub. Doing it at conftest load time guarantees the
stubs are present BEFORE any test file is imported.
"""
import sys
import types
import signal
import pytest


_CAPTURED_SHUTDOWN_CALLBACKS: list = []


def _force_stub(name, attrs=None):
    """
    Install ``name`` as a stub module, OVERWRITING any prior import.

    This is the difference vs. an autouse fixture: we don't care
    whether the real package was already imported; we want our
    minimal stub for tests to use.
    """
    sys.modules[name] = types.SimpleNamespace(**(attrs or {}))


# --- rospy + catkin stubs -------------------------------------------------

_force_stub("rospy", {
    "loginfo":   lambda *a, **k: None,
    "logwarn":   lambda *a, **k: None,
    "logdebug":  lambda *a, **k: None,
    "logerr":    lambda *a, **k: None,
    "logfatal":  lambda *a, **k: None,
    "ROSException": Exception,
    "ROSInterruptException": Exception,
    "on_shutdown": lambda cb: _CAPTURED_SHUTDOWN_CALLBACKS.append(cb),
    "is_shutdown": lambda: False,
    "init_node":  lambda *a, **k: None,
    "wait_for_service": lambda *a, **k: None,
    "Duration":   lambda *a, **k: types.SimpleNamespace(),
    "Time":       types.SimpleNamespace(now=lambda: types.SimpleNamespace()),
    "Rate":       lambda *a, **k: types.SimpleNamespace(sleep=lambda: None),
    "Publisher":  lambda *a, **k: types.SimpleNamespace(publish=lambda *a2, **k2: None),
    "ServiceProxy": lambda *a, **k: (lambda *a2, **k2: None),
})
_force_stub("rosparam", {"upload_params": lambda *a, **k: None})
_force_stub("rospkg", {
    "RosPack": lambda: types.SimpleNamespace(get_path=lambda *a: "/tmp"),
    "common":  types.SimpleNamespace(ResourceNotFound=Exception),
})
_force_stub("xacro", {"process_file": lambda *a, **k: None})


# --- ROS message-package stubs -------------------------------------------

class _DummyMsg:
    def __init__(self, *a, **k):
        for k_, v_ in k.items():
            setattr(self, k_, v_)


class _MarkerMsg:
    SPHERE = 2; CUBE = 1; CYLINDER = 3; LINE_LIST = 5
    ARROW = 0; ADD = 0; DELETE = 2; MODIFY = 0; DELETEALL = 3
    def __init__(self, *a, **k): pass


class _MarkerArrayMsg:
    def __init__(self, *a, **k):
        self.markers = []


_force_stub("visualization_msgs", {})
_force_stub("visualization_msgs.msg", {"Marker": _MarkerMsg, "MarkerArray": _MarkerArrayMsg})
_force_stub("geometry_msgs", {})
_force_stub("geometry_msgs.msg", {
    "Point": _DummyMsg, "Pose": _DummyMsg, "Quaternion": _DummyMsg,
    "Vector3": _DummyMsg, "PoseStamped": _DummyMsg, "Twist": _DummyMsg,
    "TwistStamped": _DummyMsg,
    "Transform": _DummyMsg, "TransformStamped": _DummyMsg,
    "Vector3Stamped": _DummyMsg, "QuaternionStamped": _DummyMsg,
    "Wrench": _DummyMsg, "WrenchStamped": _DummyMsg, "PointStamped": _DummyMsg,
})
_force_stub("std_msgs", {})
_force_stub("std_msgs.msg", {
    "ColorRGBA": _DummyMsg, "Header": _DummyMsg, "Float64": _DummyMsg,
})
_force_stub("sensor_msgs", {})
_force_stub("sensor_msgs.msg", {"JointState": _DummyMsg, "Image": _DummyMsg})
_force_stub("controller_manager_msgs", {})
_force_stub("controller_manager_msgs.srv", {
    "LoadController": _DummyMsg, "UnloadController": _DummyMsg,
    "ListControllers": _DummyMsg, "SwitchController": _DummyMsg,
    "SwitchControllerRequest": _DummyMsg,
})
_force_stub("tf", {})
_force_stub("tf.transformations", {
    "quaternion_from_euler": lambda *a, **k: (0.0, 0.0, 0.0, 1.0),
    "euler_from_quaternion": lambda *a, **k: (0.0, 0.0, 0.0),
    "euler_from_matrix":     lambda *a, **k: (0.0, 0.0, 0.0),
    "quaternion_matrix":     lambda *a, **k: [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]],
    "translation_matrix":    lambda *a, **k: [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]],
    "concatenate_matrices":  lambda *a, **k: [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]],
})

# rospy.service is a submodule on the real rospy package; multiros's
# gazebo_models does ``from rospy.service import ServiceException``.
# Register it as a separate module AND hang it off the rospy stub so
# both ``import rospy.service`` and ``rospy.service.X`` work.
_force_stub("rospy.service", {"ServiceException": Exception})
sys.modules["rospy"].service = sys.modules["rospy.service"]

# Gazebo message packages used by multiros.utils.{gazebo_models, gazebo_physics}
_force_stub("gazebo_msgs", {})
_force_stub("gazebo_msgs.srv", {
    "DeleteModel": _DummyMsg, "SpawnModel": _DummyMsg,
    "GetWorldProperties": _DummyMsg, "SpawnModelRequest": _DummyMsg,
    "GetModelState": _DummyMsg, "SetModelState": _DummyMsg,
    "GetPhysicsProperties": _DummyMsg, "SetPhysicsProperties": _DummyMsg,
    "SetPhysicsPropertiesRequest": _DummyMsg,
})
_force_stub("gazebo_msgs.msg", {"ModelState": _DummyMsg, "ODEPhysics": _DummyMsg})
_force_stub("std_srvs", {})
_force_stub("std_srvs.srv", {"Empty": _DummyMsg})

# MuJoCo message packages used by multiros.utils.{mujoco_core, mujoco_physics,
# mujoco_models}. mujoco_ros_pkgs is source-built and absent on CI / Gazebo-only
# installs; the modules guard the import, but stubbing the types lets the tests
# exercise the MuJoCo code paths.
_force_stub("mujoco_ros_msgs", {})
_force_stub("mujoco_ros_msgs.srv", {
    "SetPause": _DummyMsg, "Reload": _DummyMsg, "SetFloat": _DummyMsg,
    "SetGravity": _DummyMsg, "GetGravity": _DummyMsg, "GetSimInfo": _DummyMsg,
    "SetBodyState": _DummyMsg, "GetBodyState": _DummyMsg,
})
_force_stub("mujoco_ros_msgs.msg", {
    "StepAction": _DummyMsg, "StepGoal": _DummyMsg, "BodyState": _DummyMsg,
})
_force_stub("actionlib", {
    "SimpleActionClient": lambda *a, **k: types.SimpleNamespace(
        wait_for_server=lambda *a2, **k2: True,
        send_goal=lambda *a2, **k2: None,
        wait_for_result=lambda *a2, **k2: None,
        get_result=lambda *a2, **k2: None,
    ),
})
_force_stub("dynamic_reconfigure", {})
_force_stub("dynamic_reconfigure.client", {
    "Client": lambda *a, **k: types.SimpleNamespace(
        update_configuration=lambda *a2, **k2: None,
        get_configuration=lambda *a2, **k2: {},
    ),
})

# moveit_commander is the real heavy MoveIt Python wrapper. multiros's
# moveit_multiros.py imports it at module load. We stub it because the
# test surface doesn't exercise MoveIt, and the real module's import
# chain pulls in conversions / planning_scene_interface / etc., each
# with their own deep ROS dependencies.
class _MoveItCommander:
    """No-op MoveIt commander shell — instantiation returns a dummy object."""
    def __init__(self, *a, **k): pass
    def __getattr__(self, name): return lambda *a, **k: None
    @staticmethod
    def roscpp_initialize(*a, **k): pass


_force_stub("moveit_commander", {
    "RobotCommander": _MoveItCommander,
    "PlanningSceneInterface": _MoveItCommander,
    "MoveGroupCommander": _MoveItCommander,
    "roscpp_initialize": lambda *a, **k: None,
    "roscpp_shutdown": lambda *a, **k: None,
})
_force_stub("moveit_msgs", {})
_force_stub("moveit_msgs.msg", {
    "Constraints": _DummyMsg, "OrientationConstraint": _DummyMsg,
    "PositionConstraint": _DummyMsg, "JointConstraint": _DummyMsg,
    "BoundingVolume": _DummyMsg, "Grasp": _DummyMsg,
    "AllowedCollisionMatrix": _DummyMsg, "AllowedCollisionEntry": _DummyMsg,
})
_force_stub("shape_msgs", {})
_force_stub("shape_msgs.msg", {"SolidPrimitive": _DummyMsg, "Mesh": _DummyMsg})
_force_stub("trajectory_msgs", {})
_force_stub("trajectory_msgs.msg", {
    "JointTrajectory": _DummyMsg, "JointTrajectoryPoint": _DummyMsg,
})

# PyKDL / pykdl_utils / urdf_parser_py / kdl_parser_py / hrl_geom: heavy
# native-code deps used by uniros.utils.ros_kinematics. Tests stub the
# whole chain because the test surface doesn't exercise kinematics.
_force_stub("PyKDL", {
    "Chain": type("Chain", (), {}),
    "JntArray": lambda *a: None,
    "Frame": lambda *a, **k: None,
    "Vector": lambda *a, **k: None,
    "Rotation": types.SimpleNamespace(Quaternion=lambda *a: None),
    "ChainFkSolverPos_recursive": lambda *a: None,
    "ChainIkSolverPos_LMA": lambda *a: None,
    "ChainIkSolverPos_NR_JL": lambda *a: None,
    "ChainIkSolverVel_pinv": lambda *a: None,
    "ChainJntToJacSolver": lambda *a: None,
    "Jacobian": lambda *a: None,
    "ChainDynParam": lambda *a, **k: None,
    "JntSpaceInertiaMatrix": lambda *a: None,
})
_force_stub("kdl_parser_py", {})
_force_stub("kdl_parser_py.urdf", {
    "treeFromString": lambda *a, **k: (False, None),
    "treeFromParam":  lambda *a, **k: (False, None),
    "treeFromFile":   lambda *a, **k: (False, None),
})
_force_stub("urdf_parser_py", {})
_force_stub("urdf_parser_py.urdf", {
    "URDF": type("URDF", (), {"from_xml_string": staticmethod(lambda x: None),
                              "from_parameter_server": staticmethod(lambda *a, **k: None)}),
    "Robot": type("Robot", (), {}),
})
_force_stub("pykdl_utils", {})
_force_stub("pykdl_utils.kdl_parser", {
    "kdl_tree_from_urdf_model": lambda *a, **k: None,
})
_force_stub("pykdl_utils.kdl_kinematics", {
    "KDLKinematics": type("KDLKinematics", (), {}),
})
_force_stub("hrl_geom", {})
_force_stub("hrl_geom.pose_converter", {
    "PoseConv": type("PoseConv", (), {}),
})
# trac_ik_python is installed via apt (ros-noetic-trac-ik) on the
# developer's machine but isn't a pip package, so it's missing on
# GitHub Actions runners. uniros.utils.ros_kinematics imports it at
# module load with ``from trac_ik_python import trac_ik`` — note this
# is "from package import submodule", which requires the submodule to
# be accessible as an attribute on the parent package, not just present
# in sys.modules.
_force_stub("trac_ik_python", {})
_force_stub("trac_ik_python.trac_ik", {
    "IK": type("IK", (), {}),
})
sys.modules["trac_ik_python"].trac_ik = sys.modules["trac_ik_python.trac_ik"]


# --- fixtures -------------------------------------------------------------

@pytest.fixture
def stub_rospy_ecosystem():
    """
    Yield the captured list of rospy.on_shutdown callbacks for a single
    test. Cleared on entry and exit so tests are independent.
    """
    _CAPTURED_SHUTDOWN_CALLBACKS.clear()
    yield _CAPTURED_SHUTDOWN_CALLBACKS
    _CAPTURED_SHUTDOWN_CALLBACKS.clear()


@pytest.fixture
def fresh_ros_common():
    """
    Provide a fresh-state copy of multiros.utils.ros_common with the
    cleanup-registry state cleared and SIGINT reset to default before
    AND after each test.

    Order matters: restore SIGINT FIRST so the previous test's
    _sigint_handler isn't sitting installed when we clear
    _handlers_installed (otherwise the next register_managed_process
    call would record _sigint_handler as the "previous" handler and
    create infinite recursion on a real SIGINT).
    """
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    from multiros.utils import ros_common
    ros_common._managed_processes.clear()
    ros_common._cleanup_done = False
    ros_common._handlers_installed = False
    ros_common._prev_sigint_handler = None
    yield ros_common
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    ros_common._managed_processes.clear()
    ros_common._cleanup_done = False
    ros_common._handlers_installed = False
    ros_common._prev_sigint_handler = None
