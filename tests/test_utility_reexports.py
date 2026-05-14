"""
Regression: multiros.utils.{ros_markers, ros_kinematics,
ros_controllers} must re-export the EXACT same symbols as
uniros.utils.{...}. Identity equality, not just same name.
"""
from multiros.utils import ros_markers as mr_markers
from multiros.utils import ros_controllers as mr_ctl
from uniros.utils import ros_markers as u_markers
from uniros.utils import ros_controllers as u_ctl


class TestRosMarkersReexport:

    def test_RosMarker_identity(self):
        assert mr_markers.RosMarker is u_markers.RosMarker

    def test_RosMarkerArray_identity(self):
        assert mr_markers.RosMarkerArray is u_markers.RosMarkerArray


class TestRosControllersReexport:

    EXPECTED = [
        "load_ros_controller", "load_controller_list", "list_loaded_controllers",
        "unload_ros_controller", "unload_controller_list", "switch_controllers",
        "start_controllers", "stop_controllers", "reset_controllers",
        "spawn_controllers", "unspawn_controllers",
    ]

    def test_each_helper_is_identical(self):
        for name in self.EXPECTED:
            mr_obj = getattr(mr_ctl, name)
            u_obj = getattr(u_ctl, name)
            assert mr_obj is u_obj, f"multiros.utils.ros_controllers.{name} is not the canonical object"


class TestRosKinematicsReexport:
    """ros_kinematics is checked textually because PyKDL is a heavy native dep."""

    def test_reexport_text_present(self):
        import pathlib
        path = pathlib.Path(__file__).parent.parent / "src" / "multiros" / "utils" / "ros_kinematics.py"
        text = path.read_text()
        assert "from uniros.utils.ros_kinematics import *" in text
        assert "Kinematics_pyrobot" in text
        assert "Kinematics_pykdl" in text
