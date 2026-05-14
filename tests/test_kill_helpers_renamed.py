"""
Round 7 regression: kill_all_ros_and_gazebo + kill_all_roslaunch_process
were renamed to kill_all_HOST_ros_and_gazebo / kill_all_HOST_roslaunch_processes
to make their host-wide ``killall -9`` scope explicit. The old names
remain as DeprecationWarning-emitting aliases.
"""
import warnings

import pytest

from multiros.utils import ros_common


class TestNewNamesExist:
    def test_kill_all_host_ros_and_gazebo_callable(self):
        assert callable(ros_common.kill_all_host_ros_and_gazebo)

    def test_kill_all_host_roslaunch_processes_callable(self):
        assert callable(ros_common.kill_all_host_roslaunch_processes)


class TestOldNamesStillWork:
    """Backwards compatibility: the old names still resolve and call into
    the new implementations. Anyone with old code keeps working."""

    def test_old_kill_all_ros_and_gazebo_emits_warning(self, monkeypatch):
        # Patch the underlying implementation so the test doesn't
        # actually try to killall on the test machine.
        called = {"flag": False}
        def _fake_impl():
            called["flag"] = True
            return True
        monkeypatch.setattr(
            ros_common, "kill_all_host_ros_and_gazebo", _fake_impl
        )
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always")
            ros_common.kill_all_ros_and_gazebo()
        assert called["flag"], "Deprecated alias did not call the new impl"
        assert any(issubclass(w.category, DeprecationWarning) for w in caught)

    def test_old_kill_all_roslaunch_process_emits_warning(self, monkeypatch):
        called = {"flag": False}
        def _fake_impl():
            called["flag"] = True
            return True
        monkeypatch.setattr(
            ros_common, "kill_all_host_roslaunch_processes", _fake_impl
        )
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always")
            ros_common.kill_all_roslaunch_process()
        assert called["flag"]
        assert any(issubclass(w.category, DeprecationWarning) for w in caught)
