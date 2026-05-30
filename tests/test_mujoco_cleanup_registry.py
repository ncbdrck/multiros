"""
Regression: the managed-process registry tears down the MuJoCo server
processes THIS script spawned, the same way it handles roscore / Gazebo
processes. The MuJoCo backend registers its server PIDs under the
``mujoco_pids`` selector; cleanup must SIGTERM then SIGKILL them.
"""
import signal

import pytest


class _MockPopen:
    """Stand-in for the Popen we register — captures .terminate() / .kill()."""
    def __init__(self, *a, **k):
        self.terminated = False
        self.killed = False
        self._alive = True
        self.pid = 99999
    def poll(self):
        return None if self._alive else 0
    def terminate(self):
        self.terminated = True
        self._alive = False
    def kill(self):
        self.killed = True
        self._alive = False


@pytest.fixture
def captured_pkills(monkeypatch):
    """Intercept subprocess.run so cleanup doesn't actually pkill."""
    import subprocess
    captured = []
    def _capture(cmd, *a, **k):
        captured.append(list(cmd))
        return subprocess.CompletedProcess(args=cmd, returncode=0, stdout=b"", stderr=b"")
    monkeypatch.setattr(subprocess, "run", _capture)
    return captured


@pytest.fixture
def captured_kills(monkeypatch):
    """Intercept os.kill so cleanup doesn't actually signal real PIDs."""
    import os
    captured = []
    monkeypatch.setattr(os, "kill", lambda pid, sig: captured.append((pid, sig)))
    return captured


@pytest.fixture
def fast_sleep(monkeypatch):
    """Speed up cleanup's 0.5s wait between SIGTERM and SIGKILL."""
    import time
    monkeypatch.setattr(time, "sleep", lambda *a, **k: None)


class TestMujocoCleanup:
    """_cleanup_managed_processes handles mujoco_pids like gazebo_pids."""

    def test_mujoco_pids_sigtermed_then_sigkilled(
        self, fresh_ros_common, fast_sleep, captured_pkills, captured_kills,
    ):
        mujoco_pids = [4444, 5555]
        fresh_ros_common.register_managed_process(
            _MockPopen(), mujoco_pids=mujoco_pids, kind="mujoco"
        )
        fresh_ros_common._cleanup_managed_processes()
        # Both SIGTERM and SIGKILL should have hit both PIDs (phase 1 + 2).
        assert (4444, signal.SIGTERM) in captured_kills
        assert (5555, signal.SIGTERM) in captured_kills
        assert (4444, signal.SIGKILL) in captured_kills
        assert (5555, signal.SIGKILL) in captured_kills

    def test_terminates_registered_popen(
        self, fresh_ros_common, fast_sleep, captured_pkills,
    ):
        popen = _MockPopen()
        fresh_ros_common.register_managed_process(
            popen, mujoco_pids=[4446], kind="mujoco"
        )
        fresh_ros_common._cleanup_managed_processes()
        assert popen.terminated

    def test_mixed_gazebo_and_mujoco_both_cleaned(
        self, fresh_ros_common, fast_sleep, captured_pkills, captured_kills,
    ):
        # A process registered with both selectors (or two registrations) must
        # have every tracked PID cleaned up regardless of backend label.
        fresh_ros_common.register_managed_process(
            _MockPopen(), gazebo_pids=[7777], kind="gazebo"
        )
        fresh_ros_common.register_managed_process(
            _MockPopen(), mujoco_pids=[4444], kind="mujoco"
        )
        fresh_ros_common._cleanup_managed_processes()
        assert (7777, signal.SIGTERM) in captured_kills
        assert (4444, signal.SIGTERM) in captured_kills
