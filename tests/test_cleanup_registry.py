"""
Regression: the managed-process registry tears down the
roscore / Gazebo processes THIS script spawned on Ctrl+C or
interpreter exit, scoped to processes we tracked (not host-wide).

User-facing failure mode addressed: ``rospy.init_node()`` overwrites
the SIGINT handler, so cleanup was never running on Ctrl+C in real
training scripts. Cleanup is wired into ``rospy.on_shutdown`` as
well — these tests verify both registration paths and the cleanup
behaviour.
"""
import signal
import subprocess

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


# ------------------------------------------------------------- registration


class TestRegistration:
    """register_managed_process installs SIGINT + atexit + rospy.on_shutdown."""

    def test_first_call_installs_sigint_handler(self, fresh_ros_common):
        popen = _MockPopen()
        fresh_ros_common.register_managed_process(
            popen, roscore_port="11500", kind="roscore"
        )
        assert signal.getsignal(signal.SIGINT) is fresh_ros_common._sigint_handler

    def test_first_call_registers_with_rospy_on_shutdown(
        self, fresh_ros_common, stub_rospy_ecosystem,
    ):
        # stub_rospy_ecosystem yielded a list; new registrations append to it.
        before = len(stub_rospy_ecosystem)
        popen = _MockPopen()
        fresh_ros_common.register_managed_process(
            popen, roscore_port="11600", kind="roscore"
        )
        assert len(stub_rospy_ecosystem) == before + 1, (
            "register_managed_process did not call rospy.on_shutdown"
        )
        assert stub_rospy_ecosystem[-1] is fresh_ros_common._cleanup_managed_processes

    def test_subsequent_calls_are_idempotent(
        self, fresh_ros_common, stub_rospy_ecosystem,
    ):
        before = len(stub_rospy_ecosystem)
        for _ in range(5):
            fresh_ros_common.register_managed_process(
                _MockPopen(), roscore_port=f"11{_:03d}", kind="roscore"
            )
        # Only ONE new rospy.on_shutdown registration regardless of how
        # many processes are registered.
        assert len(stub_rospy_ecosystem) == before + 1


# -------------------------------------------------------------- cleanup


class TestCleanup:
    """_cleanup_managed_processes tears down what we tracked, scoped."""

    def test_terminates_registered_popen(
        self, fresh_ros_common, fast_sleep, captured_pkills,
    ):
        popen = _MockPopen()
        fresh_ros_common.register_managed_process(
            popen, roscore_port="11700", kind="roscore"
        )
        fresh_ros_common._cleanup_managed_processes()
        assert popen.terminated

    def test_pkill_scoped_to_our_roscore_port(
        self, fresh_ros_common, fast_sleep, captured_pkills,
    ):
        port = "11701"
        fresh_ros_common.register_managed_process(
            _MockPopen(), roscore_port=port, kind="roscore"
        )
        fresh_ros_common._cleanup_managed_processes()
        # Cleanup should have issued exactly: pkill -f "roscore -p <port>"
        # NOT a host-wide killall.
        pkill_cmds = [c for c in captured_pkills if c[:1] == ["pkill"]]
        assert any(f"roscore -p {port}" in c[-1] for c in pkill_cmds), (
            f"No targeted pkill for port {port}; got {captured_pkills}"
        )

    def test_gazebo_pids_sigtermed_then_sigkilled(
        self, fresh_ros_common, fast_sleep, captured_pkills, captured_kills,
    ):
        gazebo_pids = [7777, 8888]
        fresh_ros_common.register_managed_process(
            _MockPopen(), gazebo_pids=gazebo_pids, kind="gazebo"
        )
        fresh_ros_common._cleanup_managed_processes()
        # Both SIGTERM and SIGKILL should have hit both PIDs (phase 1 + 2).
        assert (7777, signal.SIGTERM) in captured_kills
        assert (8888, signal.SIGTERM) in captured_kills
        assert (7777, signal.SIGKILL) in captured_kills
        assert (8888, signal.SIGKILL) in captured_kills

    def test_idempotent_second_call_is_noop(
        self, fresh_ros_common, fast_sleep, captured_pkills,
    ):
        fresh_ros_common.register_managed_process(
            _MockPopen(), roscore_port="11702", kind="roscore"
        )
        fresh_ros_common._cleanup_managed_processes()
        captured_pkills.clear()
        fresh_ros_common._cleanup_managed_processes()
        assert captured_pkills == [], "Second cleanup ran additional pkills"

    def test_after_cleanup_sigint_is_reset_to_default(
        self, fresh_ros_common, fast_sleep, captured_pkills,
    ):
        # After cleanup, SIGINT goes back to SIG_DFL so a subsequent
        # Ctrl+C terminates the script immediately even if it's stuck
        # in a non-responsive loop (e.g. SB3.learn()).
        fresh_ros_common.register_managed_process(
            _MockPopen(), roscore_port="11703", kind="roscore"
        )
        fresh_ros_common._cleanup_managed_processes()
        assert signal.getsignal(signal.SIGINT) == signal.SIG_DFL


# --------------------------------------------------------- rospy shutdown


class TestRospyShutdownPath:
    """rospy.on_shutdown invoking our callback must tear down successfully
    even when rospy.init_node has replaced our SIGINT handler (the
    real-world failure case this path was added for)."""

    def test_cleanup_via_rospy_shutdown_after_sigint_was_stolen(
        self, fresh_ros_common, stub_rospy_ecosystem,
        fast_sleep, captured_pkills,
    ):
        popen = _MockPopen()
        fresh_ros_common.register_managed_process(
            popen, roscore_port="11704", kind="roscore"
        )
        # Simulate rospy.init_node installing its own SIGINT handler.
        def _fake_rospy_sigint(signum, frame):
            pass
        signal.signal(signal.SIGINT, _fake_rospy_sigint)
        # Now simulate rospy invoking its registered shutdown hooks.
        for cb in stub_rospy_ecosystem:
            cb()
        assert popen.terminated, (
            "rospy.on_shutdown path did not terminate the registered Popen"
        )
        pkill_cmds = [c for c in captured_pkills if c[:1] == ["pkill"]]
        assert any("roscore -p 11704" in c[-1] for c in pkill_cmds)
