"""
Round 6.1 regression: launch_roscore() uses socket.bind(0) to pick
free ports atomically rather than guessing via a racy /tmp file.

These tests exercise the allocator without launching real roscores
(subprocess.Popen is monkeypatched).
"""
import socket
import threading
import warnings

import pytest

from multiros.utils import ros_common


# ----------------------------------------------------------------- helpers


class _NoopPopen:
    """Stand-in for subprocess.Popen — captures the command but does nothing."""
    instances = []
    def __init__(self, cmd, *a, **k):
        self.cmd = cmd
        type(self).instances.append(self)
        self._alive = True
    def poll(self):
        return None if self._alive else 0
    def terminate(self):
        self._alive = False
    def kill(self):
        self._alive = False
    def wait(self):
        return 0


@pytest.fixture
def patched_launch(monkeypatch, tmp_path):
    """Patch subprocess.Popen, time.sleep, change_ros_gazebo_master,
    and redirect the port-log to a tmp file."""
    import subprocess as sp
    import time as _time

    _NoopPopen.instances.clear()
    monkeypatch.setattr(sp, "Popen", _NoopPopen)
    monkeypatch.setattr(_time, "sleep", lambda *a, **k: None)
    monkeypatch.setattr(ros_common, "change_ros_gazebo_master", lambda *a, **k: True)
    monkeypatch.setattr(ros_common, "_PORT_LOG_PATH", str(tmp_path / "ports.log"))
    yield


# -------------------------------------------------------------- low-level


class TestKernelAllocator:
    """The pure kernel-allocator helpers — no subprocess needed."""

    def test_reserve_free_port_returns_open_port(self):
        p = ros_common._reserve_free_port()
        # Sanity: it's a valid TCP port number and rebinding succeeds.
        assert 1024 <= p < 65536
        # We can re-bind on the same port now (SO_REUSEADDR + the fact
        # that the previous bind was released).
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind(('127.0.0.1', p))
        finally:
            sock.close()

    def test_port_is_free_for_high_port(self):
        # The kernel will give us a free one; that one must register as free.
        p = ros_common._reserve_free_port()
        assert ros_common._port_is_free(p)

    def test_port_is_free_returns_false_for_squatted_port(self):
        squat = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        squat.bind(('127.0.0.1', 0))
        squat.listen(1)
        squatted = squat.getsockname()[1]
        try:
            assert ros_common._port_is_free(squatted) is False
        finally:
            squat.close()


# --------------------------------------------------------- launch_roscore


class TestLaunchRoscore:
    def test_returns_distinct_ros_and_gazebo_ports(self, patched_launch):
        ros_p, gaz_p = ros_common.launch_roscore(set_new_master_vars=False)
        assert ros_p != gaz_p
        assert int(ros_p) > 0 and int(gaz_p) > 0

    def test_serial_calls_get_unique_ports(self, patched_launch):
        # 10 sequential launches → 20 unique ports (no duplicates between
        # ros and gazebo across the whole series).
        all_ports = []
        for _ in range(10):
            ros_p, gaz_p = ros_common.launch_roscore(set_new_master_vars=False)
            all_ports.append(ros_p)
            all_ports.append(gaz_p)
        assert len(set(all_ports)) == len(all_ports), (
            f"Duplicates in {all_ports}"
        )

    def test_parallel_calls_get_unique_ports(self, patched_launch):
        # 20 concurrent launches from threads. The kernel-allocator must
        # serialize correctly so no two callers receive the same port.
        results = []
        lock = threading.Lock()
        def _launch():
            r, g = ros_common.launch_roscore(set_new_master_vars=False)
            with lock:
                results.append((r, g))
        threads = [threading.Thread(target=_launch) for _ in range(20)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        all_ports = [p for tup in results for p in tup]
        assert len(set(all_ports)) == len(all_ports), (
            f"Concurrent allocator handed out duplicates: {all_ports}"
        )

    def test_requested_port_falls_back_when_taken(self, patched_launch):
        squat = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        squat.bind(('127.0.0.1', 0))
        squat.listen(1)
        squatted = squat.getsockname()[1]
        try:
            ros_p, _ = ros_common.launch_roscore(
                port=squatted, set_new_master_vars=False
            )
            # The allocator should have refused the squatted port and
            # chosen a different one from the kernel.
            assert int(ros_p) != squatted
        finally:
            squat.close()

    def test_runs_roscore_command_via_subprocess(self, patched_launch):
        ros_p, _ = ros_common.launch_roscore(set_new_master_vars=False)
        # The xterm wrapper command should contain the chosen port.
        commands = [inst.cmd for inst in _NoopPopen.instances]
        assert any(f"roscore -p {ros_p}" in c for c in commands), (
            f"No Popen invocation included 'roscore -p {ros_p}'; got {commands}"
        )


# --------------------------------------------------------- deprecation


class TestLegacyHelperDeprecation:
    """Round 6.1: the four legacy port-list helpers emit DeprecationWarning."""

    def test_get_all_the_ros_masters_emits_warning(self, patched_launch):
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always")
            ros_common.get_all_the_ros_masters()
        assert any(issubclass(w.category, DeprecationWarning) for w in caught)

    def test_add_to_rosmaster_list_emits_warning(self, patched_launch):
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always")
            ros_common.add_to_rosmaster_list("11500")
        assert any(issubclass(w.category, DeprecationWarning) for w in caught)

    def test_remove_from_rosmaster_list_emits_warning(self, patched_launch):
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always")
            ros_common.remove_from_rosmaster_list("11500")
        assert any(issubclass(w.category, DeprecationWarning) for w in caught)

    def test_remove_all_from_rosmaster_list_emits_warning(self, patched_launch):
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always")
            ros_common.remove_all_from_rosmaster_list()
        assert any(issubclass(w.category, DeprecationWarning) for w in caught)
