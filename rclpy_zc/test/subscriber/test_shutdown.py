"""Fresh-child omitted-close regressions with synthetic and opt-in native queues."""

import os
from concurrent.futures import ThreadPoolExecutor, TimeoutError
from pathlib import Path
import subprocess
import sys
from threading import Event, Thread
import uuid
from weakref import WeakSet

import pytest

from system_runtime import preflight, system_tld_requested


def test_shutdown_waits_for_registered_notifier_start_and_rejects_late_start(monkeypatch):
    from rclpy_agnocast.subscription import shutdown

    entered_start, permit_start, entered_shutdown = Event(), Event(), Event()
    stopped = Event()

    class Endpoint:
        def __init__(self):
            self._notifier = Thread(target=lambda: stopped.wait(3), daemon=False)

        def _stop_for_shutdown(self):
            assert self._notifier.ident is not None, "shutdown observed an unstarted notifier"
            stopped.set()
            self._notifier.join(timeout=2)
            assert not self._notifier.is_alive()

    endpoint = Endpoint()
    original_start = endpoint._notifier.start

    def delayed_start():
        entered_start.set()
        assert permit_start.wait(2)
        original_start()

    def stop_notifiers():
        entered_shutdown.set()
        shutdown._stop_notifiers()

    with monkeypatch.context() as patch:
        patch.setattr(shutdown, "_shutting_down", False)
        patch.setattr(shutdown, "_subscriptions", WeakSet())
        patch.setattr(endpoint._notifier, "start", delayed_start)
        try:
            with ThreadPoolExecutor(max_workers=2) as executor:
                try:
                    start = executor.submit(shutdown.start_notifier, endpoint)
                    assert entered_start.wait(1)
                    assert shutdown._lock.locked()
                    stop = executor.submit(stop_notifiers)
                    assert entered_shutdown.wait(1)
                    with pytest.raises(TimeoutError):
                        stop.result(timeout=0.05)
                    assert not shutdown._shutting_down
                    assert not stopped.is_set()
                    permit_start.set()
                    start.result(timeout=2)
                    stop.result(timeout=2)
                    assert stopped.is_set()
                    assert not endpoint._notifier.is_alive()
                    late = Endpoint()
                    with pytest.raises(RuntimeError, match="interpreter shutdown"):
                        shutdown.start_notifier(late)
                    assert late._notifier.ident is None
                finally:
                    permit_start.set()
                    stopped.set()
        finally:
            shutdown.discard_notifier(endpoint)
            if endpoint._notifier.ident is not None:
                endpoint._notifier.join(timeout=2)
            assert not endpoint._notifier.is_alive()


@pytest.mark.parametrize("backend", ["synthetic", "native"])
@pytest.mark.parametrize("scenario", [
    "return", "shutdown-return", "exception", "late-close", "queued-return", "held-return",
])
def test_child_exits_without_notifier_hang_and_preserves_original_failure(
        backend, scenario, tmp_path):
    if backend == "native":
        if not system_tld_requested():
            pytest.skip("set RCLPY_AGNOCAST_RUN_SYSTEM_TLD=1 for actual shutdown TLD")
        environment = preflight().child_environment(20 + uuid.uuid4().int % 180)
    else:
        environment = os.environ.copy()
        environment["ROS_DOMAIN_ID"] = str(20 + uuid.uuid4().int % 180)
    environment["ROS_LOG_DIR"] = str(tmp_path / "ros-log")
    command = [
        sys.executable, str(Path(__file__).with_name("shutdown_process.py")), backend, scenario,
    ]
    child = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                             text=True, env=environment)
    try:
        try:
            output, _ = child.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            child.kill()
            output, _ = child.communicate(timeout=3)
            pytest.fail(f"child failed to exit within 5s: {backend}/{scenario}\n{output}")
        assert "READY" in output, output
        if scenario == "exception":
            assert child.returncode == 1, output
            assert "LookupError: shutdown-original-sentinel" in output
        else:
            assert child.returncode == 0, output
            assert "MAIN_RETURNED" in output
        if scenario == "late-close":
            assert "CLOSED" in output
        if scenario in ("queued-return", "held-return"):
            assert "OWNERS_PRESERVED" in output, output
    finally:
        if child.poll() is None:
            child.kill()
            child.communicate(timeout=3)
