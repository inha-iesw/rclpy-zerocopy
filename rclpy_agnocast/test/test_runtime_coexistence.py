"""Opt-in subprocess TLD for rclpy/Agnocast logging and signal coexistence."""

from __future__ import annotations

import json
import os
from pathlib import Path
import queue
import signal
import subprocess
import sys
import threading
import time
from typing import Dict, List, Optional

import pytest

from system_runtime import PreflightError
from system_runtime import preflight
from system_runtime import SystemConfig


EVENT_PREFIX = "TLD_EVENT "
SKIP_EXIT = 77
OPT_IN = "RCLPY_AGNOCAST_RUN_SYSTEM_TLD"
FORBIDDEN_LOG_TEXT = (
    "Failed to configure logging",
    "Failed to finalize logging",
    "logging system is not initialized",
    "terminate called",
)

pytestmark = pytest.mark.skipif(
    os.environ.get(OPT_IN, "").lower() not in {"1", "true", "yes"},
    reason=f"set {OPT_IN}=1 to run the Agnocast backend system TLD",
)


def parse_event(line: str) -> Optional[Dict[str, object]]:
    """Parse one child event while ignoring ROS log output."""
    if not line.startswith(EVENT_PREFIX):
        return None
    value = json.loads(line[len(EVENT_PREFIX):])
    if not isinstance(value, dict) or not isinstance(value.get("name"), str):
        raise AssertionError(f"malformed child event: {line}")
    return value


class Child:
    """Own one fresh Python interpreter and its diagnostic transcript."""

    def __init__(self, scenario: str, environment: Dict[str, str]) -> None:
        helper = Path(__file__).with_name("runtime_coexistence_child.py")
        self.process = subprocess.Popen(
            [sys.executable, str(helper), scenario],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            env=environment,
        )
        self.lines: "queue.Queue[Optional[str]]" = queue.Queue()
        self.transcript: List[str] = []
        self.events: List[Dict[str, object]] = []
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()

    def _read(self) -> None:
        assert self.process.stdout is not None
        for raw_line in self.process.stdout:
            line = raw_line.rstrip("\n")
            self.transcript.append(line)
            event = parse_event(line)
            if event is not None:
                self.events.append(event)
            self.lines.put(line)
        self.lines.put(None)

    def wait_event(self, name: str, timeout: float = 15.0) -> Dict[str, object]:
        """Wait for a named event without losing non-event diagnostics."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                line = self.lines.get(timeout=deadline - time.monotonic())
            except queue.Empty:
                break
            if line is None:
                self.process.wait(timeout=2.0)
                self.reader.join(timeout=2.0)
                self.raise_explicit_skip_if_present()
                break
            event = parse_event(line)
            if event is not None and event["name"] == name:
                return event
        pytest.fail(self.failure_text(f"did not emit {name!r}"))

    def raise_explicit_skip_if_present(self) -> None:
        """Convert only exit 77 plus one reason-bearing child event into a skip."""
        if self.process.returncode != SKIP_EXIT:
            return
        skips = [event for event in self.events if event["name"] == "skip"]
        if len(skips) != 1 or not skips[0].get("reason"):
            pytest.fail(self.failure_text("exit 77 lacked one explicit skip reason"))
        pytest.skip(f"Agnocast backend preflight unavailable: {skips[0]['reason']}")

    def finish(self, timeout: float = 30.0) -> Dict[str, object]:
        """Require success, or convert only an explicit exit-77 event into pytest skip."""
        try:
            return_code = self.process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait(timeout=5.0)
            pytest.fail(self.failure_text("timed out"))
        self.reader.join(timeout=2.0)
        if return_code == SKIP_EXIT:
            self.raise_explicit_skip_if_present()
        if return_code != 0:
            pytest.fail(self.failure_text(f"exit={return_code}"))
        results = [event for event in self.events if event["name"] == "result"]
        if len(results) != 1:
            pytest.fail(self.failure_text("did not emit exactly one result"))
        return results[0]

    def cleanup(self) -> None:
        """Kill only a still-running child owned by this test."""
        if self.process.poll() is None:
            self.process.kill()
            self.process.wait(timeout=5.0)

    def failure_text(self, reason: str) -> str:
        transcript = "\n".join(self.transcript[-40:])
        return f"{reason}; command={self.process.args!r}\n{transcript}"


def configured_system() -> SystemConfig:
    """Resolve the artifact set only after the shared exact-artifact preflight passes."""
    try:
        return preflight()
    except PreflightError as error:
        pytest.skip(f"Agnocast backend preflight unavailable: {error}")


def child_environment(
    system: SystemConfig, log_directory: Path, ordinal: int
) -> Dict[str, str]:
    """Build a deterministic environment applied by exec before Python startup."""
    hook = system.heaphook
    environment = os.environ.copy()
    environment["LD_PRELOAD"] = str(hook)
    prior_library_path = environment.get("LD_LIBRARY_PATH", "")
    environment["LD_LIBRARY_PATH"] = str(hook.parent) + (
        os.pathsep + prior_library_path if prior_library_path else ""
    )
    environment["AGNOCAST_BRIDGE_MODE"] = "off"
    environment["AGNOCAST_NO_DISCOVERY_AGENT"] = "1"
    environment["AGNOCAST_DEVICE_PATH"] = str(system.device)
    environment["ROS_DOMAIN_ID"] = str(20 + ((os.getpid() + ordinal) % 180))
    environment["ROS_LOG_DIR"] = str(log_directory)
    environment["RCUTILS_COLORIZED_OUTPUT"] = "0"
    environment["PYTHONUNBUFFERED"] = "1"
    return environment


def start_child(scenario: str, tmp_path: Path, ordinal: int) -> Child:
    """Start one scenario in a directly exec'd, preloaded Python child."""
    log_directory = tmp_path / "ros_logs"
    log_directory.mkdir()
    return Child(
        scenario,
        child_environment(configured_system(), log_directory, ordinal),
    )


def assert_clean_transcript(child: Child) -> None:
    """Reject global logging lifecycle diagnostics even when the child returned zero."""
    transcript = "\n".join(child.transcript)
    for forbidden in FORBIDDEN_LOG_TEXT:
        assert forbidden not in transcript, child.failure_text(f"observed {forbidden!r}")


def test_rclpy_logging_survives_adapter_close(tmp_path: Path) -> None:
    child = start_child("normal_logging", tmp_path, 0)
    try:
        ready = child.wait_event("ready")
        result = child.finish()
        assert result["rclpy_shutdown"] is True
        assert result["rosout_markers"] == 3
        assert_clean_transcript(child)

        pid = int(ready["pid"])
        markers = [
            f"RCLPY_AGNOCAST_TLD_BEFORE_{pid}",
            f"RCLPY_AGNOCAST_TLD_DURING_{pid}",
            f"RCLPY_AGNOCAST_TLD_AFTER_{pid}",
        ]
        non_events = "\n".join(
            line for line in child.transcript if not line.startswith(EVENT_PREFIX)
        )
        log_text = "\n".join(
            path.read_text(encoding="utf-8", errors="replace")
            for path in (tmp_path / "ros_logs").rglob("*")
            if path.is_file()
        )
        for marker in markers:
            assert marker in non_events, child.failure_text(f"console lost {marker}")
            assert marker in log_text, child.failure_text(f"file logging lost {marker}")
    finally:
        child.cleanup()


@pytest.mark.parametrize("signum", [signal.SIGINT, signal.SIGTERM])
def test_rclpy_signal_is_chained_while_adapter_is_open(
    tmp_path: Path, signum: signal.Signals
) -> None:
    child = start_child("signal_while_open", tmp_path, int(signum))
    try:
        child.wait_event("ready")
        child.process.send_signal(signum)
        result = child.finish()
        assert result["phase"] == "while_open"
        assert result["rclpy_signal_observed"] is True
        assert result["agnocast_state_observed"] is False
        # Plain Humble rclpy raises KeyboardInterrupt on SIGINT only.
        assert result["keyboard_interrupt"] is (signum == signal.SIGINT)
        assert_clean_transcript(child)
    finally:
        child.cleanup()


@pytest.mark.parametrize("signum", [signal.SIGINT, signal.SIGTERM])
def test_rclpy_signal_is_restored_after_adapter_close(
    tmp_path: Path, signum: signal.Signals
) -> None:
    child = start_child("signal_after_close", tmp_path, 10 + int(signum))
    try:
        child.wait_event("ready")
        child.process.send_signal(signum)
        result = child.finish()
        assert result["phase"] == "after_close"
        assert result["rclpy_signal_observed"] is True
        assert result["agnocast_state_observed"] is False
        assert result["keyboard_interrupt"] is (signum == signal.SIGINT)
        assert_clean_transcript(child)
    finally:
        child.cleanup()
