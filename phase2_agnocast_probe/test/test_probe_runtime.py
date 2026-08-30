"""Unit tests for the Phase 2 probe's deterministic event protocol."""

import os
from pathlib import Path
import threading
import time

import pytest

from probe_runtime import Probe, parse_event, process_is_running, require, wait_gone


def test_parse_event_returns_all_fields():
    event = parse_event("EVENT name=RX sequence=7 valid=true")

    assert event == {"name": "RX", "sequence": "7", "valid": "true"}


def test_parse_event_ignores_non_event_output():
    assert parse_event("[INFO] unrelated Agnocast log") is None


@pytest.mark.parametrize(
    "line",
    [
        "EVENT name=RX name=PUB",
        "EVENT name",
        "EVENT =RX",
    ],
)
def test_parse_event_rejects_ambiguous_tokens(line):
    with pytest.raises(RuntimeError):
        parse_event(line)


def test_require_uses_the_supplied_failure_message():
    with pytest.raises(RuntimeError, match="expected failure"):
        require(False, "expected failure")


def test_wait_gone_observes_delayed_unlink(tmp_path):
    resource = tmp_path / "agnocast-resource"
    resource.touch()

    worker = threading.Thread(target=lambda: (time.sleep(0.05), resource.unlink()))
    worker.start()
    try:
        assert wait_gone(resource, timeout=1.0)
    finally:
        worker.join()


def test_quit_rejects_a_child_that_already_failed():
    probe = Probe("/bin/sh", "-c", ["exit 3"], os.environ.copy())
    probe.process.wait(timeout=1.0)

    try:
        with pytest.raises(RuntimeError, match="exit=3, expected=0"):
            probe.quit()
    finally:
        Probe.cleanup_all()


def test_process_is_running_distinguishes_current_and_missing_pid():
    host_pid = int(Path("/proc/self/stat").read_text(encoding="ascii").split()[0])
    assert process_is_running(host_pid)
    assert not process_is_running(2_147_483_647)
