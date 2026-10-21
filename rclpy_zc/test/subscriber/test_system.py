"""Opt-in native subscriber TLD with a separate direct-fill Python publisher."""

from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import sys
import threading
from urllib.parse import unquote
import uuid

import pytest

from system_runtime import Probe
from system_runtime import payload_checksum
from system_runtime import preflight
from system_runtime import system_tld_requested
from system_runtime import wait_connected


@pytest.fixture(scope="module")
def config():
    if not system_tld_requested():
        pytest.skip("set RCLPY_AGNOCAST_RUN_SYSTEM_TLD=1 for actual subscriber TLD")
    return preflight()


@pytest.fixture
def pair_factory(config):
    probes = []

    def create(count=4, limit=2 * 1024 * 1024):
        topic = f"/subscriber_tld_{uuid.uuid4().hex}"
        environment = config.child_environment(20 + uuid.uuid4().int % 180)
        subscriber = Probe([
            sys.executable, str(Path(__file__).with_name("process.py")),
            "--topic", topic, "--count", str(count), "--limit", str(limit),
        ], environment)
        probes.append(subscriber)
        subscriber.wait_event("READY")
        publisher = Probe([
            sys.executable, str(config.publisher_process), "--topic", topic,
            "--depth", "128", "--max-payload-bytes", str(2 * 1024 * 1024),
        ], environment)
        probes.append(publisher)
        publisher.wait_event("READY")
        wait_connected(subscriber)
        return publisher, subscriber

    yield create
    for probe in reversed(probes):
        probe.shutdown()
    for probe in probes:
        assert probe.process.returncode == 0, "\n".join(probe.transcript[-30:])


def command(probe, text, event):
    probe.send(text)
    return probe.wait_event(event)


def publish(publisher, subscriber, size, sequence, received):
    published = command(publisher, f"publish {size} {sequence}", "PUBLISHED")
    assert int(published["checksum"]) == payload_checksum(sequence, size)
    return command(subscriber, f"wait {received}", "STATS")


@pytest.mark.parametrize("size", [0, 1, 4096, 1024 * 1024])
def test_typed_native_buffer_addresses_readonly_and_release(pair_factory, size):
    publisher, subscriber = pair_factory()
    stats = publish(publisher, subscriber, size, 41, 1)
    assert int(stats["callback_tid"]) != int(stats["caller_tid"])
    event = command(subscriber, "inspect", "INSPECTED")
    assert int(event["sequence"]) == 41
    assert int(event["size"]) == size
    assert int(event["checksum"]) == payload_checksum(41, size)
    assert event["readonly"] == "true"
    if size:
        assert (int(event["native_address"]) == int(event["view_address"])
                == int(event["numpy_address"]) == int(event["callback_address"]))
    stats = command(subscriber, "stats", "STATS")
    assert int(stats["outstanding_leases"]) == 0
    assert int(stats["received_refs"]) == int(stats["released_refs"]) == 1


@pytest.mark.parametrize("kind", ["view", "slice", "numpy"])
def test_wrapper_release_retains_data_until_foreign_last_alias(pair_factory, kind):
    publisher, subscriber = pair_factory()
    publish(publisher, subscriber, 64, 17, 1)
    held = command(subscriber, f"hold {kind}", "HELD")
    assert int(held["released_refs"]) == 0
    assert int(held["outstanding_payload_bytes"]) == 64
    command(subscriber, "check-held", "HELD_VALID")
    released = command(subscriber, "release foreign", "RELEASED")
    assert int(released["released_refs"]) == 1
    assert int(released["outstanding_leases"]) == 0
    assert int(released["release_tid"]) != int(released["caller_tid"])


def test_count_admission_accounts_held_and_queued_then_recovers(pair_factory):
    publisher, subscriber = pair_factory(count=2, limit=32)
    publish(publisher, subscriber, 8, 1, 1)
    command(subscriber, "hold view", "HELD")
    publish(publisher, subscriber, 8, 2, 2)
    stats = publish(publisher, subscriber, 8, 3, 3)
    assert int(stats["queued_leases"]) == 1
    assert int(stats["outstanding_leases"]) == 2
    assert int(stats["outstanding_payload_bytes"]) == 16
    assert int(stats["dropped_outstanding"]) == 1
    assert int(command(subscriber, "inspect", "INSPECTED")["sequence"]) == 2
    command(subscriber, "release", "RELEASED")
    publish(publisher, subscriber, 8, 4, 4)
    assert int(command(subscriber, "inspect", "INSPECTED")["sequence"]) == 4
    stats = command(subscriber, "stats", "STATS")
    assert int(stats["received_refs"]) == int(stats["released_refs"]) == 4


def test_byte_admission_oversize_drop_and_queued_close(pair_factory):
    publisher, subscriber = pair_factory(count=4, limit=8)
    publish(publisher, subscriber, 5, 1, 1)
    command(subscriber, "hold numpy", "HELD")
    publish(publisher, subscriber, 4, 2, 2)
    publish(publisher, subscriber, 9, 3, 3)
    stats = publish(publisher, subscriber, 3, 4, 4)
    assert int(stats["outstanding_payload_bytes"]) == 8
    assert int(stats["dropped_outstanding"]) == 1
    assert int(stats["dropped_oversize"]) == 1
    busy = command(subscriber, "close", "BUSY")
    assert int(busy["queued_leases"]) == 0
    assert int(busy["outstanding_leases"]) == 1
    command(subscriber, "check-held", "HELD_VALID")
    # A closing endpoint must still consume/reset native arrivals while a held view exists.
    stats = publish(publisher, subscriber, 1, 5, 5)
    assert int(stats["outstanding_payload_bytes"]) == 5
    command(subscriber, "release", "RELEASED")
    closed = command(subscriber, "close", "CLOSED_ENDPOINT")
    assert int(closed["outstanding_payload_bytes"]) == 0
    assert int(closed["released_refs"]) == 5


@pytest.mark.parametrize("close_handle_first", [False, True])
def test_runtime_close_retries_after_export_and_is_idempotent(pair_factory, close_handle_first):
    publisher, subscriber = pair_factory()
    publish(publisher, subscriber, 8, 11, 1)
    command(subscriber, "hold slice", "HELD")
    command(subscriber, "runtime-close", "BUSY")
    command(subscriber, "check-held", "HELD_VALID")
    command(subscriber, "release foreign", "RELEASED")
    if close_handle_first:
        command(subscriber, "close", "CLOSED_ENDPOINT")
    closed = command(subscriber, "runtime-close", "CLOSED_ENDPOINT")
    assert int(closed["released_refs"]) == 1
    command(subscriber, "runtime-close", "CLOSED_ENDPOINT")


def test_native_endpoint_recreate_preserves_another_subscription(pair_factory):
    publisher, subscriber = pair_factory()
    command(subscriber, "side-create", "SIDE_READY")
    publish(publisher, subscriber, 8, 1, 1)
    command(subscriber, "inspect", "INSPECTED")
    command(subscriber, "side-inspect", "INSPECTED")
    command(subscriber, "recreate", "RECREATED")
    wait_connected(subscriber)
    publish(publisher, subscriber, 8, 2, 1)
    assert int(command(subscriber, "inspect", "INSPECTED")["sequence"]) == 2
    assert int(command(subscriber, "side-inspect", "INSPECTED")["sequence"]) == 2


def test_native_burst_progresses_without_gil_or_queue_capacity_wait(pair_factory):
    publisher, subscriber = pair_factory(count=2, limit=32)
    command(subscriber, "block-gil 8", "BLOCKING")
    for sequence in range(8):
        command(publisher, f"publish 8 {sequence}", "PUBLISHED")
    stats = subscriber.wait_event("GIL_PROGRESS")
    assert int(stats["received_total"]) == 8
    assert int(stats["queued_leases"]) == 2
    assert int(stats["dropped_outstanding"]) == 6
    assert int(stats["released_refs"]) == 6
    assert int(command(subscriber, "inspect", "INSPECTED")["sequence"]) == 0
    assert int(command(subscriber, "inspect", "INSPECTED")["sequence"]) == 1
    closed = command(subscriber, "close", "CLOSED_ENDPOINT")
    assert int(closed["released_refs"]) == 8


def test_close_during_traffic_releases_references_and_allows_recreate(pair_factory):
    publisher, subscriber = pair_factory(count=2, limit=32)
    started = threading.Event()

    def stream():
        for sequence in range(300):
            command(publisher, f"publish 8 {sequence}", "PUBLISHED")
            started.set()

    with ThreadPoolExecutor(max_workers=1) as executor:
        traffic = executor.submit(stream)
        assert started.wait(timeout=10)
        closed = command(subscriber, "close", "CLOSED_ENDPOINT")
        assert int(closed["outstanding_leases"]) == 0
        assert int(closed["received_refs"]) == int(closed["released_refs"])
        traffic.result(timeout=20)
    command(subscriber, "recreate", "RECREATED")
    wait_connected(subscriber)
    publish(publisher, subscriber, 8, 999, 1)
    assert int(command(subscriber, "inspect", "INSPECTED")["sequence"]) == 999


@pytest.mark.parametrize("exit_kind", ["cancel", "failure", "sigint", "sigterm"])
@pytest.mark.parametrize("held", [False, True])
def test_endpoint_close_after_executor_exit_preserves_live_views(pair_factory, exit_kind, held):
    publisher, subscriber = pair_factory()
    publish(publisher, subscriber, 8, 21, 1)
    if held:
        command(subscriber, "hold view", "HELD")
    stopped = command(subscriber, f"stop-executor {exit_kind}", "EXECUTOR_STOPPED")
    assert ("spin failure" if exit_kind == "failure" else "stopped") in unquote(stopped["message"])
    if held:
        busy = command(subscriber, "close", "BUSY")
        assert int(busy["outstanding_leases"]) == 1
        command(subscriber, "check-held", "HELD_VALID")
        command(subscriber, "release foreign", "RELEASED")
    closed = command(subscriber, "close", "CLOSED_ENDPOINT")
    assert int(closed["received_refs"]) == int(closed["released_refs"]) == 1
    assert int(closed["outstanding_leases"]) == 0
    command(subscriber, "close", "CLOSED_ENDPOINT")
    command(subscriber, "runtime-close", "CLOSED_ENDPOINT")


def test_endpoint_close_recovers_when_spin_fails_before_queued_control_runs(pair_factory):
    publisher, subscriber = pair_factory()
    publish(publisher, subscriber, 8, 22, 1)
    command(subscriber, "fail-next-control", "CONTROL_ARMED")
    closed = command(subscriber, "close", "CLOSED_ENDPOINT")
    assert int(closed["received_refs"]) == int(closed["released_refs"]) == 1
    assert int(closed["outstanding_leases"]) == 0
    command(subscriber, "runtime-close", "CLOSED_ENDPOINT")
