"""Opt-in public rclpy subscriber delivery using the actual native transport."""

from pathlib import Path
import sys
import time
from urllib.parse import unquote
import uuid

import pytest

from system_runtime import command, launch_pair, payload_checksum, preflight
from system_runtime import system_tld_requested, wait_connected


@pytest.fixture(scope="module")
def config():
    if not system_tld_requested():
        pytest.skip("set RCLPY_AGNOCAST_RUN_SYSTEM_TLD=1 for actual GuardCondition TLD")
    return preflight()


@pytest.fixture
def pair(config, request):
    topic = f"/guard_tld_{uuid.uuid4().hex}"
    environment = config.child_environment(20 + uuid.uuid4().int % 180)
    count, limit = getattr(request, "param", (256, 2 * 1024 * 1024))
    subscriber = [
        sys.executable, str(Path(__file__).with_name("guard_process.py")), "--topic", topic,
        "--count", str(count), "--limit", str(limit),
    ]
    publisher = [
        sys.executable, str(config.publisher_process), "--topic", topic,
        "--depth", "256", "--max-payload-bytes", str(2 * 1024 * 1024),
    ]
    with launch_pair(subscriber, publisher, environment) as pair:
        yield pair


def publish(publisher, size, sequence):
    event = command(publisher, f"publish {size} {sequence}", "PUBLISHED")
    assert int(event["checksum"]) == payload_checksum(sequence, size)


@pytest.mark.parametrize("size", [0, 1, 4096, 1024 * 1024])
def test_callback_uses_rclpy_thread_and_preserves_zero_copy_readonly_payload(pair, size):
    publisher, subscriber = pair
    publish(publisher, size, 37)
    command(subscriber, "wait 1", "STATS")
    stats = command(subscriber, "spin 1", "SPUN")
    details = command(subscriber, "details", "DETAILS")
    assert int(details["sequence"]) == 37
    assert int(details["checksum"]) == payload_checksum(37, size)
    assert details["readonly"] == "true"
    if size:
        assert (details["native_address"] == details["callback_address"]
                == details["view_address"] == details["numpy_address"])
    assert stats["caller_tid"] == details["python_tid"] == stats["dispatch_tids"]
    assert stats["trigger_tids"] == stats["notifier_tid"]
    assert len({stats["native_tid"], stats["notifier_tid"], details["python_tid"]}) == 3
    assert stats["triggers"] == stats["dispatches"] == "1"
    assert stats["received_refs"] == stats["released_refs"] == "1"


def test_one_guard_trigger_drains_sixteen_queued_messages(pair):
    publisher, subscriber = pair
    for sequence in range(16):
        publish(publisher, 8, sequence)
    before = command(subscriber, "wait 16", "STATS")
    assert before["queued_leases"] == "16"
    assert before["triggers"] == "1"
    after = command(subscriber, "spin 16", "SPUN")
    assert after["triggers"] == after["dispatches"] == "1"
    assert unquote(after["sequences"]) == ",".join(map(str, range(16)))
    assert after["received_refs"] == after["released_refs"] == "16"


def test_callback_exception_releases_current_and_pending_then_next_delivery_works(pair):
    publisher, subscriber = pair
    command(subscriber, "mode error", "MODE")
    for sequence in (1, 2):
        publish(publisher, 8, sequence)
    command(subscriber, "wait 2", "STATS")
    error = command(subscriber, "spin-error", "CALLBACK_ERROR")
    assert error["error"] == "LookupError"
    assert unquote(error["message"]) == "guard callback sentinel"
    stats = command(subscriber, "stats", "STATS")
    assert stats["outstanding_leases"] == stats["queued_leases"] == "0"
    assert stats["received_refs"] == stats["released_refs"] == "2"
    publish(publisher, 8, 3)
    stats = command(subscriber, "spin 2", "SPUN")
    assert unquote(stats["sequences"]) == "1,3"
    assert stats["received_refs"] == stats["released_refs"] == "3"
    command(subscriber, "close", "CLOSED_ENDPOINT")


def test_rearm_and_concurrent_burst_preserve_delivery_with_timer_and_service(pair):
    publisher, subscriber = pair
    command(subscriber, "coexist", "COEXIST_READY")
    for sequence in range(100):
        publish(publisher, 8, sequence)
        before = command(subscriber, f"spin {sequence + 1}", "SPUN")
    subscriber.send("spin 200")
    for sequence in range(100, 200):
        publish(publisher, 8, sequence)
    stats = subscriber.wait_event("SPUN")
    assert unquote(stats["sequences"]) == ",".join(map(str, range(200)))
    assert stats["received_refs"] == stats["released_refs"] == "200"
    assert stats["dropped_outstanding"] == stats["dropped_oversize"] == "0"
    assert stats["queued_leases"] == stats["outstanding_leases"] == "0"
    assert int(stats["timer_count"]) > int(before["timer_count"])
    assert int(stats["service_count"]) > int(before["service_count"])
    service_samples = map(int, unquote(stats["service_samples"]).split(","))
    assert any(100 < received < 200 for received in service_samples)


def test_guard_task_scheduled_before_close_suppresses_user_callback(pair):
    publisher, subscriber = pair
    publish(publisher, 8, 23)
    command(subscriber, "wait 1", "STATS")
    stats = command(subscriber, "close-pending", "STALE_SUPPRESSED")
    assert stats["callbacks"] == "0"
    assert stats["dispatches"] == "1"
    assert stats["guards"] == stats["registry"] == "0"
    assert stats["received_refs"] == stats["released_refs"] == "1"
    assert stats["notifier_alive"] == "false"


@pytest.mark.parametrize("pair", [(8, 64)], indirect=True)
def test_thirty_second_overload_preserves_held_owner_and_recovers_every_round(pair, request):
    publisher, subscriber = pair
    command(subscriber, "mode hold-view", "MODE")
    publish(publisher, 8, 0)
    command(subscriber, "spin-forget 1", "SPUN")
    command(subscriber, "mode inspect", "MODE")
    started = time.monotonic()
    rounds = 0
    while rounds < 20 or time.monotonic() - started < 30:
        first = 1 + 64 * rounds
        for sequence in range(first, first + 64):
            publish(publisher, 8, sequence)
        rounds += 1
        received = 1 + 64 * rounds
        dropped = 57 * rounds
        queued = command(subscriber, f"wait {received}", "STATS")
        assert int(queued["received_total"]) == received
        assert int(queued["dropped_outstanding"]) == dropped
        assert queued["dropped_oversize"] == "0"
        assert queued["queued_leases"] == "7"
        assert queued["outstanding_leases"] == "8"
        assert queued["outstanding_payload_bytes"] == "64"
        drained = command(subscriber, "spin-forget 7", "SPUN")
        assert unquote(drained["sequences"]) == ",".join(map(str, range(first, first + 7)))
        assert int(drained["delivered_total"]) == 1 + 7 * rounds
        assert int(drained["delivered_total"]) + dropped == received
        assert drained["queued_leases"] == "0"
        assert drained["outstanding_leases"] == "1"
        assert drained["outstanding_payload_bytes"] == "8"
        assert int(drained["received_refs"]) == received
        assert int(drained["released_refs"]) == received - 1
    elapsed = time.monotonic() - started
    released = command(subscriber, "release", "RELEASED")
    assert released["queued_leases"] == released["outstanding_leases"] == "0"
    assert released["outstanding_payload_bytes"] == "0"
    assert int(released["received_refs"]) == int(released["released_refs"]) == received
    request.node.user_properties.extend([
        ("overload_rounds", rounds), ("overload_messages", received),
        ("overload_elapsed_seconds", round(elapsed, 6)),
        ("overload_delivered", 1 + 7 * rounds), ("overload_dropped", dropped),
    ])


def test_endpoint_recreate_removes_guard_and_preserves_sibling(pair):
    publisher, subscriber = pair
    command(subscriber, "side-create", "SIDE_READY")
    for sequence in (1, 2, 3):
        publish(publisher, 8, sequence)
        command(subscriber, "spin 1", "SPUN")
        stats = command(subscriber, f"side-spin {sequence}", "SIDE_SPUN")
        assert stats["guards"] == stats["registry"] == "2"
        closed = command(subscriber, "close", "CLOSED_ENDPOINT")
        assert closed["guards"] == closed["registry"] == "1"
        assert closed["notifier_alive"] == "false"
        recreated = command(subscriber, "recreate", "RECREATED")
        assert recreated["guards"] == recreated["registry"] == "2"
        wait_connected(subscriber)


@pytest.mark.parametrize("kind", ["lease", "view"])
def test_retained_callback_owner_blocks_close_then_allows_retry(pair, kind):
    publisher, subscriber = pair
    command(subscriber, f"mode hold-{kind}", "MODE")
    publish(publisher, 8, 17)
    command(subscriber, "spin 1", "SPUN")
    busy = command(subscriber, "close", "BUSY")
    assert "Lease/view=1" in unquote(busy["message"])
    stats = command(subscriber, "stats", "STATS")
    assert stats["guards"] == stats["registry"] == "1"
    assert stats["notifier_alive"] == "false"
    command(subscriber, "release", "RELEASED")
    closed = command(subscriber, "close", "CLOSED_ENDPOINT")
    assert closed["guards"] == closed["registry"] == "0"
    command(subscriber, "close", "CLOSED_ENDPOINT")


@pytest.mark.parametrize("target", ["close", "runtime-close"])
def test_callback_self_close_fails_fast_for_dispatcher_and_retries_outside(pair, target):
    publisher, subscriber = pair
    command(subscriber, f"mode {target}", "MODE")
    publish(publisher, 8, 19)
    stats = command(subscriber, "spin 1", "SPUN")
    assert stats["busy_count"] == "1"
    assert stats["outstanding_leases"] == "0"
    assert stats["guards"] == stats["registry"] == "1"
    closed = command(subscriber, target, "CLOSED_ENDPOINT")
    assert closed["guards"] == closed["registry"] == "0"
    assert closed["notifier_alive"] == "false"


@pytest.mark.parametrize("reason", ["cancel", "failure"])
def test_idle_notifier_wakes_on_native_exit_without_take_polling(pair, reason):
    _, subscriber = pair
    command(subscriber, f"stop {reason}", "STOPPED")
    error = command(subscriber, "spin-error", "CALLBACK_ERROR")
    assert error["error"] == "RuntimeError"
    assert ("spin failure" if reason == "failure" else "stopped") in unquote(error["message"])
    closed = command(subscriber, "close", "CLOSED_ENDPOINT")
    assert closed["guards"] == closed["registry"] == "0"
    assert closed["notifier_alive"] == "false"


@pytest.mark.parametrize("reason", ["sigint", "sigterm"])
def test_signal_shutdown_with_public_subscription_joins_idle_notifier(pair, reason):
    _, subscriber = pair
    stopped = command(subscriber, f"stop {reason}", "STOPPED")
    assert stopped["context_ok"] == "false"
    closed = command(subscriber, "runtime-close", "CLOSED_ENDPOINT")
    assert closed["guards"] == closed["registry"] == "0"
    assert closed["notifier_alive"] == "false"


def test_multithreaded_independent_callbacks_reject_competing_borrow_and_recover(pair):
    publisher, subscriber = pair
    subscriber.send("competing-borrow")
    subscriber.wait_event("COMPETING_READY")
    publish(publisher, 8, 700)
    subscriber.wait_event("COMPETING_RETRY_READY")
    publish(publisher, 8, 701)
    result = subscriber.wait_event("COMPETING")
    assert result["threads"] == "2"
    assert result["recovered"] == "true"
    assert "busy" in unquote(result["message"]).lower()
