"""Opt-in real kmod/heaphook publisher system TLD."""

from __future__ import annotations

import sys
import uuid

import pytest

from system_runtime import PreflightError
from system_runtime import Probe
from system_runtime import SystemConfig
from system_runtime import parse_event
from system_runtime import payload_checksum
from system_runtime import preflight
from system_runtime import system_tld_requested
from system_runtime import wait_connected


PAYLOAD_LIMIT = 4096
_PREFLIGHT_CHECKED = False
_PREFLIGHT_CONFIG = None
_PREFLIGHT_FAILURE = None


def test_event_parser_rejects_ambiguous_records():
    assert parse_event("ordinary output") is None
    assert parse_event("EVENT name=READY pid=42") == {"name": "READY", "pid": "42"}
    with pytest.raises(RuntimeError, match="malformed"):
        parse_event("EVENT name=READY name=SECOND")


def test_payload_checksum_wraps_each_pattern_byte():
    assert payload_checksum(254, 4) == 254 + 255 + 0 + 1
    assert payload_checksum(7, 0) == 0


def _preflight_once() -> SystemConfig:
    global _PREFLIGHT_CHECKED, _PREFLIGHT_CONFIG, _PREFLIGHT_FAILURE
    if not _PREFLIGHT_CHECKED:
        try:
            _PREFLIGHT_CONFIG = preflight()
        except PreflightError as error:
            _PREFLIGHT_FAILURE = error
        _PREFLIGHT_CHECKED = True
    if _PREFLIGHT_FAILURE is not None:
        raise _PREFLIGHT_FAILURE
    assert _PREFLIGHT_CONFIG is not None
    return _PREFLIGHT_CONFIG


def test_requested_system_preflight():
    if not system_tld_requested():
        pytest.skip("system TLD was not explicitly requested")
    try:
        _preflight_once()
    except PreflightError as error:
        pytest.fail(f"requested system TLD failed preflight: {error}", pytrace=False)


@pytest.fixture(scope="module")
def system_config() -> SystemConfig:
    if not system_tld_requested():
        pytest.skip("set RCLPY_AGNOCAST_RUN_SYSTEM_TLD=1 for real backend tests")
    try:
        return _preflight_once()
    except PreflightError:
        pytest.skip("preflight failed; see test_requested_system_preflight")


def _publisher_command(config: SystemConfig, topic: str) -> list[str]:
    return [
        sys.executable,
        str(config.publisher_process),
        "--topic",
        topic,
        "--depth",
        "1",
        "--max-payload-bytes",
        str(PAYLOAD_LIMIT),
    ]


@pytest.fixture
def backend_pair(system_config: SystemConfig):
    topic = f"/rclpy_agnocast_system_{uuid.uuid4().hex}"
    domain_id = 20 + uuid.uuid4().int % 180
    environment = system_config.child_environment(domain_id)
    probes = []
    try:
        subscriber = Probe(
            [str(system_config.subscriber), topic, "1"], environment
        )
        probes.append(subscriber)
        ready = subscriber.wait_event("READY")
        assert int(ready["pid"]) == subscriber.process.pid

        publisher = Probe(_publisher_command(system_config, topic), environment)
        probes.append(publisher)
        ready = publisher.wait_event("READY")
        assert int(ready["pid"]) == publisher.process.pid
        assert int(ready["max_payload_bytes"]) == PAYLOAD_LIMIT
        wait_connected(subscriber)
        yield publisher, subscriber
    finally:
        for probe in reversed(probes):
            probe.shutdown()
        for probe in probes:
            assert probe.process.returncode == 0, "\n".join(probe.transcript[-30:])


def _assert_roundtrip(published, received, size: int, sequence: int) -> None:
    checksum = payload_checksum(sequence, size)
    assert int(published["sequence"]) == sequence
    assert int(published["bytes"]) == size
    assert int(published["checksum"]) == checksum
    assert published["format"] == "B"
    assert int(published["itemsize"]) == 1
    assert int(published["ndim"]) == 1
    assert int(published["stride"]) == 1
    assert published["readonly"] == "false"
    assert published["contiguous"] == "true"
    assert published["invalidated"] == "true"
    if size:
        assert int(published["data_addr"], 0) != 0

    assert int(received["sequence"]) == sequence
    assert int(received["bytes"]) == size
    assert int(received["checksum"]) == checksum
    assert received["valid"] == "true"
    assert received["content_valid"] == "true"


def _publish_and_receive(
    publisher: Probe, subscriber: Probe, size: int, sequence: int
) -> None:
    publisher.send(f"publish {size} {sequence}")
    published = publisher.wait_event("PUBLISHED")
    received = subscriber.wait_event("RX")
    _assert_roundtrip(published, received, size, sequence)


def test_actual_boundaries_cancel_recovery_and_recreate(backend_pair):
    publisher, subscriber = backend_pair
    for sequence, size in enumerate((0, 1, PAYLOAD_LIMIT - 1, PAYLOAD_LIMIT), 100):
        _publish_and_receive(publisher, subscriber, size, sequence)

    publisher.send("cancel 17 200")
    cancelled = publisher.wait_event("CANCELLED")
    assert cancelled["sequence"] == "200"
    _publish_and_receive(publisher, subscriber, 17, 201)

    for size, expected in (
        (PAYLOAD_LIMIT + 1, {"ValueError"}),
        (-1, {"ValueError"}),
        (1 << 100, {"TypeError", "OverflowError"}),
    ):
        publisher.send(f"reject {size}")
        rejected = publisher.wait_event("REJECTED")
        assert rejected["error"] in expected

    publisher.send("recreate")
    publisher.wait_event("RECREATED")
    wait_connected(subscriber)
    _publish_and_receive(publisher, subscriber, 64, 300)


def test_actual_alias_exception_and_active_close_recover(backend_pair):
    publisher, subscriber = backend_pair
    publisher.send("alias-barrier 32 400")
    barrier = publisher.wait_event("ALIAS_BARRIER")
    assert barrier["blocked"] == "BufferError"
    received = subscriber.wait_event("RX")
    _assert_roundtrip(barrier, received, 32, 400)

    publisher.send("context-error 32")
    context = publisher.wait_event("CONTEXT_ERROR")
    assert context == {
        "name": "CONTEXT_ERROR",
        "error": "LookupError",
        "original": "context-sentinel",
    }
    _publish_and_receive(publisher, subscriber, 32, 401)

    publisher.send("close-active 32")
    recovered = publisher.wait_event("ACTIVE_CLOSE_RECOVERED")
    assert recovered["blocked"] == "RuntimeError"
    wait_connected(subscriber)
    _publish_and_receive(publisher, subscriber, 32, 402)


UNOBSERVED_NATIVE_CASES = (
    ("PUB-01", "production worker/backend thread IDs are not exposed"),
    ("PUB-02", "caller and worker Agnocast TLS values are not exposed"),
    ("PUB-03", "the filling backend-access epoch and 1,000-run are pending"),
    ("PUB-06", "the production native data pointer is not exposed"),
    ("PUB-09", "cancel recovery is covered, but terminal TLS zero is not exposed"),
    ("PUB-11", "held-view deferred-cancel observation is pending"),
    ("PUB-12", "production stale-token and foreign-thread coverage is pending"),
    ("PUB-14", "native borrow-entry count is not exposed"),
    ("PUB-15", "production allocation-failure injection is not available"),
    ("PUB-17", "worker-TID allocation auditing is not available"),
    ("PUB-18", "production terminal enqueue count and TLS are not exposed"),
)


@pytest.mark.parametrize("pub_case,reason", UNOBSERVED_NATIVE_CASES)
def test_native_observation_case_is_not_claimed(
    system_config: SystemConfig, pub_case: str, reason: str
) -> None:
    del system_config
    pytest.xfail(f"{pub_case} unsupported in the current native observation API: {reason}")
