"""Full-content, zero-copy and bounded-admission Python-to-Python checkpoint."""

import json
from urllib.parse import unquote

import pytest

from checkpoint.harness import balanced, PAYLOAD_LIMIT, publish, receive
from system_runtime import command


def test_one_thousand_acknowledged_messages_preserve_all_payloads_and_addresses(peers, request):
    publisher, subscriber = peers.pair()
    samples = []
    for sequence in range(1000):
        # Varied lengths and wrapped byte values detect stale/reordered payloads.
        size = 1 + sequence % 257
        sent = publish(publisher, size, sequence)
        stats, received = receive(subscriber, sequence + 1, size, sequence,
                                  publisher_pid=publisher.process.pid, single_arrival=True)
        assert int(stats["delivered_total"]) == sequence + 1
        assert stats["dropped_outstanding"] == stats["dropped_oversize"] == "0"
        balanced(stats, sequence + 1)
        if sequence in (0, 499, 999):
            samples.append(dict(sequence=sequence, publisher=sent, subscriber=received))
    assert unquote(stats["sequences"]) == ",".join(map(str, range(1000)))
    request.node.user_properties.extend([
        ("publisher_pid", publisher.process.pid), ("subscriber_pid", subscriber.process.pid),
        ("address_samples", json.dumps(samples, sort_keys=True)),
    ])


@pytest.mark.parametrize("size", [0, 1, 17, 4095, 4096, 4097, 1024 * 1024, PAYLOAD_LIMIT])
def test_boundary_sizes_have_full_content_and_readonly_aliases(peers, size, request):
    publisher, subscriber = peers.pair()
    sent = publish(publisher, size, 537)
    stats, received = receive(subscriber, 1, size, 537,
                              publisher_pid=publisher.process.pid, single_arrival=True)
    balanced(stats, 1)
    request.node.user_properties.append((
        "addresses", json.dumps(dict(publisher=sent, subscriber=received), sort_keys=True),
    ))


@pytest.mark.parametrize("mode", ["hold-lease", "hold-view"])
@pytest.mark.parametrize("peers", [(2, 32)], indirect=True)
def test_held_owner_counts_toward_admission_and_release_restores_delivery(peers, mode):
    publisher, subscriber = peers.pair()
    command(subscriber, f"mode {mode}", "MODE")
    publish(publisher, 8, 10)
    receive(subscriber, 1, 8, 10, publisher_pid=publisher.process.pid)
    command(subscriber, "mode inspect", "MODE")
    for sequence in (11, 12, 13):
        publish(publisher, 8, sequence)
    queued = command(subscriber, "wait 4", "STATS")
    assert queued["outstanding_leases"] == "2"
    assert queued["outstanding_payload_bytes"] == "16"
    assert queued["queued_leases"] == "1"
    assert queued["dropped_outstanding"] == "2"
    assert queued["dropped_oversize"] == "0"
    # Only the first arrival survives: DropNewest rather than replace-oldest.
    stats, _ = receive(subscriber, 2, 8, 11, publisher_pid=publisher.process.pid)
    assert stats["outstanding_leases"] == "1"
    released = command(subscriber, "release", "RELEASED")
    balanced(released, 4)
    publish(publisher, 8, 14)
    stats, _ = receive(subscriber, 3, 8, 14, publisher_pid=publisher.process.pid)
    balanced(stats, 5)
    assert unquote(stats["sequences"]) == "10,11,14"


@pytest.mark.parametrize("peers", [(4, 8)], indirect=True)
def test_aggregate_bytes_and_single_oversize_drop_independently_then_recover(peers):
    publisher, subscriber = peers.pair()
    command(subscriber, "mode hold-view", "MODE")
    publish(publisher, 5, 20)
    receive(subscriber, 1, 5, 20, publisher_pid=publisher.process.pid)
    command(subscriber, "mode inspect", "MODE")
    for size, sequence in ((4, 21), (9, 22), (3, 23)):
        publish(publisher, size, sequence)
    queued = command(subscriber, "wait 4", "STATS")
    assert queued["outstanding_leases"] == "2"
    assert queued["outstanding_payload_bytes"] == "8"
    assert queued["queued_leases"] == "1"
    assert queued["dropped_outstanding"] == queued["dropped_oversize"] == "1"
    receive(subscriber, 2, 3, 23, publisher_pid=publisher.process.pid)
    balanced(command(subscriber, "release", "RELEASED"), 4)
    publish(publisher, 8, 24)
    stats, _ = receive(subscriber, 3, 8, 24, publisher_pid=publisher.process.pid)
    balanced(stats, 5)
    assert unquote(stats["sequences"]) == "20,23,24"
