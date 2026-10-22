"""Crash, recreation and retained-mapping observations for public Python peers."""

import json

import pytest

from checkpoint.harness import balanced, publish, receive
from checkpoint.resources import assert_connected_resources, cleaned, endpoint_count
from checkpoint.resources import heap_path, mapping_snapshot
from checkpoint.resources import queues, wait_for
from system_runtime import command, wait_connected


def test_normal_shutdown_removes_owned_heaps_queues_and_connections(peers):
    publisher, subscriber = peers.pair()
    publish(publisher, 4096, 31)
    stats, _ = receive(subscriber, 1, 4096, 31, publisher_pid=publisher.process.pid)
    balanced(stats, 1)
    assert_connected_resources(peers, publisher, subscriber)
    peers.stop(subscriber)
    wait_for(lambda: endpoint_count(peers, "subscriber") == 0, "subscriber remains")
    peers.stop(publisher)
    cleaned(peers)


@pytest.mark.parametrize("mode", ["hold-lease", "hold-view"])
def test_publisher_sigkill_preserves_held_payload_then_endpoint_cleanup(peers, mode):
    publisher, subscriber = peers.pair()
    command(subscriber, f"mode {mode}", "MODE")
    publish(publisher, 4096, 41)
    stats, _ = receive(subscriber, 1, 4096, 41, publisher_pid=publisher.process.pid)
    assert stats["outstanding_leases"] == "1"
    assert_connected_resources(peers, publisher, subscriber)
    peers.kill(publisher)
    wait_for(lambda: not heap_path(publisher).exists(), "dead publisher heap name remains")
    # release rechecks every held byte after unlink; the mapping still owns the data.
    balanced(command(subscriber, "release", "RELEASED"), 1)
    # Upstream lazily reclaims the dead publisher entry on subscriber removal.
    command(subscriber, "recreate", "RECREATED")
    wait_for(lambda: endpoint_count(peers, "publisher") == 0, "dead publisher remains")
    command(subscriber, "mode inspect", "MODE")
    replacement = peers.publisher()
    wait_connected(subscriber)
    publish(replacement, 4096, 42)
    stats, _ = receive(subscriber, 1, 4096, 42, publisher_pid=replacement.process.pid)
    balanced(stats, 1)
    peers.stop(subscriber)
    peers.stop(replacement)
    cleaned(peers)


def test_subscriber_sigkill_releases_references_and_allows_recreate_in_live_publisher(peers):
    publisher, subscriber = peers.pair()
    command(subscriber, "mode hold-view", "MODE")
    publish(publisher, 4096, 51)
    stats, _ = receive(subscriber, 1, 4096, 51, publisher_pid=publisher.process.pid)
    assert stats["received_refs"] == "1" and stats["released_refs"] == "0"
    assert_connected_resources(peers, publisher, subscriber)
    old_queues = queues(peers)
    peers.kill(subscriber)
    wait_for(lambda: endpoint_count(peers, "subscriber") == 0, "dead subscriber remains")
    wait_for(lambda: not heap_path(subscriber).exists(), "dead subscriber heap name remains")
    wait_for(lambda: not any(path.exists() for path in old_queues), "dead subscriber MQ remains")
    assert publisher.process.poll() is None
    # Removing the old publisher can reclaim its entry only after dead subscriber refs clear.
    command(publisher, "recreate", "RECREATED")
    assert endpoint_count(peers, "publisher") == 1
    replacement = peers.subscriber()
    wait_connected(replacement)
    publish(publisher, 4096, 52)
    stats, _ = receive(replacement, 1, 4096, 52, publisher_pid=publisher.process.pid)
    balanced(stats, 1)
    peers.stop(replacement)
    peers.stop(publisher)
    cleaned(peers)


def test_publisher_pid_changes_preserve_recreate_and_record_mapping_retention(peers, request):
    subscriber = peers.subscriber()
    snapshots = [mapping_snapshot(subscriber)]
    publisher_pids = []
    for iteration in range(4):
        publisher = peers.publisher()
        publisher_pids.append(publisher.process.pid)
        wait_connected(subscriber)
        assert_connected_resources(peers, publisher, subscriber)
        publish(publisher, 1024 * 1024, 60 + iteration)
        stats, _ = receive(subscriber, 1, 1024 * 1024, 60 + iteration,
                           publisher_pid=publisher.process.pid)
        balanced(stats, 1)
        peers.stop(publisher)
        command(subscriber, "recreate", "RECREATED")
        wait_for(lambda: endpoint_count(peers, "publisher") == 0, "old publisher remains")
        wait_for(lambda: not heap_path(publisher).exists(), "old heap name remains")
        snapshots.append(mapping_snapshot(subscriber))
    assert len(set(publisher_pids)) == 4
    # No zero-growth assertion: upstream SubscriptionBase does not unmap publisher heaps.
    request.node.user_properties.extend([
        ("publisher_pids", json.dumps(publisher_pids)),
        ("mapping_snapshots", json.dumps(snapshots, sort_keys=True)),
    ])
    peers.stop(subscriber)
    cleaned(peers)
