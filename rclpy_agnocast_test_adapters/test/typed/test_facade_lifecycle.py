"""Actual typed process exits and endpoint replacement with live peer progress."""

import pytest

from checkpoint.resources import cleaned, endpoint_count, heap_path, wait_for
from facade_peers import transfer, TypedPeers
from system_runtime import command, preflight, system_tld_requested, wait_connected


@pytest.fixture
def peers():
    """Use matching already-loaded artifacts and keep every launched PID owned."""
    if not system_tld_requested():
        pytest.skip('set RCLPY_AGNOCAST_RUN_SYSTEM_TLD=1 for typed lifecycle TLD')
    peers = TypedPeers(preflight())
    try:
        yield peers
    finally:
        peers.close()


@pytest.mark.parametrize('departing', ['publisher', 'subscriber'])
@pytest.mark.parametrize('exit_mode', ['normal', 'sigint', 'sigterm', 'sigkill'])
def test_peer_exit_preserves_survivor_and_supports_endpoint_and_process_replacement(
        peers, departing, exit_mode):
    """Release held references, replace the departed PID, and deliver through the survivor."""
    publisher, subscriber = peers.pair()
    received = transfer(publisher, subscriber, 1, release=False)
    assert endpoint_count(peers, 'publisher') == endpoint_count(peers, 'subscriber') == 1
    assert heap_path(publisher).exists() and heap_path(subscriber).exists()
    victim = publisher if departing == 'publisher' else subscriber
    survivor = subscriber if departing == 'publisher' else publisher
    survivor_pid = survivor.process.pid
    peers.stop(victim, exit_mode)
    wait_for(lambda: not heap_path(victim).exists(), 'departed typed peer heap name remains')
    assert survivor.process.poll() is None
    if departing == 'publisher':
        stats = command(subscriber, 'stats', 'STATS')
        assert stats['retained'] == '1'
        assert stats['accounted_bytes'] == received['accounted_bytes']
        # release checks the held byte after the producer heap name was unlinked.
        command(subscriber, 'release', 'RELEASED')
        command(subscriber, 'recreate', 'RECREATED')
        wait_for(lambda: endpoint_count(peers, 'publisher') == 0, 'dead publisher remains')
        publisher = peers.start('publisher')
        expected_count = 2
    else:
        wait_for(lambda: endpoint_count(peers, 'subscriber') == 0, 'dead subscriber remains')
        command(publisher, 'recreate', 'RECREATED')
        subscriber = peers.start('subscriber')
        expected_count = 1
    assert survivor.process.pid == survivor_pid and survivor.process.poll() is None
    wait_connected(subscriber)
    transfer(publisher, subscriber, expected_count)
    peers.stop(subscriber)
    peers.stop(publisher)
    cleaned(peers)


def test_recreating_typed_subscription_preserves_sibling_endpoint_progress(peers):
    """Destroy only one endpoint while another subscription shares its Node and runtime."""
    publisher, subscriber = peers.pair()
    command(subscriber, 'add_sibling', 'ADDED')
    assert endpoint_count(peers, 'subscriber') == 2
    transfer(publisher, subscriber, 1)
    wait_for(lambda: command(subscriber, 'stats', 'STATS')['sibling_received'] == '1',
             'sibling did not receive before recreation')
    command(subscriber, 'recreate', 'RECREATED')
    assert endpoint_count(peers, 'subscriber') == 2
    wait_connected(subscriber)
    transfer(publisher, subscriber, 2)
    wait_for(lambda: command(subscriber, 'stats', 'STATS')['sibling_received'] == '2',
             'surviving sibling did not progress after recreation')
    peers.stop(subscriber)
    peers.stop(publisher)
    cleaned(peers)


def test_native_receive_adoption_failure_preserves_held_alias_and_original_exception(peers):
    """Inject bad_alloc after ownership adoption, not a literal allocator exhaustion."""
    publisher, subscriber = peers.pair()
    received = transfer(publisher, subscriber, 1, release=False)
    before = command(subscriber, 'stats', 'STATS')
    assert before['received_refs'] == before['admitted'] == '1'
    assert before['released_refs'] == '0'
    command(subscriber, 'fail_receive', 'ARMED')
    command(publisher, 'publish', 'PUBLISHED')
    assert subscriber.wait_event('INJECTED_FAILURE')['kind'] == 'MemoryError'
    wait_for(lambda: command(subscriber, 'stats', 'STATS')['released_refs'] == '1',
             'failed receive native reference was not returned')
    stats = command(subscriber, 'stats', 'STATS')
    assert stats['received_refs'] == '2'
    assert stats['admitted'] == stats['received'] == stats['retained'] == '1'
    assert stats['accounted_bytes'] == received['accounted_bytes']
    # The held byte is checked before release, despite the endpoint's terminal failure.
    command(subscriber, 'release', 'RELEASED')
    command(subscriber, 'recreate', 'RECREATED')
    wait_connected(subscriber)
    transfer(publisher, subscriber, 2)


def test_native_factory_topic_failure_rolls_back_and_preserves_valid_endpoint(peers):
    """Native topic resolution failure leaves the live sibling and new endpoint usable."""
    publisher, subscriber = peers.pair()
    command(subscriber, 'add_sibling', 'ADDED')
    before = endpoint_count(peers, 'subscriber')
    assert before == 2
    command(subscriber, 'bad_topic', 'REJECTED')
    assert endpoint_count(peers, 'subscriber') == before
    command(subscriber, 'recreate', 'RECREATED')
    wait_connected(subscriber)
    transfer(publisher, subscriber, 1)
    wait_for(lambda: command(subscriber, 'stats', 'STATS')['sibling_received'] == '1',
             'sibling did not receive after native factory rollback')
