"""Opt-in separate-process CPU Tensor delivery and retained-alias checkpoint."""

import json
from pathlib import Path
import signal
import sys
import uuid

import pytest

from checkpoint.resources import (
    assert_connected_resources, cleaned, heap_mappings, publisher_mapping)
from system_runtime import command, preflight, Probe, system_tld_requested, wait_connected


@pytest.fixture(scope='module')
def tensor_config():
    """Require explicit device opt-in and the matching kernel/userspace artifacts."""
    if not system_tld_requested():
        pytest.skip('set RCLPY_AGNOCAST_RUN_SYSTEM_TLD=1 for the CPU Tensor checkpoint')
    pytest.importorskip('torch')
    return preflight()


class TensorPeers:
    """Own both exact-artifact child processes and require their orderly teardown."""

    def __init__(self, config):
        """Reserve one topic and discovery domain for this exact peer pair."""
        self.config = config
        self.topic = f'/tensor_checkpoint_{uuid.uuid4().hex}'
        self.environment = config.child_environment(20 + uuid.uuid4().int % 180)
        self.children = []

    def start(self, mode):
        """Launch one independently initialized and preloaded Python participant."""
        child = Probe([
            sys.executable, str(Path(__file__).with_name('system_peers.py')),
            '--mode', mode, '--topic', self.topic,
        ], self.environment)
        self.children.append(child)
        assert int(child.wait_event('READY')['pid']) == child.process.pid
        return child

    def pair(self):
        """Require distinct PIDs, discovery, and nonempty owned native resources."""
        subscriber, publisher = self.start('subscriber'), self.start('publisher')
        assert publisher.process.pid != subscriber.process.pid
        wait_connected(subscriber)
        assert_connected_resources(self, publisher, subscriber)
        return publisher, subscriber

    def stop(self):
        """Require normal child exits and full removal of owned native resources."""
        for child in reversed(self.children):
            child.send('quit')
            child.wait_event('STOPPED')
            assert child.process.wait(timeout=15) == 0, child._failure('normal exit failed')
        cleaned(self)

    def close(self):
        """Reap every child even after an assertion fails."""
        for child in reversed(self.children):
            child.shutdown()
        for child in self.children:
            assert child.process.returncode == 0, child._failure('unexpected exit')


@pytest.fixture
def tensor_peers(tensor_config):
    """Keep cleanup active across partial startup and failed test assertions."""
    peers = TensorPeers(tensor_config)
    try:
        yield peers
    finally:
        peers.close()


def assert_payload(event, sequence):
    """Check sequence, shape, content checksum, and native/Tensor storage equality."""
    rows, columns = 2 + sequence % 4, 3 + sequence % 13
    assert int(event['sequence']) == sequence
    assert int(event['rows']) == rows and int(event['columns']) == columns
    expected = sum((sequence * 7 + (index // columns) * 13 + (index % columns) * 3) & 255
                   for index in range(rows * columns))
    assert int(event['checksum']) == expected
    assert event['native_address'] == event['tensor_address']
    assert int(event['native_address']) > 0


def assert_balanced(event, count):
    """Require every native reference and retained root reservation to be returned."""
    assert int(event['received_total']) == count
    assert int(event['received_refs']) == int(event['released_refs']) == count
    assert event['retained'] == event['accounted_bytes'] == event['queued'] == '0'


def assert_mapping(subscriber, publisher, event):
    """Require the Tensor's entire payload to lie in the publisher's read-only heap."""
    return publisher_mapping(heap_mappings(subscriber), publisher.process.pid,
                             int(event['native_address']),
                             int(event['rows']) * int(event['columns']))


def test_one_thousand_tensor_messages_match_independent_values_and_native_storage(
        tensor_peers, request):
    """Verify every acknowledged payload without cross-process address comparisons."""
    publisher, subscriber = tensor_peers.pair()
    samples = []
    for sequence in range(1000):
        sent = command(publisher, f'publish {sequence}', 'PUBLISHED')
        received = subscriber.wait_event('RX')
        assert_payload(sent, sequence)
        assert_payload(received, sequence)
        assert received['readonly'] == 'true'
        assert int(received['received']) == int(received['delivered_total']) == sequence + 1
        assert sent['accounted_bytes'] == received['root_accounted_bytes']
        assert received['dropped_count'] == received['dropped_bytes'] == '0'
        assert received['dropped_oversize'] == '0'
        assert_balanced(received, sequence + 1)
        mapping = assert_mapping(subscriber, publisher, received)
        if sequence in (0, 499, 999):
            samples.append(dict(publisher=sent, subscriber=received, mapping=mapping))
    request.node.user_properties.extend([
        ('publisher_pid', publisher.process.pid), ('subscriber_pid', subscriber.process.pid),
        ('address_samples', json.dumps(samples, sort_keys=True)),
    ])
    tensor_peers.stop()


def test_tensor_slice_detach_keeps_full_budget_through_source_release_and_close_retry(
        tensor_peers):
    """Keep only an alias through a confined write fault, admission, and close retry."""
    publisher, subscriber = tensor_peers.pair()
    command(subscriber, 'hold', 'MODE')
    sent = command(publisher, 'publish_alias 37', 'PUBLISHED')
    received = subscriber.wait_event('RX')
    assert_payload(sent, 37)
    assert_payload(received, 37)
    assert sent['rejected_alias_publish'] == 'true'
    assert received['readonly'] == 'true'
    assert_mapping(subscriber, publisher, received)
    assert received['retained'] == '1' and received['released_refs'] == '0'
    assert received['accounted_bytes'] == sent['accounted_bytes']
    assert int(received['accounted_bytes']) > int(received['rows']) - 1
    fault = command(subscriber, 'write_fault', 'FAULT')
    assert int(fault['signal']) in (signal.SIGSEGV, signal.SIGBUS)
    assert fault['preserved'] == 'true' and fault['retained'] == '1'
    assert fault['released_refs'] == '0' and fault['received_refs'] == '1'
    assert fault['accounted_bytes'] == sent['accounted_bytes']
    # Only a Tensor alias remains, yet it consumes the single root admission slot.
    command(publisher, 'publish 38', 'PUBLISHED')
    blocked = command(subscriber, 'wait 2', 'STATS')
    assert blocked['received'] == blocked['retained'] == blocked['dropped_count'] == '1'
    assert blocked['accounted_bytes'] == sent['accounted_bytes']
    assert blocked['received_refs'] == '2' and blocked['released_refs'] == '1'
    busy = command(subscriber, 'close_busy', 'BUSY')
    assert busy['preserved'] == 'true' and busy['retained'] == '1'
    assert busy['accounted_bytes'] == sent['accounted_bytes']
    assert_balanced(command(subscriber, 'release', 'RELEASED'), 2)
    assert_balanced(command(subscriber, 'close', 'CLOSED'), 2)
    tensor_peers.stop()
