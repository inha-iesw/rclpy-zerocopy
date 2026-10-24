"""Opt-in typed facade interop with native layout and same-process pointer oracles."""

from contextlib import ExitStack
import os
from pathlib import Path
import sys
import uuid

import pytest

from system_runtime import command, launch_pair, preflight, system_tld_requested


@pytest.fixture(scope='module')
def config():
    """Keep system tests opt-in and require matching existing kernel/userspace artifacts."""
    if not system_tld_requested():
        pytest.skip('set RCLPY_AGNOCAST_RUN_SYSTEM_TLD=1 for typed facade TLD')
    return preflight()


@pytest.fixture
def pair_factory(config):
    """Launch one Python facade and one independent native participant per test."""
    native = Path(os.environ['RCLPY_AGNOCAST_TYPED_FACADE_NATIVE'])
    assert native.is_file() and os.access(native, os.X_OK)
    with ExitStack() as stack:
        def create(kind, direction, remap=False):
            topic = f'/typed_facade_{uuid.uuid4().hex}'
            environment = config.child_environment(20 + uuid.uuid4().int % 180)
            python_mode = 'publisher' if direction == 'python_to_cpp' else 'subscriber'
            native_mode = 'subscriber' if direction == 'python_to_cpp' else 'publisher'
            python_command = [
                sys.executable, str(Path(__file__).with_name('facade_process.py')),
                '--mode', python_mode, '--kind', kind,
            ]
            if remap:
                namespace = topic
                topic = namespace + '/delivered'
                python_command.extend([
                    '--topic', 'requested', '--namespace', namespace, '--remap', 'delivered'])
            else:
                python_command.extend(['--topic', topic])
            native_command = [str(native), native_mode, kind, topic]
            subscriber, publisher = (
                (native_command, python_command) if direction == 'python_to_cpp'
                else (python_command, native_command))
            return stack.enter_context(launch_pair(subscriber, publisher, environment))
        yield create


@pytest.mark.parametrize('direction', ['python_to_cpp', 'cpp_to_python'])
@pytest.mark.parametrize('kind', ['Image', 'PointCloud2', 'Containers'])
def test_public_facade_roundtrip_matches_native_fields_capacity_and_view_pointer(
        pair_factory, kind, direction):
    """Validate both directions without using cross-process address equality as proof."""
    publisher, subscriber = pair_factory(kind, direction)
    published = command(publisher, 'publish', 'PUBLISHED')
    received = subscriber.wait_event('RX')
    assert published['valid'] == received['valid'] == 'true'
    assert received['count'] == '1'
    assert published['accounted_bytes'] == received['accounted_bytes']
    python_observation = published if direction == 'python_to_cpp' else received
    assert python_observation['pointer_equal'] == 'true'
    if direction == 'cpp_to_python':
        released = command(subscriber, 'release', 'RELEASED')
        assert released['retained'] == released['accounted_bytes'] == '0'


def test_received_one_byte_alias_preserves_full_root_budget_until_release_and_close_retry(
        pair_factory):
    """A derived readonly view keeps native storage and charge through failed close."""
    publisher, subscriber = pair_factory('Containers', 'cpp_to_python')
    published = command(publisher, 'publish', 'PUBLISHED')
    subscriber.wait_event('RX')
    stats = command(subscriber, 'stats', 'STATS')
    assert stats['retained'] == '1'
    assert stats['alias_bytes'] == '1'
    assert stats['accounted_bytes'] == published['accounted_bytes']
    assert int(stats['accounted_bytes']) > 1
    assert command(subscriber, 'close_busy', 'BUSY')['preserved'] == 'true'
    stats = command(subscriber, 'stats', 'STATS')
    assert stats['retained'] == '1'
    assert stats['accounted_bytes'] == published['accounted_bytes']
    assert command(subscriber, 'release', 'RELEASED')['accounted_bytes'] == '0'
    command(subscriber, 'close', 'CLOSED')


@pytest.mark.parametrize('direction', ['python_to_cpp', 'cpp_to_python'])
def test_node_namespace_and_relative_remap_deliver_on_resolved_native_topic(
        pair_factory, direction):
    """A native peer on the resolved absolute name receives the facade's remapped traffic."""
    publisher, subscriber = pair_factory('Image', direction, remap=True)
    published = command(publisher, 'publish', 'PUBLISHED')
    received = subscriber.wait_event('RX')
    assert published['valid'] == received['valid'] == 'true'
    assert received['count'] == '1'
    assert published['accounted_bytes'] == received['accounted_bytes']
    if direction == 'cpp_to_python':
        command(subscriber, 'release', 'RELEASED')
