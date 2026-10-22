"""Opt-in generated-provider TLD with an independent C++ payload and capacity oracle."""

from contextlib import ExitStack
import os
from pathlib import Path
import sys
import uuid

import pytest

from system_runtime import command, launch_pair, preflight, system_tld_requested


@pytest.fixture(scope='module')
def config():
    """Require the exact already-loaded module and matching heaphook artifacts."""
    if not system_tld_requested():
        pytest.skip('set RCLPY_AGNOCAST_RUN_SYSTEM_TLD=1 for typed factory TLD')
    return preflight()


@pytest.fixture
def pair_factory(config):
    """Create separately mapped native and Python participants for one fixture type."""
    inspector = Path(os.environ['RCLPY_AGNOCAST_TYPED_INSPECTOR'])
    assert inspector.is_file() and os.access(inspector, os.X_OK)
    with ExitStack() as stack:
        def create(kind):
            topic = f'/typed_fixture_{uuid.uuid4().hex}'
            environment = config.child_environment(20 + uuid.uuid4().int % 180)
            publisher = [
                sys.executable, str(Path(__file__).with_name('process.py')),
                '--topic', topic, '--kind', kind,
            ]
            return stack.enter_context(launch_pair(
                [str(inspector), kind, topic], publisher, environment))
        yield create


@pytest.mark.parametrize('kind', ['Scalars', 'Containers', 'Nested'])
def test_generated_factory_direct_fill_matches_native_fields_and_capacity(pair_factory, kind):
    """Verify every field using native ROS layout and independently summed retained storage."""
    publisher, subscriber = pair_factory(kind)
    published = command(publisher, 'publish', 'PUBLISHED')
    received = subscriber.wait_event('RX')
    assert received['valid'] == 'true'
    assert received['count'] == '1'
    assert published['accounted_bytes'] == received['accounted_bytes']
    assert published['data_address'] == received['data_address']
    assert published['root_address'] == received['root_address']
    assert published['active_loans'] == '0'
    assert command(subscriber, 'stats', 'STATS')['received'] == '1'


@pytest.mark.parametrize('case', ['bounds', 'lifetime', 'allocation'])
def test_structural_and_terminal_failures_recover_without_native_publish(pair_factory, case):
    """Use real factory exceptions, aliases and cleanup with zero native deliveries."""
    publisher, subscriber = pair_factory('Containers')
    checked = command(publisher, case, 'CHECKED')
    assert checked['case'] == case
    assert checked['active_loans'] == '0'
    assert command(subscriber, 'stats', 'STATS')['received'] == '0'
