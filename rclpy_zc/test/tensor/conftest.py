"""CPU tensor fixtures use real owners and isolated POSIX shared storage."""

import gc
import subprocess
import sys
import threading
from types import SimpleNamespace

import pytest

from rclpy_agnocast import _native
from rclpy_agnocast.types.publisher import TypedPublisher


@pytest.fixture
def torch():
    """Load the optional consumer only for tests that exercise Torch storage."""
    return pytest.importorskip('torch')


@pytest.fixture
def publisher():
    """Own a native publisher and verify worker cleanup after each test."""
    assert _native._test_tensor_live_managers() == 0
    handle = _native._TestTypedPublisher(65536)
    runtime = SimpleNamespace(_lifecycle_lock=threading.RLock())
    yield TypedPublisher(handle, runtime), handle
    gc.collect()
    handle.wait_idle()
    handle.close()
    trace = handle.inspect()
    assert trace['creates'] == trace['destroys'] == 1
    assert trace['worker_only']
    assert _native._test_tensor_live_managers() == 0


@pytest.fixture
def shared_file():
    """Another process owns the filename and removes it after all mappings close."""
    processes = []

    def create(*, recognized=True):
        script = '''
import os
import struct
import sys
if sys.argv[1] == 'recognized':
    from rclpy_agnocast import _native
    name = _native._test_agnocast_shm_name(os.getpid())
else:
    name = '/unknown@' + str(os.getpid())
path = '/dev/shm' + name
try:
    with open(path, 'xb') as output:
        output.write(struct.pack('=6i', *range(10, 16)))
    print(name, flush=True)
    sys.stdin.readline()
finally:
    if os.path.exists(path):
        os.unlink(path)
'''
        process = subprocess.Popen(
            [sys.executable, '-c', script, 'recognized' if recognized else 'unknown'],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        processes.append(process)
        name = process.stdout.readline().strip()
        assert name, process.stderr.read()
        return name

    yield create
    for process in processes:
        process.communicate('\n', timeout=5)
        assert process.returncode == 0


@pytest.fixture
def subscription(shared_file):
    """Map another process's storage and require all native owners to close."""
    subscriptions = []

    def create(*, count=4, retained=1000000, maximum=1000000, writable=False, name=None):
        endpoint = _native._TestTensorSubscription(
            name or shared_file(), count, retained, maximum, writable)
        subscriptions.append(endpoint)
        return endpoint

    yield create
    gc.collect()
    for endpoint in subscriptions:
        endpoint.clear_pending()
        endpoint.close()
    assert _native._test_tensor_live_managers() == 0
