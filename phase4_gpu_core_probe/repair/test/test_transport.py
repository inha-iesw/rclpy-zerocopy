"""Run real repaired-core GPU peers with typed Agnocast descriptors."""

import os
from pathlib import Path
import queue
import signal
import socket
import subprocess
import threading
import time

import pytest


pytestmark = pytest.mark.skipif(
    os.environ.get('G3_RUN_TRANSPORT') != '1',
    reason='requires the verified Agnocast module and ARM64 heaphook')

EVENTS = {'READY', 'PUB', 'FULL', 'AVAILABLE', 'CONNECTIONS', 'CLOSED', 'STASHED',
          'MAPPED', 'REJECTED', 'DROPPED', 'DONE', 'REMAPPED', 'LIMITS_REJECTED',
          'LIMITS_ACCEPTED', 'FAST_DONE', 'RESOURCES', 'MISSING', 'UNEXPECTED_MAP',
          'PENDING', 'COMPLETE', 'PRODUCED', 'POISONED', 'CLOSING', 'READ_CLOSED', 'FAULT_STATS',
          'FD_REPLY_STATS', 'RECEIPT_STATS', 'VMM_STATS'}


class Peer:
    def __init__(self, binary, role, topic, limit, directory, fault=None, reply_fault=None):
        self.role = role
        self.stopped = False
        self.descriptors = set()
        env = os.environ.copy()
        env.update(AGNOCAST_DEVICE_PATH='/dev/agnocast_rclpy', AGNOCAST_BRIDGE_MODE='off',
                   AGNOCAST_NO_DISCOVERY_AGENT='1')
        if fault is not None:
            env['G3_IMPORT_FAILURE'] = fault
        if reply_fault is not None:
            env['G3_FD_REPLY_FAILURE'] = reply_fault
        hook = Path(__file__).resolve().parents[3] / 'build/g2-device-arm64/lib'
        env['LD_PRELOAD'] = str(hook / 'libagnocast_heaphook.so')
        env['LD_LIBRARY_PATH'] = str(hook) + ':' + env.get('LD_LIBRARY_PATH', '')
        self.stderr = (directory / (role + '.stderr')).open('w')
        self.stdout = (directory / (role + '.stdout')).open('w')
        self.events = queue.Queue()
        self.process = subprocess.Popen(
            [str(binary), role, topic, str(limit)], env=env, stdin=subprocess.PIPE,
            stdout=subprocess.PIPE, stderr=self.stderr, text=True, bufsize=1)
        self.reader = threading.Thread(target=self.collect, daemon=True)
        self.reader.start()
        try:
            assert self.receive()[0] == 'READY'
        except BaseException:
            self.stop()
            raise

    def collect(self):
        for line in self.process.stdout:
            self.stdout.write(line)
            self.stdout.flush()
            tokens = line.split()
            if tokens and tokens[0] in EVENTS:
                if self.role == 'publisher' and tokens[0] == 'PUB':
                    self.descriptors.add((tokens[2], tokens[4]))
                self.events.put(tokens)
        self.events.put(None)

    def receive(self):
        event = self.events.get(timeout=8)
        assert event is not None, f'peer exited with {self.process.poll()}'
        return event

    def ask(self, command):
        self.process.stdin.write(command + '\n')
        self.process.stdin.flush()
        return self.receive()

    def stop(self):
        if self.stopped:
            return
        if self.process.poll() is None:
            self.process.terminate()
        try:
            self.process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait(timeout=5)
        self.process.stdin.close()
        self.reader.join(timeout=2)
        # SIGKILL leaves named IPC objects; remove only this dead test peer's names.
        for block, address in self.descriptors:
            Path(f'/dev/shm/rclpy_zc_gpu_v1_{self.process.pid}_{block}').unlink(missing_ok=True)
            Path(address).unlink(missing_ok=True)
        self.process.stdout.close()
        self.stdout.close()
        self.stderr.close()
        self.stopped = True


@pytest.fixture
def peers(tmp_path):
    binary = Path(os.environ['G3_TRANSPORT_PEER'])
    active = []

    def start(blocks=1, reads=1, fault=None, reply_fault=None):
        topic = f'/g3_repair_{os.getpid()}_{time.monotonic_ns()}'
        directory = tmp_path / str(len(active))
        directory.mkdir()
        publisher_binary = Path(os.environ['G3_CLEANUP_PEER']) if reply_fault else binary
        publisher = Peer(publisher_binary, 'publisher', topic, blocks, directory,
                         reply_fault=reply_fault)
        active.append(publisher)
        subscriber_binary = Path(os.environ['G3_CLEANUP_PEER']) if fault else binary
        subscriber = Peer(subscriber_binary, 'subscriber', topic, reads, directory, fault)
        active.append(subscriber)
        deadline = time.monotonic() + 3
        while publisher.ask('connections') != ['CONNECTIONS', '1']:
            assert time.monotonic() < deadline
            time.sleep(0.01)
        return publisher, subscriber

    yield start
    for peer in reversed(active):
        peer.stop()


def publish(publisher, value):
    deadline = time.monotonic() + 3
    while True:
        event = publisher.ask(f'publish {value}')
        if event[0] == 'PUB':
            return event
        assert event == ['FULL'] and time.monotonic() < deadline
        time.sleep(0.005)


def close(publisher, subscriber):
    subscriber.process.stdin.write('quit\n')
    subscriber.process.stdin.flush()
    assert subscriber.process.wait(timeout=5) == 0
    deadline = time.monotonic() + 3
    while publisher.ask('close') != ['CLOSED', '1']:
        assert time.monotonic() < deadline
        time.sleep(0.005)
    assert publisher.process.wait(timeout=5) == 0


def test_descriptor_release_does_not_release_pending_gpu_read(peers):
    publisher, subscriber = peers()
    first = publish(publisher, 17)
    assert subscriber.ask('take')[:2] == ['MAPPED', first[1]]
    assert subscriber.ask('drop') == ['DROPPED']
    time.sleep(0.15)
    assert subscriber.ask('pending') == ['PENDING']
    assert publisher.ask('probe') == ['FULL']
    assert publisher.ask('close') == ['CLOSED', '0']
    assert subscriber.ask('finish') == ['DONE', str(17 * 64)]
    second = publish(publisher, 23)
    assert second[1] != first[1]
    assert second[2] == first[2]
    assert subscriber.ask('take')[:2] == ['MAPPED', second[1]]
    assert subscriber.ask('finish') == ['DONE', str(23 * 64)]
    close(publisher, subscriber)


def test_stale_descriptor_cannot_read_recycled_generation(peers):
    publisher, subscriber = peers()
    first = publish(publisher, 31)
    assert subscriber.ask('stash') == ['STASHED', first[1]]
    time.sleep(0.15)
    second = publish(publisher, 47)
    assert second[1] != first[1]
    assert second[2] == first[2]
    assert subscriber.ask('map') == ['REJECTED']
    assert subscriber.ask('take')[:2] == ['MAPPED', second[1]]
    assert subscriber.ask('finish') == ['DONE', str(47 * 64)]
    close(publisher, subscriber)


def test_ipc_ready_event_orders_pending_producer_before_gpu_consumer(peers):
    publisher, subscriber = peers()
    packet = publisher.ask('publish_pending 61')
    assert packet[0] == 'PUB'
    assert subscriber.ask('stash') == ['STASHED', packet[1]]
    assert subscriber.ask('map_only')[:2] == ['MAPPED', packet[1]]
    # This stream has only the core's ready-event wait, with no consumer gate.
    time.sleep(0.15)
    assert subscriber.ask('pending') == ['PENDING']
    assert publisher.ask('probe') == ['FULL']
    assert publisher.ask('close') == ['CLOSED', '0']
    assert publisher.ask('release') == ['PRODUCED']
    assert subscriber.ask('finish') == ['DONE', str(61 * 64)]
    close(publisher, subscriber)


def test_mapper_recreation_cannot_bypass_live_admission(peers):
    publisher, subscriber = peers()
    publish(publisher, 11)
    assert subscriber.ask('take')[0] == 'MAPPED'
    assert subscriber.ask('remap') == ['REMAPPED']
    assert subscriber.ask('map') == ['REJECTED']
    assert subscriber.ask('wrong_limits') == ['LIMITS_REJECTED']
    assert subscriber.ask('finish') == ['DONE', str(11 * 64)]
    close(publisher, subscriber)


def test_subscriber_close_seals_recreated_mappers_until_gpu_read_drains(peers):
    publisher, subscriber = peers()
    publish(publisher, 37)
    assert subscriber.ask('take')[0] == 'MAPPED'
    assert subscriber.ask('close_read') == ['READ_CLOSED', '0']
    assert subscriber.ask('drop') == ['DROPPED']
    assert subscriber.ask('pending') == ['PENDING']
    assert subscriber.ask('remap') == ['REMAPPED']
    assert subscriber.ask('map_only') == ['CLOSING']
    assert subscriber.ask('close_read') == ['READ_CLOSED', '0']
    assert publisher.ask('close') == ['CLOSED', '0']
    assert subscriber.ask('finish') == ['DONE', str(37 * 64)]
    deadline = time.monotonic() + 3
    while subscriber.ask('close_read') != ['READ_CLOSED', '1']:
        assert time.monotonic() < deadline
        time.sleep(0.005)
    assert subscriber.ask('close_read') == ['READ_CLOSED', '1']
    assert subscriber.ask('map_only') == ['CLOSING']
    # Only after every old mapper/owner is gone may a new epoch admit reads.
    assert subscriber.ask('remap') == ['REMAPPED']
    publish(publisher, 41)
    assert subscriber.ask('take')[0] == 'MAPPED'
    assert subscriber.ask('finish') == ['DONE', str(41 * 64)]
    close(publisher, subscriber)


def test_completed_read_is_released_behind_stalled_read(peers):
    publisher, subscriber = peers(reads=2)
    publish(publisher, 13)
    assert subscriber.ask('take')[0] == 'MAPPED'
    assert subscriber.ask('drop') == ['DROPPED']
    assert subscriber.ask('pending') == ['PENDING']
    assert subscriber.ask('fastmap') == ['FAST_DONE']
    deadline = time.monotonic() + 3
    while subscriber.ask('map')[0] != 'MAPPED':
        assert time.monotonic() < deadline
        time.sleep(0.005)
    assert subscriber.ask('map') == ['REJECTED']
    assert subscriber.ask('pending') == ['PENDING']
    assert subscriber.ask('finish') == ['DONE', str(13 * 64)]
    close(publisher, subscriber)


def test_import_failures_do_not_accumulate_fds_or_metadata_maps(peers):
    publisher, subscriber = peers()
    publish(publisher, 19)
    assert subscriber.ask('stash')[0] == 'STASHED'
    assert subscriber.ask('bad_event') == ['REJECTED']
    before = list(map(int, subscriber.ask('resources')[1:]))
    for _ in range(50):
        assert subscriber.ask('bad_event') == ['REJECTED']
        assert subscriber.ask('oversize') == ['REJECTED']
    after = list(map(int, subscriber.ask('resources')[1:]))
    assert after == before
    assert subscriber.ask('take') == ['MISSING']
    # The same cached descriptor remains importable after ordinary errors.
    assert subscriber.ask('map')[0] == 'MAPPED'
    assert subscriber.ask('finish') == ['DONE', str(19 * 64)]
    close(publisher, subscriber)


@pytest.mark.parametrize('fault', ['recv_error', 'recv_eof'])
def test_connected_fd_request_failure_releases_admission_and_allows_import(peers, fault):
    publisher, subscriber = peers(fault=fault)
    publish(publisher, 47)
    assert subscriber.ask('stash')[0] == 'STASHED'
    before = subscriber.ask('resources')
    for attempt in range(1, 4):
        assert subscriber.ask('map_only') == ['REJECTED']
        stats = subscriber.ask('fault_stats')
        assert list(map(int, stats[1:4])) == [attempt, 0, 0]
        assert list(map(int, stats[5:])) == [attempt, attempt, 0]
        assert subscriber.ask('resources') == before
    # H=1: three real connected requests and this successful import prove that
    # failed requests return admission. Normal peer close also checks read refs.
    assert subscriber.ask('map')[0] == 'MAPPED'
    assert subscriber.ask('finish') == ['DONE', str(47 * 64)]
    stats = subscriber.ask('fault_stats')
    assert list(map(int, stats[1:4])) == [3, 0, 1]
    assert list(map(int, stats[5:])) == [4, 4, 0]
    close(publisher, subscriber)
    diagnostics = Path(subscriber.stderr.name).read_text()
    if fault == 'recv_eof':
        assert diagnostics.count('publisher closed FD request without data') == 3
        assert 'recvmsg on ' not in diagnostics
    else:
        assert diagnostics.count('recvmsg on ') == 3
        assert 'publisher closed FD request without data' not in diagnostics


RECEIPT_FIELDS = ('fds', 'cloexec', 'closed', 'context_restored', 'retains', 'pushes',
                  'pops', 'releases', 'imports', 'refs', 'lifetime_retains',
                  'lifetime_pushes', 'lifetime_pops', 'lifetime_releases')
VMM_FIELDS = ('imported', 'granularities', 'reserves', 'maps', 'accesses', 'unmaps',
             'address_frees', 'allocation_releases', 'event_opens', 'fd_closes')
FAULT_FIELDS = ('failures', 'cleanup_calls', 'imports', 'retains', 'connections',
                'receives', 'poisoned')


def parse_stats(event, tag, fields):
    assert len(event) == len(fields) + 1, (tag, event)
    assert event[0] == tag, event
    return dict(zip(fields, map(int, event[1:])))


# Unspecified expected fields are zero, including every VMM stage after an
# unsuccessful import. These dictionaries are never modified by the test.
NO_VMM = {'fd_closes': 1}
IMPORT_ROLLBACK = {'retains': 1, 'pushes': 2, 'pops': 2, 'releases': 1, 'imports': 1}
RECEIPT_CASES = [
    ('receipt_retain', {'retains': 1}, NO_VMM, 'Failed to retain CUDA primary context:'),
    ('receipt_push', {'retains': 1, 'releases': 1}, NO_VMM,
     'Failed to activate CUDA context:'),
    ('receipt_import', IMPORT_ROLLBACK, NO_VMM, 'cuMemImportFromShareableHandle failed:'),
]
RECEIPT_DIAGNOSTICS = tuple(diagnostic for _, _, _, diagnostic in RECEIPT_CASES)


@pytest.mark.parametrize('fault,rollback,vmm_rollback,diagnostic', RECEIPT_CASES)
def test_received_vmm_fd_cuda_failure_returns_resources_and_allows_import(
        peers, fault, rollback, vmm_rollback, diagnostic):
    publisher, subscriber = peers(fault=fault)
    packet = publish(publisher, 59)
    assert subscriber.ask('stash') == ['STASHED', packet[1]]
    before = (settled_resources(publisher), settled_resources(subscriber))
    for attempt in range(1, 4):
        assert subscriber.ask('map_only') == ['REJECTED']
        receipt = parse_stats(subscriber.ask('receipt_stats'), 'RECEIPT_STATS', RECEIPT_FIELDS)
        # Later driver work can reuse the FD number. Observe its actual close
        # independently of map-return EBADF and the settled FD count below.
        vmm = parse_stats(subscriber.ask('vmm_stats'), 'VMM_STATS', VMM_FIELDS)
        assert vmm == dict.fromkeys(VMM_FIELDS, 0) | vmm_rollback
        # Context restored, successful pushes/pops balanced, no read ref remains.
        expected = dict.fromkeys(RECEIPT_FIELDS, 0) | rollback | {
            'fds': 1, 'cloexec': 1, 'context_restored': 1,
            'lifetime_retains': attempt * rollback.get('releases', 0),
            'lifetime_pushes': attempt * rollback.get('pushes', 0),
            'lifetime_pops': attempt * rollback.get('pops', 0),
            'lifetime_releases': attempt * rollback.get('releases', 0)}
        if vmm_rollback.get('imported', 0) == 0:
            expected['closed'] = 1  # Rejection before a successful import: EBADF.
        else:
            del receipt['closed']
            del expected['closed']
        assert receipt == expected
        stats = parse_stats(subscriber.ask('fault_stats'), 'FAULT_STATS', FAULT_FIELDS)
        assert stats['failures'] == attempt and stats['cleanup_calls'] == 0
        assert stats['imports'] == attempt * rollback.get('imports', 0)
        assert stats['connections'] == stats['receives'] == attempt
        assert stats['poisoned'] == 0
        assert (settled_resources(publisher), settled_resources(subscriber)) == before
    # Same UID and H=1 mapper, with no republish or epoch recreation.
    assert subscriber.ask('map') == ['MAPPED', packet[1], '1']
    mapped = parse_stats(subscriber.ask('receipt_stats'), 'RECEIPT_STATS', RECEIPT_FIELDS)
    # Driver work after successful import may reuse the now-closed FD number.
    # Check settled FD counts below, rather than requiring that number be EBADF.
    for name, value in {'fds': 1, 'cloexec': 1, 'context_restored': 1, 'retains': 1,
                        'pushes': 3, 'pops': 3, 'releases': 0, 'imports': 1, 'refs': 1}.items():
        assert mapped[name] == value, (name, mapped)
    assert parse_stats(subscriber.ask('vmm_stats'), 'VMM_STATS', VMM_FIELDS) == {
        'imported': 1, 'granularities': 1, 'reserves': 1, 'maps': 1, 'accesses': 1,
        'unmaps': 0, 'address_frees': 0, 'allocation_releases': 0, 'event_opens': 1, 'fd_closes': 1}
    assert subscriber.ask('finish') == ['DONE', str(59 * 64)]
    stats = parse_stats(subscriber.ask('fault_stats'), 'FAULT_STATS', FAULT_FIELDS)
    assert stats['failures'] == 3 and stats['cleanup_calls'] == 0
    assert stats['imports'] == 1 + 3 * rollback.get('imports', 0)
    assert stats['connections'] == stats['receives'] == 4
    assert stats['poisoned'] == 0
    deadline = time.monotonic() + 3
    while subscriber.ask('close_read') != ['READ_CLOSED', '1']:
        assert time.monotonic() < deadline
        time.sleep(0.005)
    drained = parse_stats(subscriber.ask('receipt_stats'), 'RECEIPT_STATS', RECEIPT_FIELDS)
    assert drained['refs'] == 0
    assert drained['lifetime_retains'] == drained['lifetime_releases'] == (
        1 + 3 * rollback.get('releases', 0))
    assert drained['lifetime_pushes'] == drained['lifetime_pops']
    # Worker polling adds balanced push/pop pairs; the minimum is not an exact count.
    assert drained['lifetime_pushes'] >= 7 + 3 * rollback.get('pushes', 0)
    assert (settled_resources(publisher), settled_resources(subscriber)) == before
    close(publisher, subscriber)
    diagnostics = Path(subscriber.stderr.name).read_text()
    assert diagnostics.count(diagnostic) == 3
    for other in RECEIPT_DIAGNOSTICS:
        if other != diagnostic:
            assert other not in diagnostics


@pytest.mark.parametrize('fault', [
    'unmap', 'address_free', 'release', 'event_destroy', 'import_pop',
    'partial_unmap', 'partial_pop'])
def test_import_cleanup_failure_retains_reference_and_blocks_new_imports(peers, fault):
    publisher, subscriber = peers(fault=fault)
    publish(publisher, 43)
    assert subscriber.ask('stash')[0] == 'STASHED'
    mapped = subscriber.ask('map_only')
    if fault.startswith('partial_') or fault == 'import_pop':
        assert mapped == ['POISONED']
    else:
        assert mapped[0] == 'MAPPED'
        assert subscriber.ask('drop') == ['DROPPED']
    deadline = time.monotonic() + 3
    while True:
        before = subscriber.ask('fault_stats')
        if before[-1] == '1':
            break
        assert time.monotonic() < deadline
        time.sleep(0.005)
    assert before[1] == '1'  # One actual injected cleanup failure.
    assert int(before[2]) == (2 if fault == 'partial_pop' else 1)
    assert int(before[3]) == 1  # A real VMM import preceded cleanup.
    assert int(before[4]) >= 1
    assert subscriber.ask('close_read') == ['READ_CLOSED', '0']
    assert publisher.ask('close') == ['CLOSED', '0']
    for _ in range(20):
        assert subscriber.ask('remap') == ['REMAPPED']
        assert subscriber.ask('map_only') == ['POISONED']
        assert subscriber.ask('close_read') == ['READ_CLOSED', '0']
        assert publisher.ask('probe') == ['FULL']
    assert subscriber.ask('fault_stats') == before
    # These poisoned peers require exit; fixture cleanup records their own names.


def test_fd_request_disconnect_does_not_terminate_publisher(peers):
    publisher, subscriber = peers()
    packet = publish(publisher, 29)
    for _ in range(50):
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as request:
            request.connect(packet[4])
    assert publisher.ask('connections') == ['CONNECTIONS', '1']
    assert subscriber.ask('take')[0] == 'MAPPED'
    assert subscriber.ask('finish') == ['DONE', str(29 * 64)]
    close(publisher, subscriber)


@pytest.mark.parametrize('fault', ['two_fds', 'truncated', 'send_error'])
def test_invalid_fd_reply_releases_resources_and_allows_real_import(peers, fault):
    publisher, subscriber = peers(reply_fault=fault)
    publish(publisher, 43)
    assert subscriber.ask('stash')[0] == 'STASHED'
    before = (settled_resources(publisher), settled_resources(subscriber))
    for attempt in range(1, 4):
        assert subscriber.ask('map_only') == ['REJECTED']
        # Settled publisher FDs also establish that its send thread closed the
        # accepted socket after recording the send result.
        assert (settled_resources(publisher), settled_resources(subscriber)) == before
        sent = 0 if fault == 'send_error' else attempt
        assert publisher.ask('fd_reply_stats') == [
            'FD_REPLY_STATS', str(attempt), str(attempt), str(sent)]
    # Keep the same published VMM FD, descriptor and H=1 mapper for recovery.
    assert subscriber.ask('map')[0] == 'MAPPED'
    assert subscriber.ask('finish') == ['DONE', str(43 * 64)]
    assert publisher.ask('fd_reply_stats') == [
        'FD_REPLY_STATS', '4', '3', '0' if fault == 'send_error' else '3']
    assert (settled_resources(publisher), settled_resources(subscriber)) == before
    close(publisher, subscriber)
    if fault == 'send_error':
        diagnostics = Path(subscriber.stderr.name).read_text()
        assert diagnostics.count('publisher closed FD request without data') == 3
        assert 'recvmsg on ' not in diagnostics


def test_gpu_consumer_crash_strands_fixed_slot_until_peer_restart(peers):
    publisher, subscriber = peers()
    publish(publisher, 53)
    assert subscriber.ask('take')[0] == 'MAPPED'
    assert subscriber.ask('pending') == ['PENDING']
    subscriber.process.kill()
    assert subscriber.process.wait(timeout=5) == -signal.SIGKILL
    time.sleep(0.15)
    before = publisher.ask('resources')
    for _ in range(20):
        assert publisher.ask('probe') == ['FULL']
        assert publisher.ask('close') == ['CLOSED', '0']
    assert publisher.ask('resources') == before
    publisher.process.kill()
    assert publisher.process.wait(timeout=5) == -signal.SIGKILL
    # Recovery is a fresh pair of GPU processes, not dead-reader reclamation.
    publisher, subscriber = peers()
    publish(publisher, 59)
    assert subscriber.ask('take')[0] == 'MAPPED'
    assert subscriber.ask('finish') == ['DONE', str(59 * 64)]
    close(publisher, subscriber)


def test_repeated_peer_restart_cleans_only_recorded_names_and_preserves_other_pair(peers):
    healthy_publisher, healthy_subscriber = peers()
    for cycle in range(3):
        publisher, subscriber = peers()
        packet = publish(publisher, 67 + cycle)
        assert subscriber.ask('take')[0] == 'MAPPED'
        subscriber.process.kill()
        assert subscriber.process.wait(timeout=5) == -signal.SIGKILL
        assert publisher.ask('close') == ['CLOSED', '0']
        assert publisher.ask('probe') == ['FULL']
        # An unrelated pair remains usable while this pair requires restart.
        publish(healthy_publisher, 71 + cycle)
        assert healthy_subscriber.ask('take')[0] == 'MAPPED'
        assert healthy_subscriber.ask('finish') == ['DONE', str((71 + cycle) * 64)]
        publisher.process.kill()
        assert publisher.process.wait(timeout=5) == -signal.SIGKILL
        metadata = Path(f'/dev/shm/rclpy_zc_gpu_v1_{publisher.process.pid}_{packet[2]}')
        address = Path(packet[4])
        assert metadata.exists() and address.exists()
        subscriber.stop()
        publisher.stop()
        assert not metadata.exists() and not address.exists()
    close(healthy_publisher, healthy_subscriber)


def settled_resources(peer):
    deadline = time.monotonic() + 3
    previous = None
    stable = 0
    while stable < 3:
        current = peer.ask('resources')
        stable = stable + 1 if current == previous else 0
        previous = current
        assert time.monotonic() < deadline
        time.sleep(0.01)
    return current


def test_thousand_gpu_deliveries_preserve_data_and_resource_counts(
        peers, record_testsuite_property):
    publisher, subscriber = peers(blocks=32)
    generations = {}
    before = None
    for index in range(1064):
        value = index % 251 + 1
        packet = publish(publisher, value)
        old_uid = generations.get(packet[2])
        if old_uid is not None:
            assert int(packet[1]) > old_uid
        generations[packet[2]] = int(packet[1])
        event = subscriber.ask('take')
        deadline = time.monotonic() + 3
        while event == ['REJECTED']:
            assert time.monotonic() < deadline
            time.sleep(0.005)
            event = subscriber.ask('map')
        assert event[:2] == ['MAPPED', packet[1]]
        assert subscriber.ask('finish') == ['DONE', str(value * 64)]
        if index == 63:
            before = (settled_resources(publisher), settled_resources(subscriber))
    assert len(generations) == 32
    after = (settled_resources(publisher), settled_resources(subscriber))
    assert after == before
    assert after[0][2] == '32'  # Positive control for the metadata-name oracle.
    assert after[1][2] == '0'  # No idle imported metadata mappings.
    record_testsuite_property('checkpoint_warmup_deliveries', 64)
    record_testsuite_property('checkpoint_measured_deliveries', 1000)
    record_testsuite_property('checkpoint_resources_before', repr(before))
    record_testsuite_property('checkpoint_resources_after', repr(after))
    close(publisher, subscriber)
