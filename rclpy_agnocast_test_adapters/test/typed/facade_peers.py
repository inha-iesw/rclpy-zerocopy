"""Exact typed peer ownership across orderly, signal, and crash replacement tests."""

from pathlib import Path
import signal
import sys
import uuid

from system_runtime import command, Probe, wait_connected


class TypedPeers:
    """Track expected exits without treating fallback SIGKILL cleanup as a success."""

    def __init__(self, config):
        self.config = config
        self.topic = f'/typed_lifecycle_{uuid.uuid4().hex}'
        self.environment = config.child_environment(20 + uuid.uuid4().int % 180)
        self.children = []
        self.killed = set()

    def start(self, mode):
        """Start one public Node endpoint using the independent native layout oracle."""
        child = Probe([
            sys.executable, str(Path(__file__).with_name('facade_process.py')),
            '--mode', mode, '--kind', 'Containers', '--topic', self.topic,
        ], self.environment)
        self.children.append(child)
        assert int(child.wait_event('READY')['pid']) == child.process.pid
        return child

    def pair(self):
        """Connect two Python processes before admitting any payload."""
        subscriber = self.start('subscriber')
        publisher = self.start('publisher')
        wait_connected(subscriber)
        return publisher, subscriber

    def stop(self, child, mode='normal'):
        """Require actual signal observation and cleanup from exactly the owned peer."""
        assert child in self.children and child.process.poll() is None
        if mode == 'sigkill':
            child.process.kill()
            assert child.process.wait(timeout=10) == -signal.SIGKILL
            self.killed.add(child)
            return
        if mode == 'normal':
            child.send('quit')
        else:
            command(child, 'await_signal', 'SIGNAL_READY')
            signum = {'sigint': signal.SIGINT, 'sigterm': signal.SIGTERM}[mode]
            child.process.send_signal(signum)
            result = child.wait_event('SIGNAL')
            assert result['context_shutdown'] == 'true'
            assert result['keyboard_interrupt'] == ('true' if mode == 'sigint' else 'false')
        child.wait_event('STOPPED')
        assert child.process.wait(timeout=15) == 0, child._failure('orderly exit failed')

    def close(self):
        """Reap all original and replacement children even when an assertion failed."""
        for child in reversed(self.children):
            child.shutdown()
        for child in self.children:
            expected = -signal.SIGKILL if child in self.killed else 0
            assert child.process.returncode == expected, child._failure('unexpected exit')


def transfer(publisher, subscriber, count, *, release=True):
    """Require independently checked fields and same-process pointers at both peers."""
    published = command(publisher, 'publish', 'PUBLISHED')
    received = subscriber.wait_event('RX')
    assert published['valid'] == received['valid'] == 'true'
    assert published['pointer_equal'] == received['pointer_equal'] == 'true'
    assert published['accounted_bytes'] == received['accounted_bytes']
    assert int(received['count']) == count
    if release:
        assert command(subscriber, 'release', 'RELEASED')['accounted_bytes'] == '0'
    return received
