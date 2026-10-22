"""
Own fresh peers for the instrumented two-process CPU checkpoint.

Subscriber --checkpoint protocol: connections -> CONNECTED; wait N -> STATS after
native receipt/references settle; spin N -> SPUN after N callbacks since recreate;
details -> DETAILS for the last callback; mode inspect|hold-lease|hold-view -> MODE;
release -> RELEASED after held-content validation; recreate -> RECREATED (counts reset);
stats -> STATS; quit -> orderly exit. Stats expose public admission/drop counters,
native receipt/release refs, callbacks/sequences and caller_tid, with no guard probes.
Native observation/address hooks still require BUILD_TESTING, and the publisher
requires the test-observation heaphook. Nonempty observe-publish adds native/NumPy
addresses; empty publish reports bytes/checksum but has no nonempty pointer proof.
"""

from pathlib import Path
import signal
import sys
import uuid

from system_runtime import command, payload_checksum, Probe, wait_connected
from checkpoint.resources import heap_mappings, publisher_mapping


PAYLOAD_LIMIT = 2 * 1024 * 1024


class Peers:
    """
    Keep exact child ownership across normal exits and deliberate SIGKILL.

    launch_pair cannot represent replacement peers or expected signal exits.
    When changing teardown, also check launch_pair and test_publisher_system.backend_pair.
    """

    def __init__(self, config, count, limit):
        self.config = config
        self.topic = f"/checkpoint_{uuid.uuid4().hex}"
        self.environment = config.child_environment(20 + uuid.uuid4().int % 180)
        self.count = count
        self.limit = limit
        self.children = []
        self.killed = set()

    def _start(self, arguments):
        child = Probe(arguments, self.environment)
        self.children.append(child)
        ready = child.wait_event("READY")
        assert int(ready["pid"]) == child.process.pid
        return child

    def subscriber(self):
        """Start public delivery without accessing G0 guard/notifier internals."""
        script = Path(__file__).parents[1] / "subscriber/guard_process.py"
        return self._start([
            sys.executable, str(script), "--topic", self.topic,
            "--count", str(self.count), "--limit", str(self.limit), "--checkpoint",
        ])

    def publisher(self):
        """Enable the existing native address/allocation observation boundary."""
        return self._start([
            sys.executable, str(self.config.publisher_process), "--topic", self.topic,
            "--depth", "256", "--max-payload-bytes", str(PAYLOAD_LIMIT), "--observe",
        ])

    def pair(self):
        """Return two connected Python peers with distinct operating-system PIDs."""
        subscriber = self.subscriber()
        publisher = self.publisher()
        assert subscriber.process.pid != publisher.process.pid
        wait_connected(subscriber)
        return publisher, subscriber

    def kill(self, child):
        """Kill only an owned, live peer and require the exact signal exit code."""
        assert child in self.children and child.process.poll() is None
        child.process.kill()
        assert child.process.wait(timeout=10) == -signal.SIGKILL
        self.killed.add(child)

    def stop(self, child):
        """Require orderly exit; fallback cleanup must not masquerade as success."""
        child.send("quit")
        assert child.process.wait(timeout=15) == 0, child._failure("normal exit failed")

    def close(self):
        """Reap every owned child, including after failed test assertions."""
        for child in reversed(self.children):
            child.shutdown()
        for child in self.children:
            expected = -signal.SIGKILL if child in self.killed else 0
            assert child.process.returncode == expected, child._failure("unexpected exit")


def publish(publisher, size, sequence):
    """Publish through direct fill, checking all nonempty publisher addresses."""
    operation = "observe-publish" if size else "publish"
    event = command(publisher, f"{operation} {size} {sequence}", "PUBLISHED")
    assert int(event["sequence"]) == sequence
    assert int(event["bytes"]) == size
    assert int(event["checksum"]) == payload_checksum(sequence, size)
    assert event["invalidated"] == "true"
    if size:
        assert event["native_address"] == event["data_addr"] == event["numpy_address"]
        assert int(event["native_address"], 16) != 0
        assert event["filling_allocations"] == "0"
    return event


def receive(subscriber, count, size, sequence, *, publisher_pid, single_arrival=False):
    """ACK content and the RO publisher mapping; compare callback pointer only without a burst."""
    stats = command(subscriber, f"spin {count}", "SPUN")
    details = command(subscriber, "details", "DETAILS")
    assert int(details["sequence"]) == sequence
    assert int(details["size"]) == size
    assert int(details["checksum"]) == payload_checksum(sequence, size)
    assert details["readonly"] == "true"
    if size:
        assert (details["native_address"] == details["view_address"]
                == details["numpy_address"])
        assert int(details["native_address"]) != 0
        assert publisher_pid != subscriber.process.pid
        details["publisher_mapping"] = publisher_mapping(
            heap_mappings(subscriber), publisher_pid, int(details["native_address"]), size,
        )
        if single_arrival:
            assert details["callback_address"] == details["native_address"]
    assert details["python_tid"] == stats["caller_tid"]
    return stats, details


def balanced(stats, received):
    """Check exact native receipt/release counts and empty admission budget."""
    assert int(stats["received_total"]) == received
    assert int(stats["received_refs"]) == int(stats["released_refs"]) == received
    assert stats["outstanding_leases"] == stats["outstanding_payload_bytes"] == "0"
    assert stats["queued_leases"] == "0"
