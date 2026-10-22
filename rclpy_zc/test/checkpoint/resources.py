"""Read owned Agnocast resources and the kernel's domain-scoped endpoint graph."""

import ctypes
import fcntl
import os
from pathlib import Path
import time


class _Name(ctypes.Structure):
    _fields_ = [("pointer", ctypes.c_void_p), ("length", ctypes.c_uint64)]


class _Query(ctypes.Structure):
    _fields_ = [("topic", _Name), ("entries", ctypes.c_uint64),
                ("capacity", ctypes.c_uint32), ("domain", ctypes.c_uint32)]


class _Result(ctypes.Structure):
    _fields_ = [("node", ctypes.c_char * 256), ("depth", ctypes.c_uint32),
                ("transient", ctypes.c_bool), ("reliable", ctypes.c_bool),
                ("bridge", ctypes.c_bool)]


def endpoint_count(peers, kind):
    """Query the current fork's CLI ioctl ABI without registering a participant."""
    # Mirrors agnocast_kmod/agnocast.h ioctl_topic_info_args/topic_info_ret.
    # Recheck this ABI when updating the fork.
    # The installed ioctl wrapper hardcodes /dev/agnocast, which may belong to another module.
    # Use only the device selected by exact-artifact preflight for this fork.
    assert ctypes.sizeof(ctypes.c_void_p) == 8
    assert ctypes.sizeof(_Query) == 32 and ctypes.sizeof(_Result) == 264
    number = {"subscriber": 21, "publisher": 22}[kind]
    topic = peers.topic.encode()
    name = ctypes.create_string_buffer(topic)
    entries = (_Result * 16)()
    query = _Query(_Name(ctypes.addressof(name), len(topic)), ctypes.addressof(entries),
                   len(entries), int(peers.environment["ROS_DOMAIN_ID"]))
    argument = bytearray(query)
    request = (3 << 30) | (ctypes.sizeof(_Query) << 16) | (0xA6 << 8) | number
    descriptor = os.open(peers.config.device, os.O_RDONLY)
    try:
        fcntl.ioctl(descriptor, request, argument, True)
    finally:
        os.close(descriptor)
    count = ctypes.c_uint32.from_buffer(argument).value
    assert count < len(entries), "endpoint result buffer may be truncated"
    return count


def wait_for(predicate, description, timeout=10):
    """Bound asynchronous unlink/exit cleanup, retaining a useful failure reason."""
    deadline = time.monotonic() + timeout
    while not predicate():
        assert time.monotonic() < deadline, description
        time.sleep(0.02)


def heap_path(child):
    """Return only this exact child's Agnocast heap name."""
    return Path(f"/dev/shm/agnocast@{child.process.pid}")


def queues(peers):
    """List only publish-notification queues belonging to this unique test topic."""
    return sorted(Path("/dev/mqueue").glob(f"agnocast*{peers.topic[1:]}@*"))


def cleaned(peers):
    """Require both kernel graph entries and owned filesystem resources to vanish."""
    wait_for(lambda: endpoint_count(peers, "publisher") == 0
             and endpoint_count(peers, "subscriber") == 0, "kernel endpoints remain")
    wait_for(lambda: not queues(peers), "owned notification queues remain")
    wait_for(lambda: not any(heap_path(child).exists() for child in peers.children),
             "owned shared-memory names remain")


def assert_connected_resources(peers, publisher, subscriber):
    """Establish nonvacuous resource/graph evidence before testing removal."""
    assert endpoint_count(peers, "publisher") == endpoint_count(peers, "subscriber") == 1
    assert heap_path(publisher).exists() and heap_path(subscriber).exists()
    assert len(queues(peers)) == 1


def heap_mappings(child):
    """Read this process's Agnocast heap mappings, including unlinked heaps."""
    return [line for line in Path(f"/proc/{child.process.pid}/maps").read_text().splitlines()
            if "/dev/shm/agnocast@" in line]


def publisher_mapping(mappings, publisher_pid, address, size):
    """Require the entire payload to lie in the publisher's read-only shared heap."""
    assert size > 0, "publisher mapping proof requires nonempty storage"
    expected = f"/dev/shm/agnocast@{publisher_pid}"
    for line in mappings:
        fields = line.split(maxsplit=5)
        if len(fields) != 6 or fields[1] != "r--s":
            continue
        if fields[5] not in (expected, expected + " (deleted)"):
            continue
        first, last = (int(value, 16) for value in fields[0].split("-"))
        if first <= address and address + size <= last:
            return line
    raise AssertionError(
        f"payload [{address:#x}, {address + size:#x}) is not in publisher {expected}")


def mapping_snapshot(child):
    """Record retained heap mappings and total resident/proportional memory in KiB."""
    root = Path(f"/proc/{child.process.pid}")
    mappings = heap_mappings(child)
    memory = {}
    for line in (root / "smaps_rollup").read_text().splitlines():
        key, _, value = line.partition(":")
        if key in ("Rss", "Pss"):
            memory[f"{key.lower()}_kib"] = int(value.split()[0])
    assert len(memory) == 2
    mapped = 0
    for line in mappings:
        first, last = line.split()[0].split("-")
        mapped += int(last, 16) - int(first, 16)
    return dict(pid=child.process.pid, maps=mappings, mapping_count=len(mappings),
                mapped_bytes=mapped, **memory)
