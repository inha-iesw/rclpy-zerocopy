"""Negative controls for the publisher-backed read-only mapping oracle."""

import pytest

from checkpoint import resources


def mapping(path="/dev/shm/agnocast@21", permissions="r--s", first=0x1000, last=0x2000):
    return f"{first:x}-{last:x} {permissions} 00000000 00:01 42 {path}"


@pytest.mark.parametrize("suffix", ["", " (deleted)"])
def test_publisher_mapping_covers_the_entire_payload_including_unlinked_heap(suffix):
    line = mapping(path="/dev/shm/agnocast@21" + suffix)
    assert resources.publisher_mapping([line], 21, 0x1000, 0x1000) == line


@pytest.mark.parametrize("line,address,size", [
    (mapping(path="/dev/shm/agnocast@22"), 0x1000, 8),
    (mapping(path="/dev/shm/agnocast@210"), 0x1000, 8),
    (mapping(path=""), 0x1000, 8),
    (mapping(permissions="rw-s"), 0x1000, 8),
    (mapping(permissions="r--p"), 0x1000, 8),
    (mapping(), 0xFFF, 8),
    (mapping(), 0x2000, 1),
    (mapping(), 0x1FFF, 2),
])
def test_private_wrong_owner_or_out_of_range_mapping_is_rejected(line, address, size):
    with pytest.raises(AssertionError, match="publisher"):
        resources.publisher_mapping([line], 21, address, size)
