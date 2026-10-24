"""Typed receive aliases retain native storage and its queue reservation."""

from concurrent.futures import ThreadPoolExecutor
import ctypes
import gc
import struct

import numpy as np
import pytest

from rclpy_agnocast import _native
from rclpy_agnocast.types.message import Message


def endpoint(count=4, retained=1000000, maximum=1000000):
    return _native._TestTypedSubscription(count, retained, maximum)


def test_received_fields_are_readonly_before_any_mutation():
    sub = endpoint()
    assert sub.push(-17)
    message = Message(sub.take())
    assert message.signed == -17
    assert message.children[0].values[1] == -6
    assert message._owner.snapshot['phase'] == 'READ_ONLY'
    with pytest.raises(TypeError, match='read.only'):
        message.signed = 3
    with pytest.raises(TypeError, match='read.only'):
        message.values[0] = 3
    with pytest.raises(TypeError, match='read.only'):
        message.children.resize(0)
    assert message.signed == -17
    assert message.values[0] == -17
    assert message.children[0].values[1] == -6
    del message
    assert sub.release_count == 1


def test_readonly_native_buffer_metadata_pointer_and_write_requests():
    sub = endpoint()
    assert sub.push(40)
    message = Message(sub.take())
    view = message.values.buffer()
    array = np.frombuffer(view, dtype=np.int32)
    assert view.readonly and view.format == 'i'
    assert (view.ndim, view.itemsize, view.shape, view.strides) == (1, 4, (3,), (4,))
    assert view.c_contiguous and view.nbytes == 12
    assert array.tolist() == [40, 41, 42]
    assert array.__array_interface__['data'][0] == sub.last_data_address
    assert not array.flags.writeable
    with pytest.raises(TypeError):
        view[0] = 99
    with pytest.raises(ValueError):
        array.setflags(write=True)
    with pytest.raises(TypeError):
        memoryview(view.obj)[0] = 99
    with pytest.raises(TypeError):
        ctypes.c_int32.from_buffer(view.obj)
    del array
    view.release()
    del message
    assert sub.release_count == 1


def test_nested_child_retains_one_root_reservation_after_parent_is_gone():
    sub = endpoint(count=1)
    assert sub.push(7)
    message = Message(sub.take())
    child = message.children[0]
    accounted = sub.last_accounted_bytes
    assert message._owner.snapshot['accounted_bytes'] == accounted
    del message
    gc.collect()
    assert child.values[1] == 18
    assert sub.stats.outstanding_leases == 1
    assert sub.stats.outstanding_accounted_bytes == accounted
    assert not sub.push(8)
    assert sub.stats.dropped_count_full == 1
    del child
    gc.collect()
    assert sub.stats.outstanding_leases == 0
    assert sub.stats.outstanding_accounted_bytes == 0
    assert sub.push(9)
    sub.clear_pending()


def test_slice_retains_budget_after_message_and_source_view_are_released():
    sub = endpoint()
    assert sub.push(30)
    message = Message(sub.take())
    source = message.values.buffer()
    alias = source[1:]
    accounted = sub.last_accounted_bytes
    source.release()
    del message, source
    gc.collect()
    assert alias.tolist() == [31, 32]
    assert sub.release_count == 0
    assert sub.stats.outstanding_accounted_bytes == accounted
    alias.release()
    del alias
    assert sub.release_count == 1
    assert sub.stats.outstanding_accounted_bytes == 0


def test_foreign_thread_can_read_and_release_final_alias():
    sub = endpoint()
    assert sub.push(10)
    held = [Message(sub.take())]

    def consume():
        message = held.pop()
        with message.values.buffer() as view:
            return message.signed, message.children[0].values[0], view.tolist()

    with ThreadPoolExecutor(max_workers=1) as executor:
        assert executor.submit(consume).result(timeout=2) == (10, 20, [10, 11, 12])
    assert sub.release_count == 1
    assert sub.stats.outstanding_leases == 0


def test_close_drains_queued_roots_but_preserves_delivered_alias_until_retry():
    sub = endpoint()
    assert sub.push(12)
    message = Message(sub.take())
    view = message.values.buffer()
    del message
    assert sub.push(20)
    with pytest.raises(RuntimeError, match='active root or view'):
        sub.close()
    assert sub.stats.queued_leases == 0
    assert sub.stats.outstanding_leases == 1
    assert view.tolist() == [12, 13, 14]
    assert sub.release_count == 1
    view.release()
    sub.close()
    sub.close()
    assert sub.release_count == 2
    assert sub.stats.outstanding_accounted_bytes == 0


def test_strings_packed_booleans_and_empty_fields_preserve_read_contract():
    sub = endpoint()
    assert sub.push(5, 0)
    message = Message(sub.take())
    assert message.text.to_text() == 'typed-5'
    assert message.wide.to_text() == 'wide'
    with message.text.buffer() as view:
        assert view.readonly and view.format == 'B'
        assert bytes(view) == b'typed-5'
    with message.wide.buffer() as view:
        assert view.readonly and view.format == 'H' and view.itemsize == 2
        assert view.tolist() == list(map(ord, 'wide'))
    assert message.packed[0] is True and message.packed[1] is False
    with pytest.raises(NotImplementedError, match='packed boolean'):
        message.packed.buffer()
    with message.values.buffer() as view:
        assert view.readonly and view.shape == (0,) and view.nbytes == 0
    with pytest.raises(IndexError):
        _ = message.values[0]
    del message
    assert sub.release_count == 1


def test_accounted_bytes_include_nested_storage_and_limit_admission():
    probe = endpoint()
    assert probe.push(1)
    accounted = probe.last_accounted_bytes
    assert accounted > 3 * 4 + 2 * 4
    probe.clear_pending()
    sub = endpoint(retained=accounted)
    assert sub.push(1)
    first = Message(sub.take())
    assert not sub.push(2)
    assert sub.stats.dropped_bytes_full == 1
    assert sub.stats.outstanding_accounted_bytes == accounted
    del first
    assert sub.push(3)
    sub.clear_pending()
    small = endpoint(maximum=accounted - 1)
    assert not small.push(1)
    assert small.stats.dropped_oversize == 1
    assert small.stats.outstanding_accounted_bytes == 0


def test_three_roots_charge_queue_callback_and_byte_alias_storage_once():
    sub = endpoint(count=3)
    assert sub.push(10, 1)
    callback_root = Message(sub.take())
    callback_bytes = sub.last_accounted_bytes
    assert callback_root._owner.snapshot['accounted_bytes'] == callback_bytes

    assert sub.push(20, 6)
    alias_root = Message(sub.take())
    alias_bytes = sub.last_accounted_bytes
    assert alias_root._owner.snapshot['accounted_bytes'] == alias_bytes
    source = alias_root.values.buffer()
    one_byte = source.cast('B')[:1]
    second_alias = source[1:3]
    source.release()
    del source, alias_root

    assert sub.push(30, 0)
    queued_bytes = sub.last_accounted_bytes
    assert len({callback_bytes, alias_bytes, queued_bytes}) == 3
    assert sub.stats.queued_leases == 1
    assert sub.stats.outstanding_leases == 3
    assert sub.stats.outstanding_accounted_bytes == callback_bytes + alias_bytes + queued_bytes
    assert sub.stats.outstanding_payload_bytes == 0
    assert callback_root.values[0] == 10
    assert second_alias.tolist() == [21, 22]
    assert bytes(one_byte) == struct.pack('=i', 20)[:1]
    assert not sub.push(40, 2)
    assert sub.stats.dropped_count_full == 1

    sub.clear_pending()
    assert sub.stats.outstanding_accounted_bytes == callback_bytes + alias_bytes
    del callback_root
    assert sub.stats.outstanding_leases == 1
    assert sub.stats.outstanding_accounted_bytes == alias_bytes
    second_alias.release()
    del second_alias
    assert one_byte.nbytes == 1
    assert sub.stats.outstanding_accounted_bytes == alias_bytes
    one_byte.release()
    del one_byte
    assert sub.stats.outstanding_leases == 0
    assert sub.stats.outstanding_accounted_bytes == 0
    assert sub.push(50, 2)
    sub.clear_pending()


def test_readonly_export_and_root_cannot_be_constructed_from_python():
    with pytest.raises(TypeError):
        _native._RootLeaseOwner()
    with pytest.raises(TypeError):
        _native._TypedReadonlyBuffer()
