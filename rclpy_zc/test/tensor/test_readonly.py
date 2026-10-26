"""CPU-T02/T03 subscriber tensors require a verified foreign read-only mapping."""

import gc
import os
import struct
import threading

import pytest

from rclpy_agnocast import _native
from rclpy_agnocast.types.message import Message
from write_fault import assert_write_fault


def test_readonly_tensor_pointer_values_and_explicit_clone(subscription, torch):
    native = subscription()
    assert native.push()
    message = Message(native.take())
    tensor = message.values.as_tensor(shape=(2, 3))
    assert tensor.dtype == torch.int32
    assert tensor.data_ptr() == native.last_data_address
    assert tensor.tolist() == [[10, 11, 12], [13, 14, 15]]
    assert tensor.sum().item() == 75
    clone = tensor.clone()
    assert clone.data_ptr() != tensor.data_ptr()
    clone.fill_(99)
    assert tensor.tolist() == [[10, 11, 12], [13, 14, 15]]
    assert clone.tolist() == [[99, 99, 99], [99, 99, 99]]
    del clone, tensor, message
    gc.collect()
    assert native.release_count == 1
    assert native.stats.outstanding_accounted_bytes == 0


def test_tensor_alias_keeps_single_charge_after_root_and_view_release(subscription, torch):
    native = subscription(count=1)
    assert native.push()
    message = Message(native.take())
    source = message.values.buffer()
    tensor = message.values.as_tensor()
    alias = tensor[1:4].detach()
    accounted = native.last_accounted_bytes
    assert accounted == 24
    source.release()
    del source, tensor, message
    gc.collect()
    assert alias.tolist() == [11, 12, 13]
    assert native.release_count == 0
    assert native.stats.outstanding_leases == 1
    assert native.stats.outstanding_accounted_bytes == accounted
    assert not native.push()
    assert native.stats.dropped_count_full == 1
    assert native.release_count == 1
    assert native.stats.outstanding_leases == 1
    assert native.stats.outstanding_accounted_bytes == accounted
    released_before_alias = native.release_count
    with pytest.raises(RuntimeError, match='active root or view'):
        native.close()
    assert alias.tolist() == [11, 12, 13]
    del alias
    gc.collect()
    assert native.release_count == released_before_alias + 1
    assert native.stats.outstanding_leases == 0
    assert native.stats.outstanding_accounted_bytes == 0
    native.close()


def test_byte_budget_stays_reserved_by_only_tensor_alias(subscription, torch):
    native = subscription(count=3, retained=24)
    assert native.push()
    message = Message(native.take())
    tensor = message.values.as_tensor()
    alias = tensor[:1].detach()
    del tensor, message
    gc.collect()
    assert not native.push()
    assert native.stats.dropped_bytes_full == 1
    assert native.stats.outstanding_accounted_bytes == 24
    del alias
    gc.collect()
    assert native.push()
    native.clear_pending()
    assert native.stats.outstanding_accounted_bytes == 0
    small = subscription(maximum=23)
    assert not small.push()
    assert small.stats.dropped_oversize == 1
    assert small.stats.outstanding_accounted_bytes == 0


def test_last_tensor_alias_on_foreign_thread_returns_budget(subscription, torch):
    native = subscription()
    assert native.push()
    message = Message(native.take())
    tensor = message.values.as_tensor()
    aliases = [tensor[1:4].detach()]
    del tensor, message
    gc.collect()
    assert aliases[0].tolist() == [11, 12, 13]
    assert native.release_count == 0
    assert native.stats.outstanding_leases == 1
    assert native.stats.outstanding_accounted_bytes == 24

    def release():
        aliases.pop()
        gc.collect()

    thread = threading.Thread(target=release)
    thread.start()
    thread.join(timeout=5)
    assert not thread.is_alive()
    assert native.release_count == 1
    assert native.stats.outstanding_leases == 0
    assert native.stats.outstanding_accounted_bytes == 0
    native.close()


def test_unknown_heap_origin_rejects_before_tensor_export(torch):
    native = _native._TestTypedSubscription(4, 1000000, 1000000)
    assert native.push(10)
    message = Message(native.take())
    with pytest.raises((RuntimeError, NotImplementedError), match='origin|mapping|read.only'):
        message.values.as_tensor()
    assert message.values[0] == 10
    del message
    assert native.release_count == 1
    native.close()


@pytest.mark.parametrize('writable,recognized', [(True, True), (False, False)])
def test_unprotected_or_unrecognized_mapping_rejects(
        subscription, shared_file, torch, writable, recognized):
    native = subscription(writable=writable, name=shared_file(recognized=recognized))
    assert native.push()
    message = Message(native.take())
    with pytest.raises((RuntimeError, NotImplementedError), match='origin|mapping|read.only'):
        message.values.as_tensor()
    assert message.values[0] == 10
    del message
    assert native.release_count == 1


def test_same_process_publication_origin_rejects(subscription, torch):
    name = _native._test_agnocast_shm_name(os.getpid())
    path = '/dev/shm' + name
    with open(path, 'xb') as output:
        output.write(struct.pack('=6i', *range(10, 16)))
    try:
        native = subscription(name=name)
        assert native.push()
        message = Message(native.take())
        with pytest.raises((RuntimeError, NotImplementedError), match='origin|PID|process'):
            message.values.as_tensor()
        assert message.values[0] == 10
        del message
        assert native.release_count == 1
    finally:
        os.unlink(path)


@pytest.mark.skipif(not hasattr(os, 'fork'), reason='isolated write proof requires fork')
def test_foreign_mapping_write_fault_is_confined_to_child(subscription, torch):
    native = subscription()
    assert native.push()
    message = Message(native.take())
    tensor = message.values.as_tensor()
    assert_write_fault(tensor)
    assert tensor.tolist() == [10, 11, 12, 13, 14, 15]
    assert native.push()
    native.clear_pending()
    del tensor, message
    assert native.release_count == 2
