"""Device-independent subscriber admission and alias ownership contracts."""

from concurrent.futures import ThreadPoolExecutor
import gc
import os
import threading

import numpy as np
import pytest

from rclpy_agnocast import _native


def endpoint(count=2, limit=16):
    return _native._TestSubscription(count, limit)


def test_gil_hold_wait_blocks_python_producer_until_return():
    sub = endpoint()
    reader, writer = os.pipe()

    def produce():
        assert b"BLOCKING" in os.read(reader, 128)
        assert sub.push(1, b"x")

    try:
        with ThreadPoolExecutor(max_workers=1) as executor:
            producer = executor.submit(produce)
            try:
                # Capture the result in native code before Python scheduling can resume.
                assert not _native._test_wait_for_receipts_holding_gil(sub, 1, 300, writer)
            finally:
                os.close(writer)
                writer = None
            producer.result(timeout=2)
        assert sub.stats.received_total == 1
    finally:
        os.close(reader)
        if writer is not None:
            os.close(writer)


def test_count_limit_includes_queue_delivered_and_view_then_recovers():
    sub = endpoint()
    assert sub.push(10, b"abcd")
    lease = sub.take()
    view = lease.buffer
    lease.release()
    assert sub.push(11, b"efgh")
    assert not sub.push(12, b"i")
    assert sub.stats.outstanding_leases == 2
    assert sub.stats.queued_leases == 1
    assert sub.stats.outstanding_payload_bytes == 8
    view.release()
    del view
    assert sub.stats.outstanding_leases == 1
    assert sub.push(13, b"j")
    assert sub.take().sequence == 11
    assert sub.take().sequence == 13
    assert sub.stats.dropped_outstanding == 1


def test_aggregate_byte_limit_distinguishes_single_oversize():
    sub = endpoint(count=4, limit=8)
    assert sub.push(1, b"12345")
    first = sub.take()
    assert not sub.push(2, b"6789")
    assert not sub.push(3, b"123456789")
    assert sub.push(4, b"678")
    assert sub.stats.outstanding_payload_bytes == 8
    assert sub.stats.dropped_outstanding == 1
    assert sub.stats.dropped_oversize == 1
    assert sub.stats.received_total == 4
    first.release()
    assert sub.push(5, b"abcde")
    sub.close()
    assert sub.stats.outstanding_payload_bytes == 0
    assert sub.release_count == 5


def test_readonly_metadata_and_native_numpy_address_match():
    sub = endpoint()
    sub.push(42, b"abcdef")
    lease = sub.take()
    view = lease.buffer
    array = np.frombuffer(view, dtype=np.uint8)
    assert view.readonly and view.format == "B"
    assert view.ndim == 1 and view.itemsize == 1
    assert view.shape == (6,) and view.strides == (1,)
    assert view.c_contiguous and not array.flags.writeable
    assert array.__array_interface__["data"][0] == sub.last_data_address
    assert lease.sequence == 42
    with pytest.raises(TypeError):
        view[0] = 1
    with pytest.raises(ValueError):
        array[0] = 1
    with pytest.raises(ValueError):
        array.setflags(write=True)
    del array
    view.release()
    lease.release()
    assert sub.release_count == 1


def test_wrapper_release_invalidates_access_but_preserves_existing_slice():
    sub = endpoint()
    sub.push(7, b"abcdef")
    lease = sub.take()
    root = lease.buffer
    alias = root[1:5]
    lease.release()
    root.release()
    del root
    with pytest.raises(RuntimeError, match="released"):
        _ = lease.sequence
    with pytest.raises(RuntimeError, match="released"):
        _ = lease.buffer
    assert bytes(alias) == b"bcde"
    assert sub.release_count == 0
    assert sub.stats.outstanding_payload_bytes == 6
    alias.release()
    del alias
    assert sub.release_count == 1
    assert sub.stats.outstanding_leases == 0


def test_numpy_alias_outlives_lease_and_source_variable():
    sub = endpoint()
    sub.push(8, b"123456")
    lease = sub.take()
    source = lease.buffer
    array = np.frombuffer(source, dtype=np.uint8)
    lease.release()
    del lease, source
    gc.collect()
    assert array.tolist() == [49, 50, 51, 52, 53, 54]
    assert sub.release_count == 0
    del array
    gc.collect()
    assert sub.release_count == 1


def test_foreign_thread_last_alias_returns_budget_once():
    sub = endpoint()
    sub.push(1, b"xyz")
    lease = sub.take()
    held = [lease.buffer]
    lease.release()

    def release():
        held.pop().release()
        return threading.get_native_id()

    with ThreadPoolExecutor(max_workers=1) as executor:
        release_tid = executor.submit(release).result(timeout=2)
    assert sub.release_tid == release_tid != threading.get_native_id()
    lease.release()
    assert sub.release_count == 1
    assert sub.stats.outstanding_payload_bytes == 0


def test_context_exception_duplicate_release_and_gc_do_not_double_free():
    sub = endpoint()
    sub.push(1, b"x")
    with pytest.raises(LookupError, match="body"):
        with sub.take() as lease:
            raise LookupError("body")
    lease.release()
    del lease
    gc.collect()
    assert sub.release_count == 1
    assert sub.stats.outstanding_leases == 0


def test_close_drains_queue_and_retries_after_active_view():
    sub = endpoint()
    sub.push(1, b"12")
    lease = sub.take()
    view = lease.buffer
    lease.release()
    sub.push(2, b"34")
    with pytest.raises(RuntimeError, match="Lease"):
        sub.close()
    assert sub.stats.queued_leases == 0
    assert sub.stats.outstanding_leases == 1
    assert bytes(view) == b"12"
    assert not sub.push(3, b"x")
    view.release()
    sub.close()
    sub.close()
    assert sub.stats.outstanding_payload_bytes == 0
    assert sub.release_count == 3


def test_zero_size_still_consumes_one_count_slot():
    sub = endpoint(count=1)
    assert sub.push(1, b"")
    assert not sub.push(2, b"")
    lease = sub.take()
    view = lease.buffer
    assert view.readonly and view.nbytes == 0
    assert view.shape == (0,)
    view.release()
    lease.release()
    assert sub.stats.outstanding_leases == 0
    assert sub.push(3, b"")
    sub.close()


@pytest.mark.parametrize("count,limit", [(0, 4), (1, 0)])
def test_zero_limits_are_rejected(count, limit):
    with pytest.raises(ValueError):
        endpoint(count, limit)


def test_export_owner_cannot_be_constructed_from_python():
    with pytest.raises(TypeError):
        _native._ReadonlyBufferExport()
