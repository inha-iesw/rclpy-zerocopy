"""Device-independent publisher buffer and lifecycle contract tests."""

import ctypes
import gc
import queue
import threading
import time

import numpy as np
import pytest

from rclpy_agnocast import Publisher
from rclpy_agnocast import _native


def _buffer_address(view: memoryview) -> int:
    return ctypes.addressof(ctypes.c_uint8.from_buffer(view))


def _wait_for(predicate, timeout: float = 1.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.001)
    assert predicate(), "native worker did not reach the expected state"


def _close_when_idle(publisher: Publisher, timeout: float = 1.0) -> None:
    deadline = time.monotonic() + timeout
    last_error = None
    while time.monotonic() < deadline:
        try:
            publisher.close()
            return
        except RuntimeError as error:
            last_error = error
            time.sleep(0.001)
    raise AssertionError("publisher did not become idle") from last_error


@pytest.fixture
def endpoint_factory():
    opened = []

    def create(max_payload_bytes: int = 64):
        handle = _native._TestPublisher(max_payload_bytes)
        publisher = Publisher(handle)
        opened.append(publisher)
        return publisher, handle

    yield create

    gc.collect()
    for publisher in reversed(opened):
        try:
            _close_when_idle(publisher, timeout=0.1)
        except (AssertionError, RuntimeError):
            pass


def test_memoryview_metadata_address_and_terminal_invalidation(endpoint_factory):
    publisher, handle = endpoint_factory(32)
    loan = publisher.borrow(16)
    loan.sequence = 42
    view = loan.buffer

    assert view.format == "B"
    assert view.itemsize == 1
    assert view.ndim == 1
    assert view.shape == (16,)
    assert view.strides == (1,)
    assert view.c_contiguous and view.contiguous
    assert not view.readonly
    assert type(view.obj) is _native._WritableBufferExport
    assert _buffer_address(view) == handle.native_data_address

    view[:] = bytes(range(16))
    view.release()
    del view
    gc.collect()
    loan.publish()

    assert loan.phase == _native.LoanPhase.PUBLISHED
    assert handle.publish_count == 1
    assert handle.published_sequence == 42
    assert handle.terminal_on_backend_thread
    with pytest.raises(RuntimeError):
        _ = loan.sequence
    with pytest.raises(RuntimeError):
        _ = loan.buffer


def test_slice_alias_retains_export_and_blocks_publish(endpoint_factory):
    publisher, _ = endpoint_factory()
    loan = publisher.borrow(12)
    root = loan.buffer
    alias = root[3:9]

    root.release()
    del root
    gc.collect()
    assert loan._state.active_writable_exports == 1
    with pytest.raises(BufferError, match="aliases"):
        loan.publish()

    alias.release()
    del alias
    gc.collect()
    assert loan._state.active_writable_exports == 0
    loan.publish()


def test_numpy_alias_keeps_same_address_and_export_lifetime(endpoint_factory):
    publisher, handle = endpoint_factory()
    loan = publisher.borrow(24)
    source_view = loan.buffer
    array = np.frombuffer(source_view, dtype=np.uint8)

    assert array.shape == (24,)
    assert array.flags.c_contiguous and array.flags.writeable
    assert array.__array_interface__["data"][0] == handle.native_data_address
    del source_view
    gc.collect()
    assert loan._state.active_writable_exports == 1
    with pytest.raises(BufferError):
        loan.publish()

    array[:] = np.arange(24, dtype=np.uint8)
    del array
    gc.collect()
    assert loan._state.active_writable_exports == 0
    loan.publish()


def test_publish_and_cancel_reject_active_export(endpoint_factory):
    publisher, handle = endpoint_factory()
    loan = publisher.borrow(8)
    view = loan.buffer

    with pytest.raises(BufferError):
        publisher.publish(loan)
    with pytest.raises(BufferError):
        publisher.cancel(loan)
    assert loan.phase == _native.LoanPhase.FILLING

    view.release()
    del view
    gc.collect()
    loan.cancel()
    assert loan.phase == _native.LoanPhase.CANCELLED
    assert handle.cancel_count == 1
    assert handle.terminal_on_backend_thread


def test_zero_limit_and_oversize_boundaries(endpoint_factory):
    publisher, handle = endpoint_factory(64)

    zero = publisher.borrow(0)
    zero_view = zero.buffer
    assert zero_view.shape == (0,)
    assert zero_view.nbytes == 0
    zero_view.release()
    del zero_view
    gc.collect()
    zero.cancel()

    with pytest.raises(ValueError, match="configured limit"):
        publisher.borrow(65)
    with pytest.raises(ValueError, match="non-negative"):
        publisher.borrow(-1)
    with pytest.raises(TypeError, match="integer"):
        publisher.borrow(True)
    with pytest.raises((TypeError, OverflowError)):
        publisher.borrow(1 << 100)

    maximum = publisher.borrow(64)
    maximum_view = maximum.buffer
    assert len(maximum_view) == handle.max_payload_bytes
    maximum_view[:] = b"x" * 64
    maximum_view.release()
    del maximum_view
    gc.collect()
    maximum.publish()


def test_wrong_publisher_is_rejected_before_native_terminal_call(endpoint_factory):
    owner, owner_handle = endpoint_factory()
    stranger, stranger_handle = endpoint_factory()
    assert owner_handle.id != stranger_handle.id
    loan = owner.borrow(1)

    with pytest.raises(ValueError, match="different publisher"):
        stranger.publish(loan)
    with pytest.raises(ValueError, match="different publisher"):
        stranger.cancel(loan)
    assert loan.phase == _native.LoanPhase.FILLING
    loan.cancel()
    assert owner_handle.cancel_count == 1
    assert stranger_handle.cancel_count == 0


def test_creator_thread_guard_uses_a_real_foreign_thread(endpoint_factory):
    publisher, _ = endpoint_factory()
    loan = publisher.borrow(1)
    observed = queue.Queue()

    def read_from_foreign_thread():
        try:
            observed.put(loan.sequence)
        except BaseException as error:
            observed.put(error)

    thread = threading.Thread(target=read_from_foreign_thread)
    thread.start()
    thread.join()
    error = observed.get_nowait()
    assert isinstance(error, RuntimeError)
    assert "creator thread" in str(error)
    loan.cancel()


def test_context_exception_defers_cancel_until_foreign_release(endpoint_factory):
    publisher, handle = endpoint_factory()
    held_views = []
    creator_thread = threading.get_ident()

    with pytest.raises(LookupError, match="original failure"):
        with publisher.borrow(8) as loan:
            held_views.append(loan.buffer)
            raise LookupError("original failure")

    assert loan.phase == _native.LoanPhase.CANCEL_PENDING
    assert handle.cancel_count == 0
    release_threads = []

    def release_from_foreign_thread():
        release_threads.append(threading.get_ident())
        view = held_views.pop()
        view.release()
        del view
        gc.collect()

    thread = threading.Thread(target=release_from_foreign_thread)
    thread.start()
    thread.join()
    assert release_threads == [thread.ident]
    assert release_threads[0] != creator_thread
    _wait_for(lambda: handle.cancel_count == 1)
    assert handle.terminal_on_backend_thread
    assert loan.phase == _native.LoanPhase.CANCELLING
    _close_when_idle(publisher)


def test_private_exporter_cannot_be_constructed_from_python():
    with pytest.raises(TypeError):
        _native._WritableBufferExport()
