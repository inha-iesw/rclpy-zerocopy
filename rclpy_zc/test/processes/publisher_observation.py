"""Observe production publisher addresses, TLS, and allocation boundaries."""

import ctypes
import threading
import time

import numpy as np

from rclpy_agnocast import _native

from publisher_observation_lifecycle import run_lifecycle_case


def _snapshot():
    return _native._test_publisher_snapshot()


def _address(view):
    return ctypes.addressof(ctypes.c_uint8.from_buffer(view))


def observed_publish(session, size, sequence):
    """Publish bytes while checking the native filling ownership boundary."""
    assert size > 0, "pointer observation requires nonempty storage"
    before = _snapshot()
    with session._publisher.borrow(size) as loan:
        filling = _snapshot()
        assert filling["caller_tid"] == threading.get_native_id()
        assert filling["caller_tid"] != filling["worker_tid"]
        assert filling["caller_tls"] == 0
        assert filling["borrow_tls"] == 1
        view = loan.buffer
        array = np.frombuffer(view, dtype=np.uint8)
        try:
            address = _address(view)
            numpy_address = array.__array_interface__["data"][0]
            assert address == numpy_address == filling["data_address"]
            assert _native._test_is_shared(address)
            assert _native._test_is_shared(filling["message_address"])
            details = session._fill(loan, size, sequence)
            np.testing.assert_array_equal(
                array, (np.arange(size, dtype=np.uint64) + sequence) & 255
            )
            after_fill = _snapshot()
            assert after_fill["backend_epoch"] == filling["backend_epoch"]
            assert (
                after_fill["filling_allocations"]
                == before["filling_allocations"]
            )
            assert (
                after_fill["worker_allocations"]
                == filling["worker_allocations"]
            )
        finally:
            del array
            view.release()
        loan.publish()
    terminal = _snapshot()
    assert terminal["publish_complete"] == before["publish_complete"] + 1
    assert terminal["terminal_tls"] == 0
    assert terminal["filling_allocations"] == before["filling_allocations"]
    details.update(
        invalidated=session._invalidated(loan),
        worker_tid=terminal["worker_tid"],
        caller_tid=threading.get_native_id(),
        native_address=hex(address),
        numpy_address=hex(numpy_address),
        backend_epoch=filling["backend_epoch"],
        filling_allocations=after_fill["filling_allocations"],
        terminal_tls=terminal["terminal_tls"],
    )
    return details


def _python_tls(session):
    before = _snapshot()
    with session._publisher.borrow(32) as loan:
        borrowed = _snapshot()
        python_buffer = bytearray(1024 * 1024)
        numpy_buffer = np.ones(1024 * 1024, dtype=np.uint8)
        view = loan.buffer
        try:
            shared_address = _address(view)
            python_address = _address(python_buffer)
            numpy_address = numpy_buffer.__array_interface__["data"][0]
            caller = _snapshot()
            assert caller["caller_tid"] == threading.get_native_id()
            assert caller["caller_tid"] != borrowed["worker_tid"]
            assert borrowed["borrow_tls"] == 1
            assert caller["caller_tls"] == 0
            assert _native._test_is_shared(shared_address)
            assert not _native._test_is_shared(python_address)
            assert not _native._test_is_shared(numpy_address)
        finally:
            view.release()
        loan.cancel()
    terminal = _snapshot()
    assert terminal["cancel_complete"] == before["cancel_complete"] + 1
    assert terminal["terminal_tls"] == 0
    return {
        "worker_tid": borrowed["worker_tid"],
        "caller_tid": caller["caller_tid"],
        "caller_tls": caller["caller_tls"],
        "borrow_tls": borrowed["borrow_tls"],
        "shared_address": hex(shared_address),
        "python_address": hex(python_address),
        "numpy_address": hex(numpy_address),
    }


def _addresses(session):
    with session._publisher.borrow(32) as loan:
        native = _snapshot()
        view = loan.buffer
        array = np.frombuffer(view, dtype=np.uint8)
        try:
            address = _address(view)
            numpy_address = array.__array_interface__["data"][0]
            assert address == native["data_address"] == numpy_address
            assert _native._test_is_shared(address)
            array[7] = 173
            assert view[7] == 173
            view[11] = 91
            assert array[11] == 91
        finally:
            del array
            view.release()
        loan.cancel()
    return {
        "native_address": hex(native["data_address"]),
        "view_address": hex(address),
        "numpy_address": hex(numpy_address),
        "bytes": 32,
    }


def _oversize(session):
    before = _snapshot()
    errors = []
    for size, expected in (
        (session._limit + 1, (ValueError,)),
        (-1, (ValueError,)),
        (1 << 100, (TypeError, OverflowError)),
    ):
        try:
            loan = session._publisher.borrow(size)
        except expected as error:
            errors.append(type(error).__name__)
        else:
            loan.cancel()
            raise AssertionError(f"borrow accepted invalid size {size}")
    after = _snapshot()
    assert after["borrow_entries"] == before["borrow_entries"]
    return {"rejected": len(errors), "borrow_entries_delta": 0}


def _allocation_failure(session, stage, ordinal):
    before = _snapshot()
    _native._test_fail_allocation(stage, ordinal)
    try:
        loan = session._publisher.borrow(min(128, session._limit))
    except MemoryError:
        pass
    else:
        loan.cancel()
        raise AssertionError(f"{stage} allocation {ordinal} did not fail")
    failed = _snapshot()
    assert failed["injected_failures"] == before["injected_failures"] + 1
    assert failed["terminal_tls"] == 0
    assert failed["borrow_entries"] == before["borrow_entries"] + 1
    assert failed["borrow_complete"] == before["borrow_complete"]
    assert failed["publish_complete"] == before["publish_complete"]
    assert failed["cancel_complete"] == before["cancel_complete"]
    assert failed["failure_live_allocations"] == 0
    assert failed["failure_tracking_overflow"] == 0
    if stage == "borrow":
        assert failed["failure_tracked_allocations"] == ordinal
    else:
        assert failed["failure_tracked_allocations"] >= 1
    return {
        "failure_stage": stage,
        "failure_ordinal": ordinal,
        "injected_failures": (
            failed["injected_failures"] - before["injected_failures"]
        ),
        "failure_live_allocations": failed["failure_live_allocations"],
        "failure_tracked_allocations": failed["failure_tracked_allocations"],
        "failure_tracking_overflow": failed["failure_tracking_overflow"],
        "terminal_tls": failed["terminal_tls"],
    }


def _filling_wait(session):
    before = _snapshot()
    with session._publisher.borrow(32) as loan:
        filling = _snapshot()
        try:
            _native._test_fail_allocation("borrow", 0)
        except RuntimeError:
            pass
        else:
            raise AssertionError("allocation injection accepted an active loan")
        session._fill(loan, 32, 817)
        time.sleep(0.05)
        after_wait = _snapshot()
        assert after_wait["backend_epoch"] == filling["backend_epoch"]
        assert (
            after_wait["filling_allocations"] == before["filling_allocations"]
        )
        assert (
            after_wait["worker_allocations"] == filling["worker_allocations"]
        )
        assert filling["shared_allocations"] > before["shared_allocations"]
        loan.cancel()
    terminal = _snapshot()
    assert terminal["filling_allocations"] == before["filling_allocations"]
    return {
        "backend_epoch_before": filling["backend_epoch"],
        "backend_epoch_after": after_wait["backend_epoch"],
        "worker_allocations_delta": 0,
        "filling_allocations_delta": 0,
        "shared_allocations_delta": (
            filling["shared_allocations"] - before["shared_allocations"]
        ),
    }


def run_case(session, case):
    """Run one observation case against the session's production endpoint."""
    case = case.replace("-", "")
    failures = {
        "PUB15borrow0": ("borrow", 0),
        "PUB15borrow1": ("borrow", 1),
        "PUB15borrow2": ("borrow", 2),
        "PUB15resize0": ("resize", 0),
    }
    if case in failures:
        return _allocation_failure(session, *failures[case])
    handlers = {
        "PUB02": _python_tls,
        "PUB06": _addresses,
        "PUB14": _oversize,
        "PUB17": _filling_wait,
    }
    if case in handlers:
        return handlers[case](session)
    return run_lifecycle_case(session, case)
