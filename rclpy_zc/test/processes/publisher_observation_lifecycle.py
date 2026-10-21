"""Observe production publisher affinity, rejection, and deferred cleanup."""

import gc
import threading
import time
import warnings

from rclpy_agnocast import _native


def _snapshot():
    return _native._test_publisher_snapshot()


def _wait_cancel(expected):
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        observed = _snapshot()
        if observed["cancel_complete"] >= expected:
            assert observed["cancel_complete"] == expected
            assert observed["terminal_tls"] == 0
            return observed
        time.sleep(0.001)
    raise AssertionError(
        f"cancel count did not reach {expected}: {_snapshot()}"
    )


def _reject(operation, expected):
    try:
        operation()
    except expected as error:
        return error
    raise AssertionError("invalid operation unexpectedly succeeded")


def _affinity(session):
    before = _snapshot()
    loan = session._publisher.borrow(16)
    session._fill(loan, 16, 8101)
    loan.publish()
    loan = session._publisher.borrow(16)
    loan.cancel()
    loan = session._publisher.borrow(16)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", ResourceWarning)
        del loan
        gc.collect()
    after = _wait_cancel(before["cancel_complete"] + 2)
    worker_tid = after["worker_tid"]
    assert worker_tid != threading.get_native_id()
    operations = (
        "borrow_tid", "resize_tid", "publish_tid", "cancel_tid", "destroy_tid"
    )
    for key in operations:
        assert after[key] == worker_tid, (key, after[key], worker_tid)
    assert after["publish_complete"] == before["publish_complete"] + 1
    assert after["unpublished_destroy"] == before["unpublished_destroy"] + 2
    return {
        "worker_tid": worker_tid,
        "caller_tid": threading.get_native_id(),
        "borrow_tid": after["borrow_tid"],
        "resize_tid": after["resize_tid"],
        "publish_tid": after["publish_tid"],
        "cancel_tid": after["cancel_tid"],
        "destroy_tid": after["destroy_tid"],
        "unpublished_destroy": (
            after["unpublished_destroy"] - before["unpublished_destroy"]
        ),
        "published_sequence": 8101,
        "published_bytes": 16,
    }


def _cancel_recovery(session):
    before = _snapshot()
    with session._publisher.borrow(16) as loan:
        loan.cancel()
    cancelled = _snapshot()
    assert cancelled["cancel_complete"] == before["cancel_complete"] + 1
    assert (
        cancelled["unpublished_destroy"] == before["unpublished_destroy"] + 1
    )
    assert cancelled["terminal_tls"] == 0
    with session._publisher.borrow(1) as recovery:
        recovery.cancel()
    after = _snapshot()
    assert after["borrow_complete"] == before["borrow_complete"] + 2
    assert after["cancel_complete"] == before["cancel_complete"] + 2
    assert after["terminal_tls"] == 0
    return {
        "cancel_complete_delta": 2,
        "borrow_complete_delta": 2,
        "terminal_tls": 0,
    }


def _deferred_cancel(session, foreign):
    before = _snapshot()
    sentinel = LookupError("held-view-original-exception")
    try:
        with session._publisher.borrow(32) as loan:
            root = loan.buffer
            held = [root[1:]]
            root.release()
            raise sentinel
    except LookupError as caught:
        assert caught is sentinel
    assert loan.phase == _native.LoanPhase.CANCEL_PENDING
    pending = _snapshot()
    assert pending["cancel_complete"] == before["cancel_complete"]
    assert pending["terminal_enqueues"] == before["terminal_enqueues"]
    assert pending["unpublished_destroy"] == before["unpublished_destroy"]
    releases = []
    failures = []

    def release():
        try:
            releases.append(threading.get_native_id())
            alias = held.pop()
            alias.release()
        except BaseException as error:
            failures.append(error)

    if foreign:
        thread = threading.Thread(target=release)
        thread.start()
        thread.join(timeout=5.0)
        assert not thread.is_alive()
        assert releases[0] != threading.get_native_id()
    else:
        release()
    assert not failures, failures
    after = _wait_cancel(before["cancel_complete"] + 1)
    assert after["unpublished_destroy"] == before["unpublished_destroy"] + 1
    assert after["terminal_enqueues"] == before["terminal_enqueues"] + 1
    assert (
        after["async_cancel_enqueues"] == before["async_cancel_enqueues"] + 1
    )
    assert after["last_enqueue_tid"] == releases[0]
    assert after["cancel_tid"] == after["worker_tid"]
    assert loan.phase == _native.LoanPhase.CANCELLING
    _reject(loan.publish, RuntimeError)
    _reject(loan.cancel, RuntimeError)
    final = _snapshot()
    assert final["terminal_enqueues"] == after["terminal_enqueues"]
    assert final["cancel_complete"] == after["cancel_complete"]
    return {
        "original_exception": str(sentinel),
        "cancel_complete_delta": 1,
        "terminal_enqueues_delta": 1,
        "async_cancel_enqueues_delta": 1,
        "unpublished_destroy_delta": 1,
        "release_tid": releases[0],
        "last_enqueue_tid": after["last_enqueue_tid"],
        "worker_tid": after["worker_tid"],
        "cancel_tid": after["cancel_tid"],
        "terminal_tls": after["terminal_tls"],
    }


def _misuse(session):
    stranger = session._runtime.create_publisher(
        session._topic + "_stranger", depth=1, max_payload_bytes=session._limit
    )
    try:
        loan = session._publisher.borrow(8)
        before = _snapshot()
        _reject(lambda: stranger.publish(loan), ValueError)
        _reject(lambda: stranger.cancel(loan), ValueError)
        errors = []

        def foreign_access():
            for operation in (
                lambda: loan.sequence,
                lambda: loan.buffer,
                loan.publish,
                loan.cancel,
            ):
                try:
                    operation()
                except BaseException as error:
                    errors.append(error)

        thread = threading.Thread(target=foreign_access)
        thread.start()
        thread.join(timeout=5.0)
        assert not thread.is_alive()
        assert len(errors) == 4
        assert all(isinstance(error, RuntimeError) for error in errors)
        assert all("creator thread" in str(error) for error in errors)
        rejected = _snapshot()
        assert rejected["terminal_enqueues"] == before["terminal_enqueues"]
        assert loan.phase == _native.LoanPhase.FILLING
        loan.cancel()
        cancelled = _snapshot()
        _reject(loan.publish, RuntimeError)
        _reject(loan.cancel, RuntimeError)
        assert (
            _snapshot()["terminal_enqueues"] == cancelled["terminal_enqueues"]
        )
        with session._publisher.borrow(8) as current:
            before_stale = _snapshot()
            stale = _reject(
                lambda: _native._test_replay_publish(loan._state), RuntimeError
            )
            assert "generation is stale" in str(stale)
            after_stale = _snapshot()
            for key in (
                "terminal_enqueues", "publish_complete", "cancel_complete"
            ):
                assert after_stale[key] == before_stale[key]
            current.sequence = 812
            assert current.sequence == 812
            current.cancel()
        return {
            "wrong_publisher_rejections": 2,
            "foreign_thread_rejections": 4,
            "duplicate_rejections": 2,
            "stale_rejections": 1,
        }
    finally:
        stranger.close()


def run_lifecycle_case(session, case):
    """Run one production lifecycle scenario without replacing the endpoint."""
    handlers = {
        "PUB01": _affinity,
        "PUB09": _cancel_recovery,
        "PUB11": lambda current: _deferred_cancel(current, False),
        "PUB12": _misuse,
        "PUB18": lambda current: _deferred_cancel(current, True),
    }
    return handlers[case](session)
