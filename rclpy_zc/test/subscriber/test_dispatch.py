"""Device-independent GuardCondition dispatch and shutdown contracts."""

from concurrent.futures import ThreadPoolExecutor
from threading import Event, RLock, Thread

import pytest

from rclpy_agnocast import _native
from rclpy_agnocast.subscription import Subscription
from rclpy_agnocast.subscription import shutdown

from subscriber.fakes import Node


class Runtime:
    """Retain wrappers and serialize their explicit lifecycle operations."""

    def __init__(self):
        self._rclpy_node = Node()
        self._lifecycle_lock = RLock()
        self._subscriptions = set()


@pytest.fixture
def endpoint():
    owned = []

    def create(callback):
        runtime = Runtime()
        handle = _native._TestSubscription(8, 64)
        subscription = Subscription(handle, callback, runtime_owner=runtime)
        runtime._subscriptions.add(subscription)
        owned.append((subscription, runtime._rclpy_node.guards[0]))
        return subscription, handle, runtime._rclpy_node.guards[0]

    yield create
    for subscription, guard in owned:
        subscription.close()
        assert guard.triggers_after_destroy == 0


def test_one_notification_drains_all_messages_then_rearms(endpoint):
    seen = []
    subscription, handle, guard = endpoint(lambda lease: seen.append(lease.sequence))
    for sequence in range(4):
        handle.push(sequence, b"abc")
    guard.wait(1)
    guard.callback()
    assert seen == [0, 1, 2, 3]
    assert guard.triggers == 1
    assert handle.stats.outstanding_leases == 0
    handle.push(4, b"abc")
    guard.wait(2)
    guard.callback()
    assert seen == [0, 1, 2, 3, 4]
    assert handle.release_count == 5
    subscription.close()
    guard.callback()  # A previously scheduled rclpy task may survive guard removal.
    assert seen == [0, 1, 2, 3, 4]


def test_callback_exception_releases_traceback_lease_and_pending_queue(endpoint):
    seen = []

    def callback(lease):
        seen.append(lease.sequence)
        if lease.sequence == 1:
            raise LookupError("callback failed")

    _, handle, guard = endpoint(callback)
    handle.push(1, b"abc")
    handle.push(2, b"def")
    guard.wait(1)
    with pytest.raises(LookupError, match="callback failed") as captured:
        guard.callback()
    assert captured.value.__traceback__ is not None
    assert handle.stats.outstanding_leases == 0
    assert handle.release_count == 2
    handle.push(3, b"ghi")
    guard.wait(2)
    guard.callback()
    assert seen == [1, 3]


def test_dispatch_defers_arrivals_until_next_guard_callback(endpoint):
    seen = []

    def callback(lease):
        sequence = lease.sequence
        seen.append(sequence)
        lease.release()
        # Keep the old unbounded dispatcher finite so a regression fails, not hangs.
        if sequence < 8:
            assert handle.push(sequence + 1, b"abc")

    _, handle, guard = endpoint(callback)
    handle.push(1, b"abc")
    guard.wait(1)
    guard.callback()
    assert seen == [1]
    assert handle.stats.queued_leases == 1
    guard.wait(2)
    guard.callback()
    assert seen == [1, 2]


def test_close_fails_while_dispatcher_runs_even_after_lease_release(endpoint):
    entered, finish = Event(), Event()

    def callback(lease):
        lease.release()
        entered.set()
        assert finish.wait(3)

    subscription, handle, guard = endpoint(callback)
    handle.push(1, b"abc")
    guard.wait(1)
    with ThreadPoolExecutor(max_workers=1) as executor:
        future = executor.submit(guard.callback)
        try:
            assert entered.wait(3)
            assert handle.stats.outstanding_leases == 0
            with pytest.raises(RuntimeError, match="dispatcher=1"):
                subscription.close()
            assert not guard.destroyed
            assert not subscription._notifier.is_alive()
        finally:
            finish.set()
        future.result(timeout=3)
    subscription.close()
    assert guard.destroyed


def test_callback_can_request_close_without_waiting_for_itself(endpoint):
    def callback(lease):
        lease.release()
        with pytest.raises(RuntimeError, match="dispatcher=1"):
            subscription.close()

    subscription, handle, guard = endpoint(callback)
    handle.push(1, b"abc")
    guard.wait(1)
    guard.callback()
    subscription.close()


def test_retained_view_blocks_close_until_release(endpoint):
    views = []
    subscription, handle, guard = endpoint(lambda lease: views.append(lease.buffer))
    handle.push(1, b"abc")
    guard.wait(1)
    guard.callback()
    with pytest.raises(RuntimeError, match="Lease/view=1"):
        subscription.close()
    assert views[0].tolist() == [97, 98, 99]
    assert not guard.destroyed
    views.pop().release()
    subscription.close()
    assert guard.destroyed


def test_native_failure_is_delivered_on_dispatch_thread_and_close_joins(endpoint):
    subscription, handle, guard = endpoint(lambda lease: pytest.fail("unexpected payload"))
    handle.fail()
    guard.wait(1)
    with pytest.raises(RuntimeError, match="injected receive failure"):
        guard.callback()
    with pytest.raises(RuntimeError, match="injected receive failure"):
        _ = subscription.publisher_count
    with pytest.raises(RuntimeError, match="injected receive failure"):
        guard.callback()
    subscription.close()
    assert not subscription._notifier.is_alive()


def test_callback_failure_preserves_an_exported_view_until_explicit_release(endpoint):
    views = []

    def callback(lease):
        views.append(lease.buffer)
        raise LookupError("held view")

    subscription, handle, guard = endpoint(callback)
    handle.push(1, b"abc")
    guard.wait(1)
    with pytest.raises(LookupError, match="held view"):
        guard.callback()
    try:
        with pytest.raises(RuntimeError, match="Lease/view=1"):
            subscription.close()
        assert views[0].tolist() == [97, 98, 99]
    finally:
        views.pop().release()
    subscription.close()


def test_awaitable_callback_result_is_rejected_and_lease_released(endpoint):
    async def deferred():
        pass

    _, handle, guard = endpoint(lambda lease: deferred())
    handle.push(1, b"abc")
    guard.wait(1)
    with pytest.raises(TypeError, match="synchronous"):
        guard.callback()
    assert handle.stats.outstanding_leases == 0


def test_async_generator_result_is_rejected_and_lease_released(endpoint):
    async def deferred():
        yield 1

    _, handle, guard = endpoint(lambda lease: deferred())
    handle.push(1, b"abc")
    guard.wait(1)
    with pytest.raises(TypeError, match="synchronous"):
        guard.callback()
    assert handle.stats.outstanding_leases == 0


def test_base_exception_releases_current_and_pending_leases(endpoint):
    def callback(lease):
        raise KeyboardInterrupt("callback interrupted")

    _, handle, guard = endpoint(callback)
    handle.push(1, b"abc")
    handle.push(2, b"abc")
    guard.wait(1)
    with pytest.raises(KeyboardInterrupt, match="callback interrupted"):
        guard.callback()
    assert handle.stats.outstanding_leases == 0
    assert handle.release_count == 2


def test_shutdown_stops_notifications_without_draining_or_destroying_owners(endpoint, monkeypatch):
    subscription, handle, guard = endpoint(lambda lease: None)
    handle.push(1, b"abc")
    lease = handle.take()
    view = lease.buffer
    lease.release()
    handle.push(2, b"def")
    monkeypatch.setattr(shutdown, "_shutting_down", False)
    try:
        shutdown._stop_notifiers()
        subscription._notifier.join(timeout=3)
        assert not subscription._notifier.is_alive()
        assert not guard.destroyed
        assert handle.stats.queued_leases == 1
        assert handle.stats.outstanding_leases == 2
        assert view.tobytes() == b"abc"
        assert not handle.push(3, b"ghi")
        with pytest.raises(RuntimeError, match="interpreter shutdown"):
            endpoint(lambda lease: None)
    finally:
        view.release()
    subscription.close()
    assert subscription not in shutdown._subscriptions


@pytest.mark.parametrize("failure_stage", ["guard", "thread"])
def test_creation_failure_rolls_back_native_and_guard(monkeypatch, failure_stage):
    runtime = Runtime()
    handle = _native._TestSubscription(1, 8)
    handle.push(1, b"abc")

    def fail(*args, **kwargs):
        raise LookupError("construction failed")

    if failure_stage == "guard":
        monkeypatch.setattr(runtime._rclpy_node, "create_guard_condition", fail)
    else:
        monkeypatch.setattr(Thread, "start", fail)
    with pytest.raises(LookupError, match="construction failed"):
        Subscription(handle, lambda lease: None, runtime_owner=runtime)
    assert runtime._rclpy_node.guards == []
    assert handle.stats.outstanding_leases == 0
    assert not handle.push(2, b"abc")
