"""Public subscription validation and runtime-wide staged cleanup."""

import pytest

from rclpy_agnocast import _native
from rclpy_agnocast import runtime as runtime_module

from subscriber.fakes import Node, runtime_for_test


class NativeRuntime:
    """Use the real bounded queue while recording outer lifecycle ordering."""

    def __init__(self):
        self.created = []
        self.events = []
        self.busy = False
        self.state = _native.RuntimeState.OPEN

    def create_subscription(self, topic, depth, count, limit):
        if self.state != _native.RuntimeState.OPEN:
            raise RuntimeError("runtime is not open")
        handle = _native._TestSubscription(count, limit)
        self.created.append((topic, depth, count, limit, handle))
        return handle

    def prepare_close(self):
        self.state = _native.RuntimeState.QUIESCING
        for *_, handle in self.created:
            handle.prepare_close()
        if self.busy:
            raise RuntimeError("active Loan keeps the publisher worker busy")
        self.events.append("workers stopped")

    def close(self):
        for *_, handle in self.created:
            handle.close()
        self.events.append("native destroyed")
        self.state = _native.RuntimeState.CLOSED


@pytest.fixture
def runtime(monkeypatch):
    node = Node()
    value = runtime_for_test(NativeRuntime(), node)
    destroy = node.destroy_guard_condition

    def remove_guard(guard):
        assert value._native_runtime.state != _native.RuntimeState.CLOSED
        assert all(not sub._notifier.is_alive() for sub in value._subscriptions)
        value._native_runtime.events.append("guard removed")
        return destroy(guard)

    node.destroy_guard_condition = remove_guard
    monkeypatch.setattr(runtime_module, "_active_runtime", value)
    yield value
    value._native_runtime.busy = False
    value.close()


@pytest.mark.parametrize("field,value,error", [
    ("topic", "", ValueError), ("topic", 1, TypeError),
    ("callback", None, TypeError), ("depth", 0, ValueError),
    ("depth", True, TypeError), ("depth", 1.5, TypeError),
    ("max_outstanding_leases", -1, ValueError),
    ("max_outstanding_leases", False, TypeError),
    ("max_payload_bytes", 0, ValueError), ("max_payload_bytes", "8", TypeError),
])
def test_invalid_subscription_arguments_fail_before_native_creation(runtime, field, value, error):
    arguments = dict(topic="/test", callback=lambda lease: None, depth=2, max_payload_bytes=64)
    arguments[field] = value
    with pytest.raises(error):
        runtime.create_subscription(**arguments)
    assert runtime._native_runtime.created == []


def test_async_callback_is_rejected_before_native_creation(runtime):
    async def callback(lease):
        pass

    with pytest.raises(TypeError, match="synchronous"):
        runtime.create_subscription("/test", callback, depth=1, max_payload_bytes=8)
    assert runtime._native_runtime.created == []


@pytest.mark.parametrize("callable_object", [False, True])
def test_async_generator_callback_is_rejected_before_native_creation(runtime, callable_object):
    async def callback(lease):
        yield lease

    class Callback:
        __call__ = staticmethod(callback)

    with pytest.raises(TypeError, match="synchronous"):
        runtime.create_subscription("/test", Callback() if callable_object else callback,
                                    depth=1, max_payload_bytes=8)
    assert runtime._native_runtime.created == []


def test_default_count_and_runtime_shutdown_order(runtime):
    node = runtime._rclpy_node
    native = runtime._native_runtime
    subscription = runtime.create_subscription("/test", lambda lease: None,
                                               depth=3, max_payload_bytes=16)
    assert native.created[0][:4] == ("/test", 3, 3, 16)
    assert len(node.guards) == len(runtime._subscriptions) == 1
    native.created[0][4].push(1, b"abc")
    runtime.close()
    assert native.events == ["workers stopped", "guard removed", "native destroyed"]
    assert not subscription._notifier.is_alive()
    assert runtime._subscriptions == set()
    assert node.guards == []
    assert runtime_module._active_runtime is None
    runtime.close()
    subscription.close()
    assert native.events == ["workers stopped", "guard removed", "native destroyed"]


def test_busy_owner_stops_all_notifiers_and_preserves_guards_until_retry(runtime):
    node = runtime._rclpy_node
    first = runtime.create_subscription("/a", lambda lease: None, depth=2, max_payload_bytes=8)
    second = runtime.create_subscription("/b", lambda lease: None, depth=2, max_payload_bytes=8)
    first._handle.push(1, b"abc")
    lease = first._handle.take()
    second._handle.push(1, b"abc")
    other = second._handle.take()
    first._handle.push(2, b"def")
    second._handle.push(2, b"def")
    try:
        with pytest.raises(RuntimeError, match="Lease/view=1"):
            runtime.close()
        assert not first._notifier.is_alive() and not second._notifier.is_alive()
        assert first.stats.queued_leases == second.stats.queued_leases == 0
        assert len(node.guards) == 2
        assert runtime_module._active_runtime is runtime
        assert runtime.state == _native.RuntimeState.QUIESCING
        with pytest.raises(RuntimeError, match="not open"):
            runtime.create_subscription("/c", lambda lease: None, depth=1, max_payload_bytes=8)
    finally:
        lease.release()
        other.release()
    runtime.close()
    assert not node.guards


def test_active_loan_close_error_keeps_runtime_for_retry(runtime):
    subscription = runtime.create_subscription("/test", lambda lease: None,
                                               depth=1, max_payload_bytes=8)
    runtime._native_runtime.busy = True
    with pytest.raises(RuntimeError, match="active Loan"):
        runtime.close()
    assert runtime_module._active_runtime is runtime
    assert subscription._closing
    assert runtime._rclpy_node.guards
    runtime._native_runtime.busy = False
    runtime.close()
    assert subscription._closed


def test_native_close_failure_after_guard_removal_can_be_retried(runtime, monkeypatch):
    subscription = runtime.create_subscription("/test", lambda lease: None,
                                               depth=1, max_payload_bytes=8)
    close = runtime._native_runtime.close

    def fail():
        raise LookupError("native close failed")

    with monkeypatch.context() as patch:
        patch.setattr(runtime._native_runtime, "close", fail)
        with pytest.raises(LookupError, match="native close failed"):
            runtime.close()
    assert runtime._rclpy_node.guards == []
    assert subscription in runtime._subscriptions
    assert not subscription._closed
    assert not subscription._notifier.is_alive()
    assert runtime._native_runtime.close == close
    runtime.close()
    assert subscription._closed
    assert runtime._subscriptions == set()
