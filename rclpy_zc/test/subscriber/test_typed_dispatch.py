"""Real Humble executor dispatch with native heap-backed typed subscription owners."""

import gc
import os
from threading import Event, RLock, Thread
import time
from types import SimpleNamespace

import pytest
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.task import Future

from rclpy_agnocast import _native
from rclpy_agnocast.runtime import AgnocastRuntime
from rclpy_agnocast.subscription.typed import TypedSubscription


def spin_until(executor, predicate):
    deadline = time.monotonic() + 3
    while not predicate():
        assert time.monotonic() < deadline, 'executor did not make the expected progress'
        executor.spin_once(timeout_sec=0.01)


@pytest.fixture
def harness(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros-log'))
    context = Context()
    rclpy.init(context=context)
    node = rclpy.create_node(f'typed_{os.getpid()}', context=context)
    owner = AgnocastRuntime.__new__(AgnocastRuntime)
    owner._rclpy_node = node
    owner._subscriptions = set()
    owner._lifecycle_lock = RLock()
    owner._closed = False

    def prepare_close():
        owner._native_runtime.state = _native.RuntimeState.QUIESCING
        for subscription in owner._subscriptions:
            subscription._handle.prepare_close()

    def close():
        prepare_close()
        for subscription in owner._subscriptions:
            subscription._handle.close()
        owner._native_runtime.state = _native.RuntimeState.CLOSED

    owner._native_runtime = SimpleNamespace(
        prepare_close=prepare_close, close=close, state=_native.RuntimeState.OPEN)
    executors, gates = [], []

    def create(callback, *, group=None, threads=1, count=8, retained=65536, per_message=65536):
        executor = (SingleThreadedExecutor(context=context) if threads == 1 else
                    MultiThreadedExecutor(num_threads=threads, context=context))
        executors.append(executor)
        executor.add_node(node)
        handle = _native._TestTypedSubscription(count, retained, per_message)
        subscription = TypedSubscription(
            handle, callback, runtime_owner=owner,
            callback_group=group or node.default_callback_group)
        owner._subscriptions.add(subscription)
        return subscription, handle, executor

    def gate():
        future = Future()
        gates.append(future)
        return future

    yield SimpleNamespace(create=create, gate=gate, node=node, runtime=owner)

    for future in gates:
        if not future.done() and not future.cancelled():
            future.set_result(None)
    for executor in executors:
        spin_until(executor, lambda: all(sub._dispatching == 0 for sub in owner._subscriptions))
        assert executor.shutdown(timeout_sec=3)
    gc.collect()
    for subscription in tuple(owner._subscriptions):
        subscription.close()
    owner.close()
    node.destroy_node()
    rclpy.shutdown(context=context)


def test_waitable_owns_one_guard_and_preserves_nested_alias_budget(harness):
    fields = []
    seen = []

    def callback(message):
        seen.append((message.signed, message.text.to_text(), message.wide.to_text()))
        fields.append(message.children[0].values)
        with message.values.buffer() as view:
            assert view.readonly
            assert view.tolist() == [10, 11, 12]
            with pytest.raises(TypeError):
                view[0] = 1
        with pytest.raises(TypeError, match='read.only'):
            message.signed = 2

    before_guards = tuple(harness.node.guards)
    before_waitables = tuple(harness.node.waitables)
    subscription, handle, executor = harness.create(callback)
    assert tuple(harness.node.guards) == before_guards
    assert len(tuple(harness.node.waitables)) == len(before_waitables) + 1
    assert handle.push(10)
    expected_bytes = handle.last_accounted_bytes
    spin_until(executor, lambda: len(fields) == 1 and subscription._dispatching == 0)
    assert seen == [(10, 'typed-10', 'wide')]
    assert handle.stats.outstanding_leases == 1
    assert handle.stats.outstanding_accounted_bytes == expected_bytes
    assert fields[0][1] == 21
    with pytest.raises(RuntimeError, match='roots=1'):
        subscription.close()
    assert subscription._waitable in tuple(harness.node.waitables)
    fields.clear()
    assert handle.release_count == 1
    assert handle.stats.outstanding_accounted_bytes == 0
    subscription.close()
    assert tuple(harness.node.waitables) == before_waitables


@pytest.mark.parametrize('threads', [1, 4])
@pytest.mark.parametrize('reentrant', [False, True])
def test_callback_group_controls_overlap_across_await(harness, threads, reentrant):
    gate = harness.gate()
    entered, completed, ticks = [], [], []
    group = ReentrantCallbackGroup() if reentrant else MutuallyExclusiveCallbackGroup()

    async def callback(message):
        entered.append(message.signed)
        await gate
        completed.append(message.signed)

    subscription, handle, executor = harness.create(callback, group=group, threads=threads)
    timer = harness.node.create_timer(
        0.001, lambda: ticks.append(1), callback_group=ReentrantCallbackGroup())
    assert handle.push(1)
    spin_until(executor, lambda: entered == [1])
    assert handle.push(2)
    if reentrant:
        spin_until(executor, lambda: len(entered) == 2)
        assert subscription._dispatching == 2
    else:
        start_ticks = len(ticks)
        spin_until(executor, lambda: len(ticks) >= start_ticks + 3)
        assert entered == [1]
        assert subscription._dispatching == 1
    assert not completed
    assert handle.stats.outstanding_leases == 2
    gate.set_result(None)
    spin_until(executor, lambda: len(completed) == 2 and subscription._dispatching == 0)
    assert sorted(completed) == [1, 2]
    assert handle.stats.outstanding_leases == 0
    harness.node.destroy_timer(timer)


@pytest.mark.parametrize('close_runtime', [False, True])
def test_busy_close_keeps_async_continuation_and_alias_until_retry(harness, close_runtime):
    gate = harness.gate()
    aliases, entered, completed = [], [], []

    async def callback(message):
        aliases.append(message.values.buffer())
        entered.append(message.signed)
        await gate
        completed.append(message.signed)

    subscription, handle, executor = harness.create(callback)
    close = harness.runtime.close if close_runtime else subscription.close
    assert handle.push(4)
    spin_until(executor, lambda: entered == [4])
    assert handle.push(5)
    with pytest.raises(RuntimeError, match='dispatcher=1'):
        close()
    if close_runtime:
        assert harness.runtime.state == _native.RuntimeState.QUIESCING
        assert harness.runtime._rclpy_node is harness.node
    assert handle.stats.queued_leases == 0
    assert not subscription._notifier.is_alive()
    assert subscription._waitable in tuple(harness.node.waitables)
    gate.set_result(None)
    spin_until(executor, lambda: completed == [4] and subscription._dispatching == 0)
    assert entered == [4]
    assert aliases[0].tolist() == [4, 5, 6]
    with pytest.raises(RuntimeError, match='roots=1'):
        close()
    aliases.pop().release()
    close()
    if close_runtime:
        assert harness.runtime.state == _native.RuntimeState.CLOSED
        assert harness.runtime._rclpy_node is None


def test_cancelling_executor_task_releases_dispatch_but_preserves_user_alias(harness):
    gate = harness.gate()
    aliases, cancelled = [], []

    async def callback(message):
        aliases.append(message.values)
        try:
            await gate
        finally:
            cancelled.append(True)

    subscription, handle, executor = harness.create(callback)
    assert handle.push(9)
    task, entity, _ = executor.wait_for_ready_callbacks(timeout_sec=3)
    assert entity is subscription._waitable
    task()
    assert subscription._dispatching == 1
    task.cancel()
    assert cancelled == [True]
    assert subscription._dispatching == 0
    assert handle.stats.outstanding_leases == 1
    assert aliases[0][0] == 9
    assert subscription._waitable.callback_group.can_execute(subscription._waitable)
    aliases.clear()
    assert handle.stats.outstanding_leases == 0
    gate.set_result(None)
    assert handle.push(10)
    spin_until(executor, lambda: len(cancelled) == 2 and subscription._dispatching == 0)
    assert aliases[0][0] == 10
    aliases.clear()
    assert handle.stats.outstanding_leases == 0


def test_callback_can_request_close_without_self_join(harness):
    seen = []

    def callback(message):
        with pytest.raises(RuntimeError, match='dispatcher=1'):
            subscription.close()
        seen.append(message.signed)

    subscription, handle, executor = harness.create(callback)
    assert handle.push(3)
    spin_until(executor, lambda: seen == [3] and subscription._dispatching == 0)
    subscription.close()


@pytest.mark.parametrize('alias_kind', ['root', 'field', 'view'])
@pytest.mark.parametrize('async_callback', [False, True])
def test_callback_exception_does_not_invalidate_retained_alias_or_drop_next_root(
        harness, alias_kind, async_callback):
    aliases, seen = [], []

    def callback(message):
        if message.signed == 1:
            aliases.append(message if alias_kind == 'root' else (
                message.values if alias_kind == 'field' else message.values.buffer()))
            raise LookupError('typed callback sentinel')
        seen.append(message.signed)

    async def deferred(message):
        callback(message)

    subscription, handle, executor = harness.create(deferred if async_callback else callback)
    assert handle.push(1)
    assert handle.push(2)
    with pytest.raises(LookupError, match='typed callback sentinel') as captured:
        spin_until(executor, lambda: bool(seen))
    spin_until(executor, lambda: seen == [2] and subscription._dispatching == 0)
    gc.collect()
    assert (aliases[0].values[2] if alias_kind == 'root' else aliases[0][2]) == 3
    assert handle.stats.outstanding_leases == 1
    if alias_kind == 'view':
        aliases[0].release()
    aliases.clear()
    assert captured.value.__traceback__ is not None
    assert any(frame.name == 'callback' for frame in captured.traceback)
    assert handle.stats.outstanding_leases == 0


@pytest.mark.parametrize('chain', ['cause', 'context', 'suppressed', 'cycle'])
def test_saved_exception_chain_releases_task_owner_after_last_alias(harness, chain):
    aliases = []

    def helper(message):
        assert message.signed == 12
        raise ValueError('inner callback failure')

    def callback(message):
        aliases.append(message.values)
        try:
            helper(message)
        except ValueError as inner:
            outer = LookupError('outer callback failure')
            if chain == 'cycle':
                inner.__cause__ = outer
            if chain in ('cause', 'cycle'):
                raise outer from inner
            if chain == 'suppressed':
                raise outer from None
            raise outer

    subscription, handle, executor = harness.create(callback)
    assert handle.push(12)
    with pytest.raises(LookupError, match='outer callback failure') as captured:
        spin_until(executor, lambda: False)
    inner = captured.value.__context__
    assert isinstance(inner, ValueError)
    assert inner.__traceback__ is not None
    assert str(inner) == 'inner callback failure'
    if chain in ('cause', 'cycle'):
        assert captured.value.__cause__ is inner
    if chain == 'cycle':
        assert inner.__cause__ is captured.value
    assert subscription._dispatching == 0
    assert aliases[0][0] == 12
    assert handle.stats.outstanding_leases == 1
    aliases.clear()
    assert handle.stats.outstanding_leases == 0
    assert handle.release_count == 1


def test_guard_readiness_survives_callback_group_becoming_busy(harness):
    seen = []
    group = MutuallyExclusiveCallbackGroup()
    subscription, handle, executor = harness.create(lambda message: seen.append(message.signed),
                                                    group=group)
    waitable = subscription._waitable
    is_ready = waitable.is_ready
    acquired = []

    def lose_group_after_ready(wait_set):
        ready = is_ready(wait_set)
        if ready and not acquired:
            assert group.beginning_execution(waitable)
            acquired.append(True)
        return ready

    waitable.is_ready = lose_group_after_ready
    assert handle.push(6)
    # A different group makes the spin return after the guard was auto-taken.
    timer = harness.node.create_timer(
        0.001, lambda: None, callback_group=ReentrantCallbackGroup())
    spin_until(executor, lambda: bool(acquired))
    assert seen == []
    group.ending_execution(waitable)
    spin_until(executor, lambda: seen == [6])
    harness.node.destroy_timer(timer)


def test_scheduled_waitable_after_close_never_calls_user_callback(harness):
    subscription, handle, executor = harness.create(
        lambda message: pytest.fail('closed endpoint dispatched a message'))
    assert handle.push(7)
    task, entity, _ = executor.wait_for_ready_callbacks(timeout_sec=3)
    assert entity is subscription._waitable
    subscription.close()
    task()
    assert task.done()
    assert task.exception() is None
    assert handle.stats.outstanding_leases == 0
    assert handle.release_count == 1


def test_take_error_runs_group_cleanup_and_rearms_the_queued_root(harness):
    seen = []
    subscription, handle, executor = harness.create(lambda message: seen.append(message.signed))

    class FailOnce:
        failed = False

        def __getattr__(self, name):
            return getattr(handle, name)

        def take(self):
            if not self.failed:
                self.failed = True
                raise MemoryError('typed take sentinel')
            return handle.take()

    subscription._handle = FailOnce()
    assert handle.push(8)
    with pytest.raises(MemoryError, match='typed take sentinel'):
        spin_until(executor, lambda: bool(seen))
    assert subscription._waitable.callback_group.can_execute(subscription._waitable)
    spin_until(executor, lambda: seen == [8])
    assert handle.stats.outstanding_leases == 0


def test_notifier_start_failure_removes_waitable_and_releases_native_queue(harness, monkeypatch):
    from rclpy_agnocast.subscription import typed

    handle = _native._TestTypedSubscription(1, 65536, 65536)
    assert handle.push(11)
    before = tuple(harness.node.waitables)

    def fail_start(subscription):
        raise LookupError('notifier start sentinel')

    monkeypatch.setattr(typed, 'start_notifier', fail_start)
    with pytest.raises(LookupError, match='notifier start sentinel'):
        TypedSubscription(handle, lambda message: None, runtime_owner=harness.runtime,
                          callback_group=harness.node.default_callback_group)
    assert tuple(harness.node.waitables) == before
    assert handle.stats.outstanding_leases == 0
    assert handle.release_count == 1


def test_quiescing_runtime_allows_task_cancel_then_owner_release_and_retry(harness):
    gate = harness.gate()
    aliases = []

    async def callback(message):
        aliases.append(message.values)
        await gate

    subscription, handle, executor = harness.create(callback)
    assert handle.push(20)
    task, entity, _ = executor.wait_for_ready_callbacks(timeout_sec=3)
    assert entity is subscription._waitable
    task()
    with pytest.raises(RuntimeError, match='dispatcher=1'):
        harness.runtime.close()
    assert harness.runtime.state == _native.RuntimeState.QUIESCING
    task.cancel()
    assert subscription._dispatching == 0
    assert aliases[0][0] == 20
    with pytest.raises(RuntimeError, match='roots=1'):
        harness.runtime.close()
    aliases.clear()
    harness.runtime.close()
    assert harness.runtime.state == _native.RuntimeState.CLOSED


@pytest.mark.parametrize('threads', [1, 4])
def test_sustained_native_burst_allows_timer_progress_and_drains_on_close(harness, threads):
    seen, ticks = [], []
    stopped = Event()

    def callback(message):
        seen.append(message.signed)
        time.sleep(0.001)
        if not stopped.is_set():
            assert handle.push(message.signed + 4)

    subscription, handle, executor = harness.create(callback, threads=threads)
    timer = harness.node.create_timer(
        0.001, lambda: ticks.append(1), callback_group=ReentrantCallbackGroup())
    for seed in range(4):
        assert handle.push(seed)
    try:
        spin_until(executor, lambda: len(seen) >= 30 and len(ticks) >= 3)
        assert seen == list(range(len(seen)))
        assert handle.stats.queued_leases > 0
    finally:
        stopped.set()
    spin_until(executor, lambda: subscription._dispatching == 0)
    subscription.close()
    assert handle.stats.outstanding_leases == 0
    assert handle.stats.queued_leases == 0
    harness.node.destroy_timer(timer)


def test_concurrent_native_arrival_after_each_drain_never_loses_wake(harness):
    consumed, finished = Event(), Event()
    seen, errors = [], []

    def callback(message):
        seen.append(message.signed)
        consumed.set()

    subscription, handle, executor = harness.create(callback)

    def produce():
        try:
            for seed in range(100):
                assert handle.push(seed)
                assert consumed.wait(3)
                consumed.clear()
        except BaseException as error:
            errors.append(error)
        finally:
            finished.set()

    producer = Thread(target=produce)
    producer.start()
    try:
        spin_until(executor, lambda: finished.is_set())
        assert errors == []
        assert seen == list(range(100))
    finally:
        consumed.set()
        producer.join(timeout=3)
        assert not producer.is_alive()
    spin_until(executor, lambda: subscription._dispatching == 0)
