"""Synchronous rclpy dispatch and explicit subscriber resource lifetime."""

import inspect
from threading import Lock, Thread

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from .shutdown import discard_notifier, start_notifier


def _close_all(subscriptions, prepare, destroy):
    # Each phase must finish for all endpoints before a busy endpoint can abort close.
    for subscription in subscriptions:
        subscription._mark_closing()
    prepare()
    for subscription in subscriptions:
        subscription._join_and_drain()
    for subscription in subscriptions:
        subscription._check_idle()
    for subscription in subscriptions:
        subscription._remove_guard()
    destroy()
    for subscription in subscriptions:
        subscription._finish_close()


class Subscription:
    """
    Deliver read-only Leases on the rclpy executor through a private callback group.

    Retained Leases/views keep native storage alive and make close retryably fail.
    A callback failure releases its Lease and drops the pending queue, then propagates
    through rclpy; subsequent messages can be dispatched normally. Call close before
    destroying the rclpy Node or shutting down its context. Each dispatch handles at
    most the initially queued count so other executor callbacks can run under load.
    Native/notifier failures remain terminal until close, including connectivity access.
    Interpreter shutdown only stops notifications; it does not replace explicit close.
    """

    def __init__(self, handle, callback, *, runtime_owner):
        self._handle = handle
        self._callback = callback
        self._runtime_owner = runtime_owner
        self._node = runtime_owner._rclpy_node
        self._state_lock = Lock()
        self._closing = False
        self._closed = False
        self._dispatching = 0
        self._notifier_error = None
        self._guard = None
        self._notifier = Thread(target=self._notify, name="agnocast-notifier", daemon=False)
        try:
            self._guard = self._node.create_guard_condition(
                self._dispatch, callback_group=MutuallyExclusiveCallbackGroup()
            )
            start_notifier(self)
        except BaseException:
            self._closing = True
            handle.prepare_close()
            if self._notifier.ident is not None:
                self._notifier.join()
            self._remove_guard()
            handle.close()
            discard_notifier(self)
            raise

    @property
    def stats(self):
        """Return one native accounting snapshot, including queued and retained storage."""
        return self._handle.stats

    @property
    def publisher_count(self):
        """Return native graph connectivity while this subscription remains open."""
        with self._state_lock:
            if self._closing:
                raise RuntimeError("subscription is closing or closed")
            if self._notifier_error is not None:
                raise self._notifier_error
            return self._handle.publisher_count

    def _notify(self):
        generation = 0
        try:
            while True:
                generation = self._handle.wait_for_wake(generation)
                if generation == 0:
                    return
                with self._state_lock:
                    if self._closing:
                        return
                    self._guard.trigger()
        except BaseException as error:
            with self._state_lock:
                if self._closing:
                    return
                self._notifier_error = error
                try:
                    self._guard.trigger()
                except Exception:
                    # A destroyed Node/context cannot dispatch an error. Connectivity
                    # access still exposes it, and explicit close can release resources.
                    pass

    def _dispatch(self):
        with self._state_lock:
            # rclpy can retain a scheduled task after Node removes its guard.
            if self._closing:
                return
            self._dispatching += 1
            failure = self._notifier_error
        lease = None
        try:
            if failure is not None:
                raise failure
            # Arrivals during this batch are re-notified by finish_dispatch().
            for _ in range(self._handle.stats.queued_leases):
                with self._state_lock:
                    if self._closing:
                        break
                    lease = self._handle.take()
                if lease is None:
                    break
                result = self._callback(lease)
                if inspect.isawaitable(result) or inspect.isasyncgen(result):
                    if inspect.iscoroutine(result):
                        result.close()
                    raise TypeError("subscription callback must be synchronous")
                del result
                # Successful callbacks may retain the Lease; do not invalidate their owner.
                lease = None
        except BaseException:
            if lease is not None:
                lease.release()
            self._handle.clear_pending()
            raise
        finally:
            self._handle.finish_dispatch()
            with self._state_lock:
                self._dispatching -= 1

    def _mark_closing(self):
        with self._state_lock:
            self._closing = True

    def _stop_for_shutdown(self):
        self._mark_closing()
        self._handle.stop_delivery()

    def _join_and_drain(self):
        self._notifier.join()
        self._handle.clear_pending()

    def _check_idle(self):
        with self._state_lock:
            stats = self._handle.stats
            if stats.outstanding_leases or self._dispatching:
                raise RuntimeError(
                    "subscription close: active "
                    f"Lease/view={stats.outstanding_leases}, "
                    f"payload_bytes={stats.outstanding_payload_bytes}, "
                    f"dispatcher={self._dispatching}; release owners and retry"
                )

    def _remove_guard(self):
        if self._guard is not None:
            self._node.destroy_guard_condition(self._guard)
            self._guard = None

    def _finish_close(self):
        discard_notifier(self)
        self._closed = True
        self._callback = None
        self._notifier_error = None
        self._node = None
        self._runtime_owner._subscriptions.discard(self)
        self._runtime_owner = None

    def close(self):
        """
        Permanently stop delivery and remove the endpoint when no owners/dispatch remain.

        A busy failure still stops future delivery. Release retained Leases/views and
        retry after dispatch returns. Calling from a callback fails busy; retry outside it.
        """
        owner = self._runtime_owner
        if owner is None:
            return
        with owner._lifecycle_lock:
            if self._closed:
                return
            _close_all((self,), self._handle.prepare_close, self._handle.close)
