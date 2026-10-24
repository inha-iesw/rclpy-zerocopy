"""Generated read-only messages delivered through public rclpy Waitables."""

import inspect
from threading import Lock, Thread
import traceback

from .endpoint import Subscription
from .shutdown import discard_notifier, start_notifier
from .waitable import SubscriptionWaitable
from ..types.message import Message


def _clear_exception_frames(error):
    # Suppressed contexts still retain their frames; explicit causes can also form
    # cycles. Keep the exception graph and stack information while releasing locals.
    pending, seen = [error], set()
    while pending:
        current = pending.pop()
        if current is None or id(current) in seen:
            continue
        seen.add(id(current))
        traceback.clear_frames(current.__traceback__)
        pending.extend((current.__cause__, current.__context__))


class TypedSubscription(Subscription):
    """
    Retain native roots through callbacks, async tasks and independently held aliases.

    Close stops new dispatch and fails while any callback or owner remains. Keep the
    Node in its executor after a busy close so suspended callbacks can finish, then
    retry. Callback exceptions release only the task's reference, preserving aliases.
    Exceptions retain their identity and traceback stack, but completed traceback
    frame locals are cleared so saved exceptions cannot retain message storage.
    """

    def __init__(self, handle, callback, *, runtime_owner, callback_group):
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
        self._waitable = None
        self._registered = False
        self._notifier = Thread(target=self._notify, name='agnocast-notifier', daemon=False)
        try:
            self._waitable = SubscriptionWaitable(self, callback_group)
            self._guard = self._waitable.guard
            self._node.add_waitable(self._waitable)
            self._registered = True
            start_notifier(self)
        except BaseException:
            self._closing = True
            try:
                handle.prepare_close()
                if self._notifier.ident is not None:
                    self._notifier.join()
                self._remove_guard()
                handle.close()
            except BaseException:
                # Preserve the construction failure, including its traceback.
                pass
            finally:
                discard_notifier(self)
            raise

    def _take(self):
        with self._state_lock:
            if self._closing:
                return [None, None]
            if self._notifier_error is not None:
                return [None, self._notifier_error]
            try:
                owner = self._handle.take()
                if owner is None:
                    return [None, None]
                message = Message(owner, readonly=True)
                self._dispatching += 1
                return [message, None]
            except BaseException as error:
                return [None, error]
            finally:
                # Rearm at take, permitting a reentrant group to dispatch while this
                # callback awaits; the native queue serializes rearm with admission.
                self._handle.finish_dispatch()

    async def _execute(self, taken_data):
        message, failure = taken_data
        # The rclpy handler also retains this container in its exception traceback.
        # Consume it so saving an exception cannot retain a second task-owned root.
        taken_data.clear()
        taken_data = None
        if failure is not None:
            _clear_exception_frames(failure)
            raise failure
        if message is None:
            return
        result = None
        try:
            with self._state_lock:
                if self._closing:
                    return
            result = self._callback(message)
            if inspect.isasyncgen(result):
                raise TypeError('subscription callback must not return an async generator')
            if inspect.isawaitable(result):
                await result
        except BaseException as error:
            # Preserve the exception and its stack, but release locals of finished
            # callback frames. User-retained aliases remain independent owners.
            _clear_exception_frames(error)
            raise
        finally:
            # Dropping this reference never invalidates a user-held root, field or view.
            result = None
            message = None
            with self._state_lock:
                self._dispatching -= 1

    def _check_idle(self):
        with self._state_lock:
            stats = self._handle.stats
            if stats.outstanding_leases or self._dispatching:
                raise RuntimeError(
                    'subscription close: active '
                    f'roots={stats.outstanding_leases}, '
                    f'accounted_bytes={stats.outstanding_accounted_bytes}, '
                    f'dispatcher={self._dispatching}; release owners and retry')

    def _remove_guard(self):
        if self._registered:
            self._node.remove_waitable(self._waitable)
            self._registered = False
        if self._guard is not None:
            self._guard.destroy()
            self._guard = None
        self._waitable = None
