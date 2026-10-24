"""A single native notification guard participating in public rclpy Waitable dispatch."""

from threading import Lock

from rclpy.exceptions import InvalidHandle
from rclpy.guard_condition import GuardCondition
from rclpy.waitable import NumberOfEntities, Waitable


class SubscriptionWaitable(Waitable):
    """Let the executor hold the user's callback group across a callback's awaits."""

    def __init__(self, subscription, callback_group):
        super().__init__(callback_group)
        self._subscription = subscription
        self._ready_lock = Lock()
        self._ready = False
        # This guard belongs only to the Waitable, never also to Node.guards.
        self.guard = GuardCondition(None, callback_group, subscription._node.context)

    def __enter__(self):
        """Keep the guard handle alive while the executor owns its wait set."""
        self.guard.handle.__enter__()
        return self

    def __exit__(self, exception_type, exception, traceback):
        """Release the wait-set reference, permitting deferred handle destruction."""
        self.guard.handle.__exit__(exception_type, exception, traceback)

    def get_num_entities(self):
        """Contribute exactly one guard condition to the executor wait set."""
        return NumberOfEntities(num_gcs=1)

    def add_to_wait_set(self, wait_set):
        """Register the exclusively owned notification handle."""
        wait_set.add_guard_condition(self.guard.handle)
        with self._ready_lock:
            # A sibling can acquire the group after this waitable entered a wait
            # set. Guard readiness is auto-taken, so preserve it until take_data.
            if self._ready:
                try:
                    self.guard.trigger()
                except InvalidHandle:
                    # Close may request deferred destruction after __enter__.
                    pass

    def is_ready(self, wait_set):
        """Check the native guard readiness while its wait set remains alive."""
        ready = self.guard.handle.pointer in wait_set.get_ready_entities('guard_condition')
        with self._ready_lock:
            self._ready = self._ready or ready
            return self._ready

    def take_data(self):
        """Take one root; carry errors into execute so callback-group cleanup runs."""
        with self._ready_lock:
            self._ready = False
        return self._subscription._take()

    async def execute(self, taken_data):
        """Run or await the callback without bypassing executor callback groups."""
        try:
            await self._subscription._execute(taken_data)
        finally:
            taken_data.clear()
            taken_data = None
