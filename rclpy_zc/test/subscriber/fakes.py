"""Shared Node/Guard oracles and explicit Python runtime slot construction."""

from threading import Condition, RLock
from types import SimpleNamespace

from rclpy_agnocast import AgnocastRuntime


class Guard:
    """Record notifier triggers without running the user callback on that thread."""

    def __init__(self, callback):
        self.callback = callback
        self.condition = Condition()
        self.triggers = 0
        self.destroyed = False
        self.triggers_after_destroy = 0

    def trigger(self):
        with self.condition:
            if self.destroyed:
                self.triggers_after_destroy += 1
            assert not self.destroyed
            self.triggers += 1
            self.condition.notify_all()

    def wait(self, count):
        with self.condition:
            assert self.condition.wait_for(lambda: self.triggers >= count, timeout=3)


class Node:
    """Track the public Node guard registry contract."""

    def __init__(self, active=True):
        self.guards = []
        self.context = SimpleNamespace(ok=lambda: active)

    def get_name(self):
        return "python_node"

    def create_guard_condition(self, callback, callback_group):
        assert callback_group is not None
        guard = Guard(callback)
        self.guards.append(guard)
        return guard

    def destroy_guard_condition(self, guard):
        guard.destroyed = True
        self.guards.remove(guard)
        return True


def runtime_for_test(native_runtime, node):
    """Build an unregistered runtime; the calling test owns patching and cleanup."""
    runtime = AgnocastRuntime.__new__(AgnocastRuntime)
    runtime._closed = False
    runtime._native_runtime = native_runtime
    runtime._rclpy_node = node
    runtime._lifecycle_lock = RLock()
    runtime._subscriptions = set()
    return runtime
