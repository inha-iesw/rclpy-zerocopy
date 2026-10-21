"""Wake non-daemon notifiers before CPython joins threads at interpreter shutdown."""

from threading import Lock, _register_atexit
from weakref import WeakSet


_lock = Lock()
_subscriptions = WeakSet()
_shutting_down = False


def start_notifier(subscription):
    """Register and start atomically with respect to the shutdown snapshot."""
    with _lock:
        if _shutting_down:
            raise RuntimeError("cannot start a subscription during interpreter shutdown")
        _subscriptions.add(subscription)
        try:
            subscription._notifier.start()
        except BaseException:
            _subscriptions.discard(subscription)
            raise


def discard_notifier(subscription):
    """Forget a closed subscription without retaining its runtime until process exit."""
    with _lock:
        _subscriptions.discard(subscription)


def _stop_notifiers():
    global _shutting_down
    with _lock:
        _shutting_down = True
        subscriptions = tuple(_subscriptions)
    for subscription in subscriptions:
        subscription._stop_for_shutdown()


# Ordinary atexit runs after non-daemon joins and cannot wake a blocked notifier.
# CPython 3.10's ThreadPoolExecutor uses this same pre-join hook. Only notification
# is stopped here: explicit close still owns queue, guard and endpoint teardown.
_register_atexit(_stop_notifiers)
