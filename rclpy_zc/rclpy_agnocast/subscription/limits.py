"""Explicit capacity-based admission limits for generated message subscriptions."""

from dataclasses import dataclass
import sys


@dataclass(frozen=True)
class SubscriptionLimits:
    """
    Bound root count, aggregate accounted storage and each admitted root.

    Accounted bytes include the native root and dynamic capacities, not mapping or
    allocator overhead. Reservations survive callback return while any alias remains.
    """

    max_messages: int
    max_retained_bytes: int
    max_message_bytes: int

    def __post_init__(self):
        """Reject invalid limits before an endpoint or runtime is allocated."""
        for name in ('max_messages', 'max_retained_bytes', 'max_message_bytes'):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int):
                raise TypeError(f'{name} must be an integer')
            if value <= 0 or value > sys.maxsize:
                raise ValueError(f'{name} must be a positive Py_ssize_t')
