"""Validate the supported transport contract before native endpoint allocation."""

import sys

from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, LivelinessPolicy, QoSProfile, ReliabilityPolicy,
)


def positive_limit(name, value, maximum=sys.maxsize):
    """Accept only positive integer limits representable at the native boundary."""
    if isinstance(value, bool) or not isinstance(value, int):
        raise TypeError(f'{name} must be an integer')
    if not 1 <= value <= maximum:
        raise ValueError(f'{name} must be between 1 and {maximum}')
    return value


def validate_qos(profile):
    """Return KeepLast depth only for explicit best-effort, volatile QoS."""
    if isinstance(profile, int):
        raise NotImplementedError('Agnocast requires an explicit BEST_EFFORT QoSProfile')
    if not isinstance(profile, QoSProfile):
        raise TypeError('qos_profile must be a QoSProfile')
    if (profile.history != HistoryPolicy.KEEP_LAST or
            profile.reliability != ReliabilityPolicy.BEST_EFFORT or
            profile.durability != DurabilityPolicy.VOLATILE):
        raise NotImplementedError('Agnocast supports KEEP_LAST/BEST_EFFORT/VOLATILE only')
    for field in ('deadline', 'lifespan', 'liveliness_lease_duration'):
        if getattr(profile, field).nanoseconds != 0:
            raise NotImplementedError(f'Agnocast does not support {field}')
    if profile.liveliness not in (LivelinessPolicy.SYSTEM_DEFAULT, LivelinessPolicy.AUTOMATIC):
        raise NotImplementedError('Agnocast does not support manual liveliness')
    if profile.avoid_ros_namespace_conventions:
        raise NotImplementedError('Agnocast requires ROS namespace conventions')
    return positive_limit('depth', profile.depth, (1 << 32) - 1)
