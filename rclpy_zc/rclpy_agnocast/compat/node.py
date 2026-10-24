"""rclpy Node facade with explicitly selected generated Agnocast transport."""

import inspect
from threading import RLock

from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node as RclpyNode
from rclpy.publisher import Publisher as RclpyPublisher

from ..runtime import AgnocastRuntime
from ..subscription.limits import SubscriptionLimits
from ..types import resolve_adapter
from .qos import positive_limit, validate_qos


class Node(RclpyNode):
    """Keep DDS defaults and lazily own one process-wide Agnocast runtime.

    Agnocast endpoints require explicit storage limits and supported QoS. Borrowed
    roots are published directly; ordinary ROS message instances are never copied.
    Destroy can fail while a loan, callback or alias is alive. Keep the executor
    running for existing callbacks, release retained owners, and retry destruction.
    A manually owned AgnocastRuntime cannot be mixed with automatic ownership.
    """

    def __init__(self, node_name, **kwargs):
        """Initialize routing before rclpy creates its internal DDS endpoints."""
        self._agnocast_lock = RLock()
        self._agnocast_runtime = None
        self._agnocast_publishers = set()
        self._agnocast_subscriptions = set()
        self._agnocast_closing = False
        self._agnocast_destroyed = False
        super().__init__(node_name, **kwargs)

    @staticmethod
    def _validate_transport(transport):
        if transport not in ('dds', 'agnocast'):
            raise NotImplementedError(f'unsupported transport: {transport!r}')

    def _prepare_endpoint(self, msg_type, topic, qos_profile, callback_group,
                          event_callbacks, qos_overriding_options):
        if self._agnocast_closing:
            raise RuntimeError('Node is closing or destroyed')
        depth = validate_qos(qos_profile)
        if event_callbacks is not None or qos_overriding_options is not None:
            raise NotImplementedError('Agnocast does not support QoS events or overrides')
        if callback_group is not None and not isinstance(callback_group, CallbackGroup):
            raise TypeError('callback_group must be a CallbackGroup')
        if not isinstance(topic, str):
            raise TypeError('topic must be a string')
        if not topic or '\0' in topic:
            raise ValueError('topic must be non-empty and contain no NUL')
        resolved = self.resolve_topic_name(topic)
        resolve_adapter(msg_type)
        return resolved, depth

    def _create_endpoint(self, factory):
        # Caller holds the Node lock across both lazy attachment and registration.
        fresh = self._agnocast_runtime is None
        if fresh:
            self._agnocast_runtime = AgnocastRuntime(self)
        try:
            return factory(self._agnocast_runtime)
        except BaseException:
            if fresh:
                try:
                    self._agnocast_runtime.close()
                except BaseException:
                    # Preserve both the creation error and a runtime that still needs cleanup.
                    pass
                else:
                    self._agnocast_runtime = None
            raise

    def create_publisher(
        self, msg_type, topic, qos_profile, *, callback_group=None, event_callbacks=None,
        qos_overriding_options=None, publisher_class=RclpyPublisher,
        transport='dds', max_message_bytes=None,
    ):
        """Create DDS normally, or a typed loan publisher with an accounted-storage limit.

        ``max_message_bytes`` is mandatory for Agnocast and counts root size and
        native container capacities. It does not bound allocator overhead or RSS.
        """
        self._validate_transport(transport)
        if transport == 'dds':
            if max_message_bytes is not None:
                raise NotImplementedError('max_message_bytes requires Agnocast transport')
            return super().create_publisher(
                msg_type, topic, qos_profile, callback_group=callback_group,
                event_callbacks=event_callbacks, qos_overriding_options=qos_overriding_options,
                publisher_class=publisher_class)
        limit = positive_limit('max_message_bytes', max_message_bytes)
        if publisher_class is not RclpyPublisher:
            raise NotImplementedError('publisher_class is supported only for DDS transport')
        with self._agnocast_lock:
            resolved, depth = self._prepare_endpoint(
                msg_type, topic, qos_profile, callback_group,
                event_callbacks, qos_overriding_options)
            publisher = self._create_endpoint(lambda runtime: runtime.create_typed_publisher(
                msg_type, resolved, depth=depth, max_storage_bytes=limit))
            self._agnocast_publishers.add(publisher)
            return publisher

    def create_subscription(
        self, msg_type, topic, callback, qos_profile, *, callback_group=None,
        event_callbacks=None, qos_overriding_options=None, raw=False,
        transport='dds', limits=None,
    ):
        """Create DDS normally, or dispatch immutable native messages on a public Waitable.

        Agnocast callbacks may be synchronous or async. All retained field/buffer
        aliases count against ``limits`` until the last alias is released.
        """
        self._validate_transport(transport)
        if transport == 'dds':
            if limits is not None:
                raise NotImplementedError('limits requires Agnocast transport')
            return super().create_subscription(
                msg_type, topic, callback, qos_profile, callback_group=callback_group,
                event_callbacks=event_callbacks, qos_overriding_options=qos_overriding_options,
                raw=raw)
        if not isinstance(limits, SubscriptionLimits):
            raise TypeError('Agnocast subscription requires SubscriptionLimits')
        if raw:
            raise NotImplementedError('Agnocast does not support raw serialized callbacks')
        if not callable(callback):
            raise TypeError('callback must be callable')
        if (inspect.isasyncgenfunction(callback) or
                inspect.isasyncgenfunction(getattr(callback, '__call__', None))):
            raise TypeError('async generator callbacks are unsupported')
        with self._agnocast_lock:
            resolved, depth = self._prepare_endpoint(
                msg_type, topic, qos_profile, callback_group,
                event_callbacks, qos_overriding_options)
            subscription = self._create_endpoint(lambda runtime: runtime.create_typed_subscription(
                msg_type, resolved, callback, depth=depth, limits=limits,
                callback_group=callback_group or self.default_callback_group))
            self._agnocast_subscriptions.add(subscription)
            return subscription

    def destroy_publisher(self, publisher):
        """Remove this Node's publisher only after native close succeeds."""
        with self._agnocast_lock:
            if publisher in self._agnocast_publishers:
                publisher.close()
                self._agnocast_publishers.remove(publisher)
                return True
            return super().destroy_publisher(publisher)

    def destroy_subscription(self, subscription):
        """Close a typed endpoint retryably; retained owners keep it registered."""
        with self._agnocast_lock:
            if subscription in self._agnocast_subscriptions:
                subscription.close()
                self._agnocast_subscriptions.remove(subscription)
                return True
            return super().destroy_subscription(subscription)

    def destroy_node(self):
        """Quiesce Agnocast before base Node destruction, preserving async cleanup on busy."""
        with self._agnocast_lock:
            if self._agnocast_destroyed:
                return
            self._agnocast_closing = True
            if self._agnocast_runtime is not None:
                self._agnocast_runtime.close()
                self._agnocast_runtime = None
            self._agnocast_publishers.clear()
            self._agnocast_subscriptions.clear()
            super().destroy_node()
            self._agnocast_destroyed = True
