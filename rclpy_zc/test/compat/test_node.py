"""Node routing contracts using real rclpy entities and an observed runtime boundary."""

import pytest
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.context import Context
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from rclpy_agnocast import Node, SubscriptionLimits
from rclpy_agnocast.compat import node as node_module


def qos(**changes):
    values = dict(depth=3, reliability=ReliabilityPolicy.BEST_EFFORT)
    values.update(changes)
    return QoSProfile(**values)


class Endpoint:
    def __init__(self):
        self.closed = False
        self.busy = False

    def close(self):
        if self.busy:
            raise RuntimeError('active owner; retry')
        self.closed = True


class Runtime:
    active = None

    def __init__(self, node):
        if Runtime.active is not None:
            raise RuntimeError('already active')
        Runtime.active = self
        self.node = node
        self.calls = []
        self.closed = False
        self.busy = False

    def create_typed_publisher(self, *args, **kwargs):
        self.calls.append(('publisher', args, kwargs))
        return Endpoint()

    def create_typed_subscription(self, *args, **kwargs):
        self.calls.append(('subscription', args, kwargs))
        return Endpoint()

    def close(self):
        if self.busy:
            raise RuntimeError('active owner; retry')
        self.closed = True
        Runtime.active = None


@pytest.fixture
def nodes(monkeypatch, tmp_path):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros'))
    context = Context()
    context.init(args=[])
    Runtime.active = None
    monkeypatch.setattr(node_module, 'AgnocastRuntime', Runtime)
    monkeypatch.setattr(node_module, 'resolve_adapter',
                        lambda cls: {'ros_type': 'std_msgs/msg/String'})
    created = []

    def make(name='facade', **kwargs):
        node = Node(name, context=context, **kwargs)
        created.append(node)
        return node

    yield make
    if Runtime.active:
        Runtime.active.busy = False
    for node in reversed(created):
        node.destroy_node()
    Runtime.active = None
    context.shutdown()


def test_dds_default_keeps_parameter_clock_and_ordinary_endpoints(nodes):
    node = nodes()
    publisher = node.create_publisher(String, 'text', 2)
    subscription = node.create_subscription(String, 'text', lambda msg: None, 2)
    assert publisher in node.publishers
    assert subscription in node.subscriptions
    assert any(pub.topic_name == '/parameter_events' for pub in node.publishers)
    assert node.get_clock().now().nanoseconds > 0
    assert Runtime.active is None
    assert node.destroy_subscription(subscription)
    assert node.destroy_publisher(publisher)


def test_typed_endpoints_resolve_names_without_stock_native_handle_registry(nodes):
    node = nodes(namespace='/robot', cli_args=['--ros-args', '-r', 'input:=output'])
    original = (list(node.publishers), list(node.subscriptions))
    publisher = node.create_publisher(
        String, 'input', qos(), transport='agnocast', max_message_bytes=1024)
    group = ReentrantCallbackGroup()
    limits = SubscriptionLimits(3, 4096, 1024)

    def callback(msg):
        pass

    subscription = node.create_subscription(
        String, 'input', callback, qos(), transport='agnocast', limits=limits,
        callback_group=group)
    runtime = Runtime.active
    assert runtime.calls == [
        ('publisher', (String, '/robot/output'), dict(depth=3, max_storage_bytes=1024)),
        ('subscription', (String, '/robot/output', callback),
         dict(depth=3, limits=limits, callback_group=group)),
    ]
    assert (list(node.publishers), list(node.subscriptions)) == original
    assert node.destroy_subscription(subscription)
    assert node.destroy_publisher(publisher)
    assert not node.destroy_subscription(subscription)
    assert publisher.closed and subscription.closed
    assert not runtime.closed


@pytest.mark.parametrize('profile', [
    3, qos(reliability=ReliabilityPolicy.RELIABLE), qos(history=HistoryPolicy.KEEP_ALL),
    qos(durability=DurabilityPolicy.TRANSIENT_LOCAL), qos(deadline=Duration(seconds=1)),
    qos(lifespan=Duration(seconds=1)), qos(liveliness_lease_duration=Duration(seconds=1)),
    qos(avoid_ros_namespace_conventions=True),
])
@pytest.mark.parametrize('kind', ['publisher', 'subscription'])
def test_unsupported_qos_rejected_before_runtime_or_endpoint(nodes, profile, kind):
    node = nodes()
    initial = list(node.waitables)
    with pytest.raises(NotImplementedError):
        if kind == 'publisher':
            node.create_publisher(String, 'x', profile, transport='agnocast', max_message_bytes=32)
        else:
            node.create_subscription(String, 'x', lambda msg: None, profile, transport='agnocast',
                                     limits=SubscriptionLimits(1, 64, 32))
    assert Runtime.active is None
    assert list(node.waitables) == initial


@pytest.mark.parametrize('option', [
    dict(raw=True), dict(event_callbacks=object()), dict(qos_overriding_options=object())])
def test_unsupported_subscription_options_do_not_allocate(nodes, option):
    node = nodes()
    with pytest.raises(NotImplementedError):
        node.create_subscription(String, 'x', lambda msg: None, qos(), transport='agnocast',
                                 limits=SubscriptionLimits(1, 64, 32), **option)
    assert Runtime.active is None


@pytest.mark.parametrize('limit', [None, 0, -1, True, 1 << 65])
def test_invalid_publisher_limit_rejected_before_runtime(nodes, limit):
    node = nodes()
    with pytest.raises((TypeError, ValueError)):
        node.create_publisher(String, 'x', qos(), transport='agnocast', max_message_bytes=limit)
    assert Runtime.active is None


def test_schema_lookup_failure_does_not_attach_runtime(nodes, monkeypatch):
    node = nodes()

    def missing(cls):
        raise LookupError('no adapter')

    monkeypatch.setattr(node_module, 'resolve_adapter', missing)
    with pytest.raises(LookupError, match='no adapter'):
        node.create_publisher(String, 'x', qos(), transport='agnocast', max_message_bytes=32)
    assert Runtime.active is None


def test_second_agnocast_node_rejected_but_dds_node_stays_usable(nodes):
    first, second = nodes('first'), nodes('second')
    first.create_publisher(String, 'x', qos(), transport='agnocast', max_message_bytes=32)
    runtime = Runtime.active
    second.create_publisher(String, 'ordinary', 2)
    with pytest.raises(RuntimeError, match='already active'):
        second.create_publisher(String, 'y', qos(), transport='agnocast', max_message_bytes=32)
    assert Runtime.active is runtime
    assert len(runtime.calls) == 1


def test_destroy_busy_preserves_node_and_endpoint_until_retry(nodes):
    node = nodes()
    pub = node.create_publisher(String, 'x', qos(), transport='agnocast', max_message_bytes=32)
    pub.busy = True
    with pytest.raises(RuntimeError, match='retry'):
        node.destroy_publisher(pub)
    pub.busy = False
    assert node.destroy_publisher(pub)
    runtime = Runtime.active
    runtime.busy = True
    with pytest.raises(RuntimeError, match='retry'):
        node.destroy_node()
    assert node.get_name() == 'facade'
    assert any(pub.topic_name == '/parameter_events' for pub in node.publishers)
    with pytest.raises(RuntimeError):
        node.create_publisher(String, 'x', qos(), transport='agnocast', max_message_bytes=32)
    runtime.busy = False
    node.destroy_node()
    assert runtime.closed


def test_failed_initial_endpoint_releases_lazy_runtime(nodes, monkeypatch):
    node = nodes()
    native_error = ValueError('factory rejected')

    def failed(*args, **kwargs):
        raise native_error

    monkeypatch.setattr(Runtime, 'create_typed_publisher', failed)
    with pytest.raises(ValueError) as caught:
        node.create_publisher(String, 'x', qos(), transport='agnocast', max_message_bytes=32)
    assert caught.value is native_error
    assert Runtime.active is None


@pytest.mark.parametrize('depth', [0, (1 << 32)])
def test_invalid_native_depth_rejected_before_runtime(nodes, depth):
    node = nodes()
    with pytest.raises(ValueError, match='depth'):
        node.create_publisher(String, 'x', qos(depth=depth), transport='agnocast',
                              max_message_bytes=32)
    assert Runtime.active is None


@pytest.mark.parametrize('options,error', [
    (dict(transport='unknown'), NotImplementedError),
    (dict(limits=None), TypeError),
    (dict(callback=None), TypeError),
    (dict(callback_group=object()), TypeError),
    (dict(topic='bad\0topic'), ValueError),
])
def test_invalid_subscription_contract_never_attaches_runtime(nodes, options, error):
    node = nodes()
    arguments = dict(msg_type=String, topic='x', callback=lambda msg: None,
                     qos_profile=qos(), transport='agnocast', limits=SubscriptionLimits(1, 64, 32))
    arguments.update(options)
    with pytest.raises(error):
        node.create_subscription(**arguments)
    assert Runtime.active is None


def test_second_endpoint_failure_keeps_existing_runtime_and_publisher(nodes, monkeypatch):
    node = nodes()
    publisher = node.create_publisher(
        String, 'x', qos(), transport='agnocast', max_message_bytes=32)
    runtime = Runtime.active

    def failed(*args, **kwargs):
        raise ValueError('factory rejected')

    monkeypatch.setattr(runtime, 'create_typed_subscription', failed)
    with pytest.raises(ValueError, match='factory rejected'):
        node.create_subscription(String, 'x', lambda msg: None, qos(), transport='agnocast',
                                 limits=SubscriptionLimits(1, 64, 32))
    assert Runtime.active is runtime
    assert not runtime.closed
    assert not publisher.closed
    assert node.destroy_publisher(publisher)
