"""Exercise the public Node facade against independently typed native participants."""

import argparse
import gc
import os
import sys
import threading
import time

from facade_fields import fill, observe, payload, validate
from process import rejects
from system_runtime import emit


def wait_signal(node):
    """Observe rclpy's installed signal handlers without replacing them in the harness."""
    interrupted = False
    deadline = time.monotonic() + 10
    try:
        emit('SIGNAL_READY')
        while node.context.ok() and time.monotonic() < deadline:
            time.sleep(0.005)
    except KeyboardInterrupt:
        interrupted = True
        while node.context.ok() and time.monotonic() < deadline:
            time.sleep(0.005)
    assert not node.context.ok(), 'rclpy did not shut down after signal'
    emit('SIGNAL', context_shutdown=True, keyboard_interrupt=interrupted)


def publish_commands(node, message_type, args, qos):
    """Publish via facade loans, rejecting ordinary owning messages before delivery."""
    from rclpy_agnocast import _native

    def create():
        return node.create_publisher(
            message_type, args.topic, qos, transport='agnocast', max_message_bytes=65536)

    publisher = create()
    _native._test_enable_observation()
    emit('READY', pid=os.getpid())
    for command in sys.stdin:
        command = command.strip()
        if command == 'quit':
            break
        if command == 'await_signal':
            wait_signal(node)
            break
        if command == 'recreate':
            assert node.destroy_publisher(publisher)
            publisher = create()
            emit('RECREATED')
            continue
        if command != 'publish':
            raise ValueError(command)
        before = _native._test_publisher_snapshot()['publish_complete']
        rejects(TypeError, lambda: publisher.publish(message_type()))
        assert _native._test_publisher_snapshot()['publish_complete'] == before
        with publisher.borrow_message() as message:
            root = _native._test_publisher_snapshot()['message_address']
            assert root != 0
            assert _native._test_is_shared(root)
            fill(message, args.kind)
            accounted = observe(message, args.kind, root)
            publisher.publish(message)
        emit('PUBLISHED', valid=True, accounted_bytes=accounted, pointer_equal=True)


def subscribe_commands(node, message_type, args, qos):
    """Retain only a one-byte alias after dispatch and expose retryable teardown."""
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
    from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
    from rclpy_agnocast import SubscriptionLimits

    held = []
    pending = []
    received = 0
    sibling_received = 0
    errors = []
    failure_expected = threading.Event()
    resume = threading.Event()
    stopping = threading.Event()
    resume.set()
    alias_value = None
    group = MutuallyExclusiveCallbackGroup()

    def callback(message):
        nonlocal received, alias_value
        validate(message, args.kind)
        field = payload(message, args.kind)
        accounted = observe(message, args.kind, message._owner._native_root_address)
        rejects(TypeError, lambda: field.__setitem__(0, 0))
        with field.buffer() as view:
            assert view.readonly
            rejects(TypeError, lambda: view.__setitem__(0, 0))
            held.append(view[1:2])
            alias_value = view[1]
        received += 1
        pending.append((received, accounted))

    def report_received():
        # The shared exclusive group admits this timer only after receive returns.
        # RX therefore proves that only retained aliases, not the callback, own storage.
        while pending:
            count, accounted = pending.pop(0)
            emit('RX', valid=True, count=count, accounted_bytes=accounted, pointer_equal=True)

    def sibling_callback(message):
        nonlocal sibling_received
        validate(message, args.kind)
        observe(message, args.kind, message._owner._native_root_address)
        sibling_received += 1

    def create(callback):
        return node.create_subscription(
            message_type, args.topic, callback, qos, transport='agnocast',
            limits=SubscriptionLimits(max_messages=4, max_retained_bytes=262144,
                                      max_message_bytes=65536), callback_group=group)

    subscription = create(callback)
    node.create_timer(0.01, report_received, callback_group=group)
    executor = SingleThreadedExecutor(context=node.context)
    executor.add_node(node)

    def spin():
        try:
            while not stopping.is_set() and node.context.ok():
                try:
                    executor.spin()
                except MemoryError:
                    if not failure_expected.is_set():
                        raise
                    failure_expected.clear()
                    resume.clear()
                    emit('INJECTED_FAILURE', kind='MemoryError')
                    resume.wait()
        except ExternalShutdownException:
            if node.context.ok():
                raise
        except BaseException as error:
            errors.append(error)
            emit('ERROR', kind=type(error).__name__, detail=str(error))

    thread = threading.Thread(target=spin)
    thread.start()
    emit('READY', pid=os.getpid())
    try:
        for command in sys.stdin:
            command = command.strip()
            if command == 'quit':
                break
            if command == 'await_signal':
                wait_signal(node)
                break
            if errors:
                raise errors[0]
            if command == 'connections':
                count = subscription.publisher_count
                emit('CONNECTED', count=count, connected=count > 0)
            elif command == 'stats':
                stats = subscription.stats
                observation = subscription._handle._observation
                emit('STATS', received=received, retained=stats.outstanding_leases,
                     accounted_bytes=stats.outstanding_accounted_bytes,
                     alias_bytes=sum(len(alias) for alias in held),
                     sibling_received=sibling_received, admitted=stats.received_total,
                     received_refs=observation.received, released_refs=observation.released)
            elif command == 'fail_receive':
                failure_expected.set()
                subscription._handle._test_fail_next_typed_receive()
                emit('ARMED')
            elif command == 'bad_topic':
                # Bypass facade resolution to exercise the generated native factory rollback.
                rejects(RuntimeError, lambda: node._agnocast_runtime.create_typed_subscription(
                    message_type, '/bad topic', callback, depth=4,
                    limits=SubscriptionLimits(4, 262144, 65536), callback_group=group))
                emit('REJECTED')
            elif command == 'add_sibling':
                create(sibling_callback)
                emit('ADDED')
            elif command == 'recreate':
                assert not held
                assert node.destroy_subscription(subscription)
                subscription = create(callback)
                resume.set()
                emit('RECREATED')
            elif command == 'close_busy':
                assert held and held[0][0] == alias_value
                rejects(RuntimeError, lambda: node.destroy_subscription(subscription))
                assert held[0][0] == alias_value
                emit('BUSY', preserved=True)
            elif command == 'release':
                for alias in held:
                    assert alias[0] == alias_value
                    alias.release()
                held.clear()
                gc.collect()
                deadline = time.monotonic() + 5
                while subscription.stats.outstanding_leases and time.monotonic() < deadline:
                    time.sleep(0.005)
                assert subscription.stats.outstanding_leases == 0
                assert subscription.stats.outstanding_accounted_bytes == 0
                emit('RELEASED', retained=0, accounted_bytes=0)
            elif command == 'close':
                assert node.destroy_subscription(subscription)
                emit('CLOSED')
            else:
                raise ValueError(command)
    finally:
        for alias in held:
            assert alias[0] == alias_value
            alias.release()
        held.clear()
        stopping.set()
        resume.set()
        executor.shutdown(timeout_sec=5)
        thread.join(timeout=5)
        assert not thread.is_alive()
        executor.remove_node(node)
    if errors:
        raise errors[0]


def main():
    """Initialize one public Node facade in a separate preloaded process."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=['publisher', 'subscriber'], required=True)
    parser.add_argument('--kind', choices=['Image', 'PointCloud2', 'Containers'], required=True)
    parser.add_argument('--topic', required=True)
    parser.add_argument('--namespace', default='/')
    parser.add_argument('--remap')
    args = parser.parse_args()
    import rclpy
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rclpy_agnocast import Node
    from rclpy_agnocast_test_msgs.msg import Containers
    from sensor_msgs.msg import Image, PointCloud2

    rclpy.init(args=[])
    cli_args = ['--ros-args', '-r', f'{args.topic}:={args.remap}'] if args.remap else None
    node = Node(f'typed_facade_{args.mode}_{os.getpid()}',
                namespace=args.namespace, cli_args=cli_args)
    qos = QoSProfile(depth=4, history=HistoryPolicy.KEEP_LAST,
                     reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)
    message_type = {
        'Image': Image, 'PointCloud2': PointCloud2, 'Containers': Containers}[args.kind]
    try:
        if args.mode == 'publisher':
            publish_commands(node, message_type, args, qos)
        else:
            subscribe_commands(node, message_type, args, qos)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        else:
            rclpy.uninstall_signal_handlers()
    emit('STOPPED')


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        emit('ERROR', kind=type(error).__name__, detail=str(error))
        raise
