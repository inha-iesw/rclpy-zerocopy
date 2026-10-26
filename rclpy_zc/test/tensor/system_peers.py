"""Independent public rclpy peers for the CPU Tensor device checkpoint."""

import argparse
import gc
import os
import sys
import threading
import time

from subscriber.checks import receipt_settled
from system_runtime import emit
from write_fault import assert_write_fault


def publisher_commands(node, message_type, topic, qos):
    """Fill the actual native Image data storage through a writable Tensor."""
    publisher = node.create_publisher(
        message_type, topic, qos, transport='agnocast', max_message_bytes=65536)
    emit('READY', pid=os.getpid())
    for command in sys.stdin:
        words = command.split()
        if words == ['quit']:
            break
        assert len(words) == 2 and words[0] in ('publish', 'publish_alias')
        sequence = int(words[1])
        rows, columns = 2 + sequence % 4, 3 + sequence % 13
        with publisher.borrow_message() as message:
            message.data.resize(rows * columns)
            message.encoding.resize(5)
            message.header.stamp.sec = sequence
            message.height, message.width, message.step = rows, columns, columns
            with message.encoding.buffer() as encoding:
                encoding.cast('B')[:] = b'mono8'
            field = message.data
            source = field.buffer()
            assert not source.readonly
            tensor = field.as_tensor(shape=(rows, columns))
            for row in range(rows):
                for column in range(columns):
                    tensor[row, column] = (sequence * 7 + row * 13 + column * 3) & 255
            pointer = tensor.data_ptr()
            native = field._owner._native_data_address(field._path, field._generation)
            assert pointer == native != 0
            checksum = tensor.sum().item()
            accounted = message._owner.snapshot['accounted_bytes']
            source.release()
            assert message._owner.snapshot['exports'] == 1
            rejected = words[0] == 'publish_alias'
            if rejected:
                alias = tensor[1:, :1].detach()
                del tensor
                try:
                    publisher.publish(message)
                except BufferError:
                    pass
                else:
                    raise AssertionError('publish accepted a live writable Tensor alias')
                assert message._owner.snapshot['exports'] == 1
                assert alias[0, 0].item() == (sequence * 7 + 13) & 255
                del alias
                gc.collect()
            else:
                del tensor
            assert message._owner.snapshot['exports'] == 0
            publisher.publish(message)
        emit('PUBLISHED', sequence=sequence, rows=rows, columns=columns,
             checksum=checksum, native_address=native, tensor_address=pointer,
             accounted_bytes=accounted, rejected_alias_publish=rejected)


def subscriber_commands(node, message_type, topic, qos):
    """Keep only a derived Tensor when testing retained-budget and close retry."""
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
    from rclpy.executors import SingleThreadedExecutor
    from rclpy_agnocast import SubscriptionLimits

    held, pending, errors = [], [], []
    received = 0
    hold = False
    group = MutuallyExclusiveCallbackGroup()
    stopping = threading.Event()

    def snapshot():
        stats = subscription.stats
        observation = subscription._handle._observation
        return dict(received=received, received_total=stats.received_total,
                    delivered_total=stats.delivered_total,
                    retained=stats.outstanding_leases,
                    accounted_bytes=stats.outstanding_accounted_bytes,
                    queued=stats.queued_leases, dropped_count=stats.dropped_count_full,
                    dropped_bytes=stats.dropped_bytes_full,
                    dropped_oversize=stats.dropped_oversize,
                    received_refs=observation.received, released_refs=observation.released)

    def callback(message):
        nonlocal received
        sequence = message.header.stamp.sec
        rows, columns = message.height, message.width
        assert rows == 2 + sequence % 4 and columns == 3 + sequence % 13
        assert message.step == columns and message.encoding.to_text() == 'mono8'
        expected = [[(7 * sequence + 13 * row + 3 * column) % 256
                     for column in range(columns)] for row in range(rows)]
        field = message.data
        source = field.buffer()
        assert source.readonly
        tensor = field.as_tensor(shape=(rows, columns))
        pointer = tensor.data_ptr()
        native = field._owner._native_data_address(field._path, field._generation)
        assert native == pointer != 0
        assert tensor.tolist() == expected
        assert tensor.element_size() == 1 and tensor.is_contiguous()
        accounted = message._owner.snapshot['accounted_bytes']
        source.release()
        # Releasing the buffer export cannot invalidate the independent Tensor owner.
        assert tensor.tolist() == expected
        if hold:
            held.append((tensor[1:, :1].detach(), [[row[0]] for row in expected[1:]]))
        received += 1
        pending.append(dict(sequence=sequence, rows=rows, columns=columns,
                            checksum=tensor.sum().item(), native_address=native,
                            tensor_address=pointer, root_accounted_bytes=accounted,
                            readonly=True))

    subscription = node.create_subscription(
        message_type, topic, callback, qos, transport='agnocast', callback_group=group,
        limits=SubscriptionLimits(max_messages=1, max_retained_bytes=65536,
                                  max_message_bytes=65536))

    def report_received():
        # This exclusive-group timer runs after callback and dispatch root release.
        while pending:
            emit('RX', **pending.pop(0), **snapshot())

    node.create_timer(0.001, report_received, callback_group=group)
    executor = SingleThreadedExecutor(context=node.context)
    executor.add_node(node)

    def spin():
        try:
            while not stopping.is_set():
                executor.spin_once(timeout_sec=0.1)
        except BaseException as error:
            errors.append(error)
            emit('ERROR', kind=type(error).__name__, detail=str(error))

    thread = threading.Thread(target=spin)
    thread.start()
    emit('READY', pid=os.getpid())
    try:
        for command in sys.stdin:
            words = command.split()
            if words == ['quit']:
                break
            if errors:
                raise errors[0]
            if words == ['connections']:
                count = subscription.publisher_count
                emit('CONNECTED', count=count, connected=count > 0)
            elif words == ['hold']:
                hold = True
                emit('MODE')
            elif words == ['stats'] or words[:1] == ['wait']:
                target = int(words[1]) if words[0] == 'wait' else 0
                deadline = time.monotonic() + 10
                while not receipt_settled(subscription._handle, target):
                    assert time.monotonic() < deadline, 'native receipts did not settle'
                    time.sleep(0.001)
                emit('STATS', **snapshot())
            elif words == ['write_fault']:
                assert held
                before = snapshot()
                for alias, expected in held:
                    assert alias.tolist() == expected
                signum = assert_write_fault(held[0][0], index=(0, 0))
                for alias, expected in held:
                    assert alias.tolist() == expected
                del alias
                after = snapshot()
                assert after == before
                emit('FAULT', signal=signum, preserved=True, **after)
            elif words == ['close_busy']:
                assert held
                try:
                    node.destroy_subscription(subscription)
                except RuntimeError:
                    pass
                else:
                    raise AssertionError('close accepted a retained Tensor alias')
                for alias, expected in held:
                    assert alias.tolist() == expected
                del alias
                emit('BUSY', preserved=True, **snapshot())
            elif words == ['release']:
                for alias, expected in held:
                    assert alias.tolist() == expected
                del alias
                held.clear()
                gc.collect()
                emit('RELEASED', **snapshot())
            elif words == ['close']:
                assert node.destroy_subscription(subscription)
                emit('CLOSED', **snapshot())
            else:
                raise ValueError(command)
    finally:
        held.clear()
        gc.collect()
        stopping.set()
        executor.shutdown(timeout_sec=5)
        thread.join(timeout=5)
        assert not thread.is_alive()
        executor.remove_node(node)
    if errors:
        raise errors[0]


def main():
    """Keep rclpy/runtime construction inside each independently preloaded process."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=('publisher', 'subscriber'), required=True)
    parser.add_argument('--topic', required=True)
    args = parser.parse_args()
    import rclpy
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rclpy_agnocast import Node
    from sensor_msgs.msg import Image

    rclpy.init(args=[])
    node = Node(f'tensor_checkpoint_{args.mode}_{os.getpid()}')
    qos = QoSProfile(depth=4, history=HistoryPolicy.KEEP_LAST,
                     reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)
    try:
        handler = publisher_commands if args.mode == 'publisher' else subscriber_commands
        handler(node, Image, args.topic, qos)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    emit('STOPPED')


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        emit('ERROR', kind=type(error).__name__, detail=str(error))
        raise
