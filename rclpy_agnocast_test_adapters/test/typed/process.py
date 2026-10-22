"""Drive generated native factories and field proxies in a fresh real-backend process."""

import argparse
import ctypes
import gc
import os
import sys
import time

from system_runtime import emit


SCALARS = {
    'flag': True, 'byte_value': 255, 'character': 65,
    'signed8': -128, 'unsigned8': 255, 'signed16': -32768, 'unsigned16': 65535,
    'signed32': -2147483648, 'unsigned32': 4294967295,
    'signed64': -9223372036854775808, 'unsigned64': 18446744073709551615,
    'real32': 1.25, 'real64': -2.5,
}
STRINGS = {
    'text': 'typed π buffer payload', 'bounded_text': 'µ',
    'wide_text': '𝄞 typed wide payload', 'bounded_wide_text': '한글',
}
STRING_SEQUENCES = {
    'texts': ['abcdef' * 4, 'π'], 'bounded_texts': ['0123456789abcdef', 'x'],
    'wide_texts': ['큰' * 12, ''], 'bounded_wide_texts': ['한글', 'Z'],
}


def encoding(name):
    """Use explicit native UTF-16 code units only for wide string fields."""
    if 'wide' in name:
        return 'utf-16-le' if sys.byteorder == 'little' else 'utf-16-be'
    return 'utf-8'


def string_size(name, text):
    """Count native code units, including surrogate pairs but excluding terminators."""
    return len(text.encode(encoding(name))) // (2 if 'wide' in name else 1)


def prepare_containers(get):
    """Resolve each path again after structural generation changes."""
    for name, size in [('bounded', 3), ('values', 5), ('bounded_flags', 3), ('flags', 5)]:
        getattr(get(), name).resize(size)
    for name, text in STRINGS.items():
        getattr(get(), name).resize(string_size(name, text))
    for name, texts in STRING_SEQUENCES.items():
        getattr(get(), name).resize(len(texts))
        for index, text in enumerate(texts):
            getattr(get(), name)[index].resize(string_size(name, text))


def fill_string(field, name, text):
    """Write the pre-sized native storage, then explicitly decode a copy for checking."""
    with field.buffer() as view:
        with view.cast('B') as raw:
            raw[:] = text.encode(encoding(name))
    assert field.to_text() == text


def fill_scalars(message):
    """Write every scalar width using values independently checked by C++."""
    for name, value in SCALARS.items():
        setattr(message, name, value)
        assert getattr(message, name) == value


def fill_containers(message):
    """Fill fixed, bounded, packed-boolean and string storage without structural changes."""
    for name, values in {
            'fixed': [1.25, -2.5, 0.0, 4.0], 'bounded': [0.5, -1.5, 2.5],
            'values': [0, 1, 127, 128, 255], 'fixed_flags': [True, False, True, False],
            'bounded_flags': [False, True, False], 'flags': [True, False, True, False, True],
    }.items():
        field = getattr(message, name)
        if name in ('bounded_flags', 'flags'):
            for index, value in enumerate(values):
                field[index] = value
        else:
            with field.buffer() as view:
                for index, value in enumerate(values):
                    view[index] = value
    for name, text in STRINGS.items():
        fill_string(getattr(message, name), name, text)
    for name, texts in STRING_SEQUENCES.items():
        for index, text in enumerate(texts):
            fill_string(getattr(message, name)[index], name, text)


def fill(message, kind):
    """Complete all preparation before the first scalar write or buffer export."""
    if kind == 'Containers':
        prepare_containers(lambda: message)
        fill_containers(message)
    elif kind == 'Nested':
        message.children.resize(2)
        message.bounded_children.resize(1)
        prepare_containers(lambda: message.containers)
        prepare_containers(lambda: message.bounded_children[0])
        fill_scalars(message.child)
        fill_containers(message.containers)
        for index in range(2):
            fill_scalars(message.fixed_children[index])
            fill_scalars(message.children[index])
        fill_containers(message.bounded_children[0])
    else:
        fill_scalars(message)


def rejects(exception, action):
    """Require the documented exception rather than accepting an arbitrary failure."""
    try:
        action()
    except exception:
        return
    raise AssertionError(f'expected {exception}')


def wait_released(active_loans):
    """Wait only for asynchronous last-alias cancellation on the worker."""
    deadline = time.monotonic() + 3
    while active_loans() and time.monotonic() < deadline:
        time.sleep(0.005)
    assert active_loans() == 0


def bounds(publisher, active_loans, invalid_topic):
    """Reject invalid sizes and stale references while preserving a usable current root."""
    from rclpy_agnocast import _native

    before = _native._test_publisher_snapshot()['backend_epoch']
    rejects(ValueError, invalid_topic)
    assert _native._test_publisher_snapshot()['backend_epoch'] == before
    with publisher.borrow_message() as message:
        assert active_loans() == 1
        old = message.fixed
        generation = message._owner.snapshot['generation']
        message.values.resize(5)
        assert message._owner.snapshot['generation'] == generation + 1
        rejects(RuntimeError, lambda: len(old))
        generation = message._owner.snapshot['generation']
        for field, size in [('bounded', 9), ('bounded_text', 17), ('values', 1 << 63)]:
            rejects(ValueError, lambda: getattr(message, field).resize(size))
        rejects((ValueError, TypeError, OverflowError), lambda: message.values.resize(-1))
        assert message._owner.snapshot['generation'] == generation
        rejects(IndexError, lambda: message.values[5])
        rejects(ValueError, lambda: message.values[-1])
        message.values[0] = 23
        rejects(ValueError, lambda: message.values.__setitem__(0, 256))
        rejects(ValueError, lambda: message.values.__setitem__(0, -1))
        assert message.values[0] == 23
        rejects(BufferError, lambda: message.values.resize(4))
        publisher.cancel(message)
    wait_released(active_loans)
    with publisher.borrow_message() as message:
        message.flags.resize(3)
        rejects(NotImplementedError, lambda: message.flags.buffer())
        message.flags[1] = True
        assert message.flags[1] is True
        publisher.cancel(message)
    wait_released(active_loans)


def lifetime(publisher, active_loans, recreate):
    """Keep aliases valid through rejected terminals and defer context cancellation."""
    with publisher.borrow_message() as message:
        message.values.resize(8)
        view = message.values.buffer()
        alias = view[1:]
        view.release()
        alias[0] = 41
        assert message._owner.snapshot['exports'] == 1
        rejects(BufferError, lambda: publisher.publish(message))
        rejects(BufferError, lambda: publisher.cancel(message))
        rejects(RuntimeError, publisher.close)
        rejects(BufferError, lambda: message.values.resize(8))
        assert alias[0] == 41
        sys.modules.pop('rclpy_agnocast_test_adapters._types', None)
        gc.collect()
        assert message.values[1] == 41
    assert active_loans() == 1
    alias[1] = 42
    rejects(RuntimeError, lambda: len(message.values))
    alias.release()
    wait_released(active_loans)
    publisher.close()
    publisher = recreate()
    with publisher.borrow_message() as fresh:
        fresh.values.resize(1)
        fresh.values[0] = 9
        publisher.cancel(fresh)
    wait_released(active_loans)
    return publisher


def allocation(publisher, active_loans):
    """Fail the real worker's next resize allocation and recover without publishing it."""
    from rclpy_agnocast import _native

    for nested, ordinal in [(False, 0), (True, 2)]:
        before = _native._test_publisher_snapshot()
        _native._test_fail_allocation('resize', ordinal)
        with publisher.borrow_message() as message:
            borrowed = _native._test_publisher_snapshot()
            # Tracking starts while failure is armed, including borrow-stage allocations.
            # Containers has empty sequences/SSO strings; root storage, ipc control block,
            # and the long topic copy precede the separately counted resize allocations.
            borrow_allocations = borrowed['failure_tracked_allocations']
            emit('ALLOCATION_BASELINE', nested=nested, tracked=borrow_allocations)
            if nested:
                message.texts.resize(2)
                message.texts[0].resize(64)
                rejects(MemoryError, lambda: message.texts[1].resize(4096))
            else:
                rejects(MemoryError, lambda: message.values.resize(4096))
            assert message._owner.snapshot['phase'] == 'CANCEL_ONLY'
            rejects(RuntimeError, lambda: publisher.publish(message))
            publisher.cancel(message)
        wait_released(active_loans)
        failed = _native._test_publisher_snapshot()
        assert failed['injected_failures'] == before['injected_failures'] + 1
        assert failed['publish_complete'] == before['publish_complete']
        assert failed['cancel_complete'] == before['cancel_complete'] + 1
        assert failed['borrow_tls'] == 1
        assert failed['terminal_tls'] == 0
        assert failed['resize_tid'] == failed['worker_tid'] != failed['caller_tid']
        assert failed['failure_live_allocations'] == 0
        assert failed['failure_tracking_overflow'] == 0
        assert failed['failure_tracked_allocations'] == borrow_allocations + ordinal, {
            'nested': nested, 'ordinal': ordinal, 'borrow': borrowed, 'after_cancel': failed}
        with publisher.borrow_message() as message:
            message.values.resize(4096)
            publisher.cancel(message)
        wait_released(active_loans)


def main():
    """Run commands against one fixture type and close all native owners before exit."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', required=True)
    parser.add_argument('--kind', choices=['Scalars', 'Containers', 'Nested'], required=True)
    args = parser.parse_args()
    import rclpy
    from rclpy_agnocast import AgnocastRuntime
    from rclpy_agnocast import _native
    from rclpy_agnocast_test_msgs import msg as messages

    rclpy.init(args=[])
    node = rclpy.create_node(f'typed_publisher_{os.getpid()}')
    runtime = AgnocastRuntime(node)
    _native._test_enable_observation()

    def active_loans():
        counters = _native._test_publisher_snapshot()
        return (counters['borrow_complete'] - counters['publish_complete'] -
                counters['cancel_complete'])

    def create(topic=args.topic):
        return runtime.create_typed_publisher(
            getattr(messages, args.kind), topic, depth=4, max_storage_bytes=65536)

    publisher = create()
    try:
        assert active_loans() == 0
        emit('READY', pid=os.getpid())
        for command in sys.stdin:
            command = command.strip()
            if command == 'quit':
                break
            if command == 'publish':
                with publisher.borrow_message() as message:
                    observation = _native._test_publisher_snapshot()
                    assert observation['borrow_tls'] == 1
                    assert observation['caller_tls'] == 0
                    assert observation['borrow_tid'] == observation['worker_tid']
                    assert observation['borrow_tid'] != observation['caller_tid']
                    root_address = observation['message_address']
                    assert _native._test_is_shared(root_address)
                    fill(message, args.kind)
                    accounted = message._owner.snapshot['accounted_bytes']
                    address = 0
                    if args.kind != 'Scalars':
                        values = (message.values if args.kind == 'Containers'
                                  else message.containers.values)
                        with values.buffer() as view:
                            address = ctypes.addressof(ctypes.c_uint8.from_buffer(view))
                            assert _native._test_is_shared(address)
                    publisher.publish(message)
                    field = {'Scalars': 'flag', 'Containers': 'values',
                             'Nested': 'child'}[args.kind]
                    rejects(RuntimeError, lambda: getattr(message, field))
                wait_released(active_loans)
                emit('PUBLISHED', accounted_bytes=accounted,
                     active_loans=active_loans(), data_address=address, root_address=root_address)
            elif command in ('bounds', 'lifetime', 'allocation'):
                published = _native._test_publisher_snapshot()['publish_complete']
                if command == 'lifetime':
                    publisher = lifetime(publisher, active_loans, create)
                elif command == 'bounds':
                    bounds(publisher, active_loans, lambda: create(args.topic + '\0suffix'))
                else:
                    allocation(publisher, active_loans)
                assert _native._test_publisher_snapshot()['publish_complete'] == published
                emit('CHECKED', case=command, active_loans=active_loans())
            else:
                raise ValueError(f'unknown command {command}')
    finally:
        publisher.close()
        runtime.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        emit('ERROR', kind=type(error).__name__, detail=str(error))
        raise
