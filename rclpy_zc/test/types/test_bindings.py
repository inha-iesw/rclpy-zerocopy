"""Exercise real typed bindings on ordinary native storage without a kernel device."""

import ctypes
import gc
import threading
from types import SimpleNamespace

import pytest

from rclpy_agnocast import _native
from rclpy_agnocast.runtime import AgnocastRuntime
from rclpy_agnocast.types.message import Message
from rclpy_agnocast.types.publisher import TypedPublisher


@pytest.fixture
def endpoints():
    """Retain native endpoints through cleanup; do not hide cleanup failures."""
    handles = []

    def create():
        handle = _native._TestTypedPublisher(65536)
        handles.append(handle)
        runtime = SimpleNamespace(_lifecycle_lock=threading.RLock())
        return TypedPublisher(handle, runtime), handle

    yield create
    gc.collect()
    for handle in handles:
        handle.close()
        trace = handle.inspect()
        assert trace['live'] is False
        assert trace['creates'] == trace['destroys'] == 1
        assert trace['worker_only']


@pytest.mark.parametrize('field,value,error', [
    ('flag', 1, TypeError), ('signed', True, TypeError),
    ('signed', 1.5, TypeError), ('signed', -(2**31) - 1, ValueError),
    ('signed', 2**63, ValueError), ('unsigned', -1, ValueError),
    ('unsigned', 2**64, ValueError), ('real', True, TypeError),
    ('real', '1', TypeError), ('real', 3.5e38, ValueError),
    ('real', 10**400, ValueError),
])
def test_scalar_conversion_rejects_before_writing(endpoints, field, value, error):
    publisher, handle = endpoints()
    with publisher.borrow_message() as message:
        with pytest.raises(error):
            setattr(message, field, value)
        assert message._owner.snapshot['phase'] == 'PREPARE'
        native = handle.inspect()
        assert (native['flag'], native['signed'], native['unsigned'], native['real']) == (
            False, 0, 0, 0.0)
        message.flag = True
        message.signed = -(2**31)
        message.unsigned = 2**64 - 1
        message.real = 1.25
        native = handle.inspect()
        assert (native['flag'], native['signed'], native['unsigned'], native['real']) == (
            True, -(2**31), 2**64 - 1, 1.25)
        assert (message.flag, message.signed, message.unsigned, message.real) == (
            True, -(2**31), 2**64 - 1, 1.25)
        publisher.publish(message)
    assert handle.inspect()['publishes'] == 1


@pytest.mark.parametrize('field,format_,width,value', [
    ('bools', '?', 1, True), ('i8', 'b', 1, -7), ('u8', 'B', 1, 251),
    ('i16', 'h', 2, -1234), ('u16', 'H', 2, 65432),
    ('i32', 'i', 4, -(2**30)), ('u32', 'I', 4, 2**31 + 3),
    ('i64', 'q', 8, -(2**62)), ('u64', 'Q', 8, 2**63 + 3),
    ('f32', 'f', 4, 1.25), ('f64', 'd', 8, -2.5),
])
def test_buffers_have_native_metadata_address_and_values(endpoints, field, format_, width, value):
    publisher, handle = endpoints()
    with publisher.borrow_message() as message:
        with getattr(message, field).buffer() as view:
            assert (view.format, view.itemsize, view.ndim) == (format_, width, 1)
            assert (view.shape, view.strides, view.nbytes) == ((2,), (width,), 2 * width)
            assert view.contiguous and view.c_contiguous and not view.readonly
            assert ctypes.addressof(ctypes.c_char.from_buffer(view)) == handle.inspect()[
                field + '_address']
            view[1] = value
            assert handle.inspect()[field] == [0, value]
        publisher.publish(message)
    assert handle.inspect()['publishes'] == 1


def test_empty_sequence_and_fixed_and_packed_boolean_limits(endpoints):
    publisher, _ = endpoints()
    with publisher.borrow_message() as message:
        with pytest.raises(ValueError, match='fixed structure'):
            message.i32.resize(3)
        message.packed.resize(3)
        with pytest.raises(NotImplementedError, match='packed boolean'):
            message.packed.buffer()
        assert message._owner.snapshot['phase'] == 'PREPARE'
        with message.values.buffer() as view:
            assert (view.format, view.shape, view.nbytes, view.readonly) == ('i', (0,), 0, False)
        with pytest.raises(BufferError, match='PREPARE'):
            message.values.resize(1)
        message.packed[1] = True
        assert message.packed[1] is True
        with pytest.raises(TypeError, match='bool'):
            message.packed[1] = 1


def test_resize_invalidates_nested_and_sibling_proxies_and_preserves_data(endpoints):
    publisher, handle = endpoints()
    with publisher.borrow_message() as message:
        message.children.resize(1)
        child = message.children[0]
        sibling = message.values
        child.values.resize(2)
        for stale in (child, sibling):
            with pytest.raises(RuntimeError, match='stale structural generation'):
                stale._info()
        message.children[0].values[1] = 42
        assert handle.inspect()['child_values'] == [[0, 42]]
        with pytest.raises(BufferError, match='PREPARE'):
            message.values.resize(2)
        assert handle.inspect()['child_values'] == [[0, 42]]
        with pytest.raises(IndexError):
            _ = message.children[1]
        with pytest.raises(AttributeError):
            _ = message.missing


def test_strings_decode_explicitly_without_freezing_and_export_code_units(endpoints):
    publisher, handle = endpoints()
    with publisher.borrow_message() as message:
        assert message.text.to_text() == message.wide.to_text() == ''
        assert message._owner.snapshot['phase'] == 'PREPARE'
        message.text.resize(2)
        message.wide.resize(1)
        with message.text.buffer() as view:
            assert view.format == 'B'
            view[:] = 'é'.encode()
        with message.wide.buffer() as view:
            assert view.format == 'H' and view.itemsize == 2
            view[0] = ord('한')
        assert message.text.to_text() == 'é'
        assert message.wide.to_text() == '한'
        assert handle.inspect()['text_bytes'] == 'é'.encode()
        assert handle.inspect()['wide_units'] == [ord('한')]


def test_derived_view_pins_root_until_release_on_foreign_thread(endpoints):
    publisher, handle = endpoints()
    with publisher.borrow_message() as message:
        message.values.resize(3)
        view = message.values.buffer()
        alias = view[1:]
        view.release()
        assert message._owner.snapshot['exports'] == 1
        with pytest.raises(BufferError):
            publisher.publish(message)
        with pytest.raises(BufferError):
            publisher.cancel(message)
    assert message._owner.snapshot['phase'] == 'CANCEL_PENDING'
    assert handle.inspect()['cancels'] == 0
    assert handle.inspect()['live']
    thread = threading.Thread(target=alias.release)
    thread.start()
    thread.join(timeout=5)
    assert not thread.is_alive()
    handle.wait_idle()
    assert handle.inspect()['cancels'] == 1
    assert handle.inspect()['worker_only']


def test_creator_thread_and_foreign_publisher_checks_do_not_mutate(endpoints):
    publisher, handle = endpoints()
    other, other_handle = endpoints()
    with publisher.borrow_message() as message:
        errors = []

        def read_foreign():
            try:
                _ = message.signed
            except RuntimeError as error:
                errors.append(str(error))

        thread = threading.Thread(target=read_foreign)
        thread.start()
        thread.join(timeout=5)
        assert not thread.is_alive()
        assert len(errors) == 1 and 'creator thread' in errors[0]
        with pytest.raises(TypeError, match='another publisher'):
            other.publish(message)
        with pytest.raises(TypeError, match='borrowed native root'):
            publisher.publish(object())
        assert handle.inspect()['publishes'] == other_handle.inspect()['publishes'] == 0


def test_publish_failure_retires_root_and_worker_accepts_next_borrow(endpoints):
    publisher, handle = endpoints()
    with publisher.borrow_message() as message:
        handle.fail_publish = True
        with pytest.raises(RuntimeError, match='injected typed publish failure'):
            publisher.publish(message)
        assert message._owner.snapshot['phase'] == 'FAILED'
        with pytest.raises(RuntimeError):
            _ = message.signed
    assert handle.inspect()['publishes'] == 1
    assert handle.inspect()['cancels'] == 0
    handle.fail_publish = False
    with publisher.borrow_message() as recovered:
        recovered.signed = 4
        publisher.publish(recovered)
    assert handle.inspect()['publishes'] == 2


def test_original_body_exception_survives_cleanup_exception():
    def fail_cancel(*args):
        raise RuntimeError('cleanup failed')

    owner = SimpleNamespace(
        info=lambda *args: {}, cancel=fail_cancel, publisher_id=1)
    with pytest.raises(LookupError, match='body failed'):
        with Message(owner):
            raise LookupError('body failed')
    with pytest.raises(RuntimeError, match='cleanup failed'):
        with Message(owner):
            pass


def test_runtime_routes_resolved_type_and_checks_closed_before_native(endpoints, monkeypatch):
    """Test Python routing only; RuntimeCore session creation remains a device test."""
    _, handle = endpoints()
    runtime = AgnocastRuntime.__new__(AgnocastRuntime)
    runtime._lifecycle_lock = threading.RLock()
    runtime._closed = False
    requested = []
    resolved = []
    message_class = object()

    def resolve(cls):
        resolved.append(cls)
        return {'ros_type': 'binding_test/msg/Message'}

    def create(*args):
        assert runtime._lifecycle_lock._is_owned()
        requested.append(args)
        return handle

    monkeypatch.setattr('rclpy_agnocast.types.resolve_adapter', resolve)
    runtime._native_runtime = SimpleNamespace(create_typed_publisher=create)
    try:
        typed = runtime.create_typed_publisher(
            message_class, '/typed', depth=3, max_storage_bytes=65536)
        assert resolved == [message_class]
        assert requested == [('binding_test/msg/Message', '/typed', 3, 65536)]
        assert isinstance(typed, TypedPublisher) and typed.max_storage_bytes == 65536
        with typed.borrow_message() as message:
            message.signed = 17
            typed.publish(message)
        assert handle.inspect()['publishes'] == 1
        runtime._closed = True
        with pytest.raises(RuntimeError, match='runtime is closed'):
            runtime.create_typed_publisher(
                message_class, '/typed', depth=3, max_storage_bytes=65536)
        assert len(requested) == 1
    finally:
        runtime._closed = True
