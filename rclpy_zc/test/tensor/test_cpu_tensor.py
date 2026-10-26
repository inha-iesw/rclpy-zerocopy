"""CPU-T01/T02 producer tensors keep the native loan until the last storage alias."""

import builtins
import gc
from pathlib import Path
import subprocess
import sys
import textwrap
import threading

import pytest

from rclpy_agnocast import _native


_DTYPES = [
    ('bools', 'bool', 1), ('i8', 'int8', 1), ('u8', 'uint8', 1),
    ('i16', 'int16', 2), ('u16', 'uint16', 2), ('i32', 'int32', 4),
    ('u32', 'uint32', 4), ('i64', 'int64', 8), ('u64', 'uint64', 8),
    ('f32', 'float32', 4), ('f64', 'float64', 8),
]


def test_capsule_pin_keeps_only_metadata_until_wrapper_is_destroyed(publisher):
    endpoint, _ = publisher
    assert _native._test_tensor_live_managers() == 0
    with endpoint.borrow_message() as message:
        field = message.i32
        capsule = field._owner.dlpack(field._path, field._generation, (2,))
        assert _native._test_tensor_live_managers() == 1
        field._owner.discard_dlpack(capsule)
        assert message._owner.snapshot['exports'] == 0
        assert _native._test_tensor_live_managers() == 1
        del capsule
        gc.collect()
        assert _native._test_tensor_live_managers() == 0
        endpoint.publish(message)


def test_native_pointer_probe_does_not_freeze_structure(publisher):
    endpoint, native = publisher
    with endpoint.borrow_message() as message:
        field = message.i32
        address = field._owner._native_data_address(field._path, field._generation)
        assert address == native.inspect()['i32_address']
        assert message._owner.snapshot['phase'] == 'PREPARE'
        assert message._owner.snapshot['exports'] == 0
        message.values.resize(1)


def test_producer_tensor_uses_native_pointer_and_pins_loan(publisher, torch):
    endpoint, native = publisher
    with endpoint.borrow_message() as message:
        tensor = message.i32.as_tensor()
        assert _native._test_tensor_live_managers() == 1
        assert tensor.dtype == torch.int32
        assert tensor.device.type == 'cpu'
        assert tensor.data_ptr() == native.inspect()['i32_address']
        tensor[1] = -37
        assert native.inspect()['i32'] == [0, -37]
        del tensor
        gc.collect()
        assert _native._test_tensor_live_managers() == 0
        endpoint.publish(message)
    assert native.inspect()['publishes'] == 1


@pytest.mark.parametrize('field,dtype,width', _DTYPES)
def test_native_metadata_does_not_export_or_freeze(publisher, field, dtype, width):
    endpoint, _ = publisher
    with endpoint.borrow_message() as message:
        proxy = getattr(message, field)
        metadata = proxy._owner.tensor_info(proxy._path, proxy._generation)
        assert metadata == {'dtype': dtype, 'size': 2, 'item_size': width}
        assert message._owner.snapshot['exports'] == 0
        assert message._owner.snapshot['phase'] == 'PREPARE'


@pytest.mark.parametrize('field,dtype,width', _DTYPES)
def test_numeric_dtype_matches_native_storage(publisher, torch, field, dtype, width):
    endpoint, native = publisher
    with endpoint.borrow_message() as message:
        proxy = getattr(message, field)
        if not hasattr(torch, dtype):
            with pytest.raises(NotImplementedError):
                proxy.as_tensor()
            assert message._owner.snapshot['exports'] == 0
            return
        tensor = proxy.as_tensor(shape=(1, 2))
        assert tensor.dtype == getattr(torch, dtype)
        assert tuple(tensor.shape) == (1, 2)
        assert tensor.stride() == (2, 1)
        assert tensor.element_size() == width
        assert tensor.data_ptr() == native.inspect()[field + '_address']
        del tensor
        endpoint.publish(message)


@pytest.mark.parametrize('shape,error', [
    ((True, 2), TypeError), ((1.0, 2), TypeError), (2, TypeError),
    ((-1, 2), ValueError), ((sys.maxsize + 1,), ValueError),
    ((sys.maxsize, 2), ValueError), ((3,), ValueError), ((), ValueError),
])
def test_invalid_shape_rejects_before_freezing_or_export(publisher, shape, error):
    endpoint, _ = publisher
    with endpoint.borrow_message() as message:
        with pytest.raises(error):
            message.i32.as_tensor(shape=shape)
        assert message._owner.snapshot['phase'] == 'PREPARE'
        assert message._owner.snapshot['exports'] == 0
        message.values.resize(1)


@pytest.mark.parametrize('size,shape', [(1, ()), (0, (0,)), (0, (2, 0, 3)), (6, (2, 3))])
def test_scalar_empty_and_multidimensional_shapes(publisher, torch, size, shape):
    endpoint, _ = publisher
    with endpoint.borrow_message() as message:
        message.values.resize(size)
        tensor = message.values.as_tensor(shape=shape)
        assert tuple(tensor.shape) == shape
        assert tensor.numel() == size
        assert tensor.is_contiguous()
        del tensor
        endpoint.publish(message)


@pytest.mark.parametrize('field', ['text', 'wide', 'packed', 'children'])
def test_nonnumeric_fields_reject_without_export(publisher, field):
    endpoint, _ = publisher
    with endpoint.borrow_message() as message:
        with pytest.raises((TypeError, NotImplementedError)):
            getattr(message, field).as_tensor()
        assert message._owner.snapshot['phase'] == 'PREPARE'
        assert message._owner.snapshot['exports'] == 0


def test_torch_is_optional_and_missing_dependency_does_not_export(publisher, monkeypatch):
    endpoint, _ = publisher
    original_import = builtins.__import__

    def without_torch(name, *args, **kwargs):
        if name == 'torch' or name.startswith('torch.'):
            raise ImportError('torch intentionally unavailable')
        return original_import(name, *args, **kwargs)

    with endpoint.borrow_message() as message:
        monkeypatch.setattr(builtins, '__import__', without_torch)
        with pytest.raises(ImportError):
            message.i32.as_tensor()
        assert message._owner.snapshot['phase'] == 'PREPARE'
        assert message._owner.snapshot['exports'] == 0
        message.i32[0] = 19


def test_slice_and_detach_hold_export_after_source_memoryview_release(publisher, torch):
    endpoint, _ = publisher
    with endpoint.borrow_message() as message:
        message.values.resize(6)
        source = message.values.buffer()
        tensor = message.values.as_tensor(shape=(2, 3))
        tensor.fill_(7)
        alias = tensor[1:].detach()
        del tensor
        source.release()
        assert alias.tolist() == [[7, 7, 7]]
        assert message._owner.snapshot['exports'] == 1
        for operation in (endpoint.publish, endpoint.cancel):
            with pytest.raises(BufferError):
                operation(message)
        del alias
        gc.collect()
        assert message._owner.snapshot['exports'] == 0
        endpoint.publish(message)


def test_last_foreign_thread_alias_release_completes_pending_cancel(publisher, torch):
    endpoint, native = publisher
    with endpoint.borrow_message() as message:
        tensor = message.i32.as_tensor()
        aliases = [tensor.detach()[1:]]
        del tensor
    assert message._owner.snapshot['phase'] == 'CANCEL_PENDING'
    assert native.inspect()['cancels'] == 0

    def release():
        aliases.pop()
        gc.collect()

    thread = threading.Thread(target=release)
    thread.start()
    thread.join(timeout=5)
    assert not thread.is_alive()
    native.wait_idle()
    assert native.inspect()['cancels'] == 1
    assert native.inspect()['worker_only']


def test_unused_capsule_and_duplicate_consumption_release_exactly_once(publisher, torch):
    endpoint, _ = publisher
    with endpoint.borrow_message() as message:
        field = message.i32
        source = field.buffer()
        unused = field._owner.dlpack(field._path, field._generation, (2,))
        assert message._owner.snapshot['exports'] == 2
        assert _native._test_tensor_live_managers() == 1
        del unused
        gc.collect()
        assert message._owner.snapshot['exports'] == 1
        assert _native._test_tensor_live_managers() == 0
        capsule = field._owner.dlpack(field._path, field._generation, (2,))
        tensor = torch.utils.dlpack.from_dlpack(capsule)
        with pytest.raises(RuntimeError):
            torch.utils.dlpack.from_dlpack(capsule)
        assert message._owner.snapshot['exports'] == 2
        assert _native._test_tensor_live_managers() == 1
        del tensor
        gc.collect()
        assert message._owner.snapshot['exports'] == 1
        assert _native._test_tensor_live_managers() == 1
        del capsule
        gc.collect()
        assert _native._test_tensor_live_managers() == 0
        source.release()
        assert message._owner.snapshot['exports'] == 0
        endpoint.publish(message)


def test_consumer_failure_rolls_back_export_and_preserves_loan(publisher, torch, monkeypatch):
    endpoint, native = publisher

    def reject(capsule):
        raise RuntimeError('injected DLPack consumer failure')

    with endpoint.borrow_message() as message:
        source = message.i32.buffer()
        monkeypatch.setattr(torch.utils.dlpack, 'from_dlpack', reject)
        with pytest.raises(RuntimeError, match='injected') as saved_exception:
            message.i32.as_tensor()
        gc.collect()
        assert saved_exception.value.__traceback__ is not None
        assert message._owner.snapshot['exports'] == 1
        assert _native._test_tensor_live_managers() == 1
        assert message._owner.snapshot['phase'] == 'FILLING'
        with pytest.raises(BufferError):
            message.values.resize(1)
        saved_exception = None
        gc.collect()
        assert _native._test_tensor_live_managers() == 0
        source.release()
        assert message._owner.snapshot['exports'] == 0
        message.i32[0] = 83
        assert native.inspect()['i32'][0] == 83
        endpoint.publish(message)


def test_consumer_that_consumes_then_raises_preserves_live_tensor(publisher, torch, monkeypatch):
    endpoint, native = publisher
    consume = torch.utils.dlpack.from_dlpack
    aliases = []

    def consume_then_raise(capsule):
        tensor = consume(capsule)
        aliases.append(tensor[1:].detach())
        del tensor
        raise RuntimeError('failure after consuming capsule')

    with endpoint.borrow_message() as message:
        monkeypatch.setattr(torch.utils.dlpack, 'from_dlpack', consume_then_raise)
        with pytest.raises(RuntimeError, match='after consuming') as saved_exception:
            message.i32.as_tensor()
        assert saved_exception.value.__traceback__ is not None
        assert message._owner.snapshot['exports'] == 1
        assert _native._test_tensor_live_managers() == 1
        aliases[0].fill_(41)
        assert native.inspect()['i32'] == [0, 41]
        with pytest.raises(BufferError):
            endpoint.publish(message)
        aliases.clear()
        gc.collect()
        assert message._owner.snapshot['exports'] == 0
        assert _native._test_tensor_live_managers() == 1
        saved_exception = None
        gc.collect()
        assert _native._test_tensor_live_managers() == 0
        endpoint.publish(message)
    assert native.inspect()['publishes'] == 1


def test_native_invalid_shape_rejects_before_export(publisher):
    endpoint, _ = publisher
    with endpoint.borrow_message() as message:
        field = message.i32
        for shape in ((-1,), (3,), (sys.maxsize, 2)):
            with pytest.raises(ValueError):
                field._owner.dlpack(field._path, field._generation, shape)
        assert message._owner.snapshot['phase'] == 'PREPARE'
        assert message._owner.snapshot['exports'] == 0


@pytest.mark.parametrize('shape', [
    (0, 2**62, 2**62), (2**62, 0, 2**62), (2**62, 2**62, 0),
    (0, 2**32, 2**31),
])
@pytest.mark.parametrize('entry', ['public', 'native'])
def test_zero_extent_stride_overflow_rejects_before_export(publisher, monkeypatch, shape, entry):
    endpoint, _ = publisher
    original_import = builtins.__import__

    def reject_torch(name, *args, **kwargs):
        if name == 'torch' or name.startswith('torch.'):
            raise AssertionError('invalid shape reached the Torch consumer')
        return original_import(name, *args, **kwargs)

    with endpoint.borrow_message() as message:
        field = message.values
        if entry == 'public':
            monkeypatch.setattr(builtins, '__import__', reject_torch)
        with pytest.raises(ValueError, match='overflow'):
            if entry == 'public':
                field.as_tensor(shape=shape)
            else:
                field._owner.dlpack(field._path, field._generation, shape)
        assert message._owner.snapshot['exports'] == 0
        assert message._owner.snapshot['phase'] == 'PREPARE'
        message.values.resize(1)


@pytest.mark.parametrize('shape', [(0, sys.maxsize, 1), (sys.maxsize, 0, 1)])
def test_empty_tensor_accepts_int64_stride_volume_boundary(publisher, torch, shape):
    endpoint, _ = publisher
    with endpoint.borrow_message() as message:
        tensor = message.values.as_tensor(shape=shape)
        assert tuple(tensor.shape) == shape
        assert tensor.numel() == 0
        del tensor
        assert message._owner.snapshot['exports'] == 0
        endpoint.publish(message)


def _run_isolated(body, *arguments):
    """Confine crash-prone consumers and always drain the owned native publisher."""
    setup = """
        import faulthandler
        import gc
        import resource
        import sys
        from rclpy_agnocast import _native
        from rclpy_agnocast.types.message import Message
        faulthandler.disable()
        resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
        handle = _native._TestTypedPublisher(65536)
        message = Message(handle.borrow_message())
    """
    cleanup = """
        finally:
            message._owner.cancel(handle.id, True)
            handle.wait_idle()
            handle.close()
    """
    script = (textwrap.dedent(setup) + '\ntry:\n'
              + textwrap.indent(textwrap.dedent(body), '    ')
              + textwrap.dedent(cleanup))
    result = subprocess.run(
        [sys.executable, '-c', script, *arguments], cwd=Path(__file__).resolve().parents[3],
        capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr


def test_reported_overflow_cannot_crash_torch_consumer_process(torch):
    """Keep a regressed native double free from terminating the parent pytest process."""
    _run_isolated('''
        try:
            message.values.as_tensor(shape=(0, 2**62, 2**62))
        except ValueError:
            pass
        else:
            raise AssertionError('overflow shape was accepted')
        assert message._owner.snapshot['phase'] == 'PREPARE'
        assert message._owner.snapshot['exports'] == 0
        assert _native._test_tensor_live_managers() == 0
    ''')


@pytest.mark.parametrize('cleanup', ['facade', 'capsule_gc'])
def test_deleter_before_capsule_rename_preserves_independent_export(torch, cleanup):
    """Exercise failed consumer cleanup through the standard legacy DLPack ABI in a child."""
    _run_isolated('''
        import ctypes as c
        import torch.utils.dlpack

        # Public legacy DLPack layout; no assumptions about our private manager layout.
        class Device(c.Structure):
            _fields_ = [('type', c.c_int32), ('id', c.c_int32)]
        class Dtype(c.Structure):
            _fields_ = [('code', c.c_uint8), ('bits', c.c_uint8), ('lanes', c.c_uint16)]
        class Tensor(c.Structure):
            _fields_ = [('data', c.c_void_p), ('device', Device), ('ndim', c.c_int32),
                        ('dtype', Dtype), ('shape', c.POINTER(c.c_int64)),
                        ('strides', c.POINTER(c.c_int64)), ('offset', c.c_uint64)]
        class Managed(c.Structure):
            pass
        Deleter = c.CFUNCTYPE(None, c.POINTER(Managed))
        Managed._fields_ = [('tensor', Tensor), ('context', c.c_void_p), ('deleter', Deleter)]
        get_pointer = c.pythonapi.PyCapsule_GetPointer
        get_pointer.argtypes = [c.py_object, c.c_char_p]
        get_pointer.restype = c.c_void_p

        def release_without_rename(capsule):
            pointer = c.cast(get_pointer(capsule, b'dltensor'), c.POINTER(Managed))
            pointer.contents.deleter(pointer)
            assert get_pointer(capsule, b'dltensor')
            assert _native._test_tensor_live_managers() == 1

        field = message.i32
        source = field.buffer()
        try:
            if sys.argv[1] == 'facade':
                def fail_after_release(capsule):
                    release_without_rename(capsule)
                    raise RuntimeError('consumer released without renaming')
                torch.utils.dlpack.from_dlpack = fail_after_release
                saved_exception = None
                try:
                    field.as_tensor()
                except RuntimeError as error:
                    saved_exception = error
                assert saved_exception is not None
                assert message._owner.snapshot['exports'] == 1
                assert _native._test_tensor_live_managers() == 1
                saved_exception = None
            else:
                capsule = field._owner.dlpack(field._path, field._generation, (2,))
                release_without_rename(capsule)
                assert message._owner.snapshot['exports'] == 1
                del capsule
            gc.collect()
            assert _native._test_tensor_live_managers() == 0
            assert message._owner.snapshot['exports'] == 1
            source[1] = 73
            assert handle.inspect()['i32'] == [0, 73]
            try:
                message._owner.publish(handle.id)
            except BufferError:
                pass
            else:
                raise AssertionError('independent writable export was released')
            source.release()
            assert message._owner.snapshot['exports'] == 0
            message._owner.publish(handle.id)
            assert handle.inspect()['publishes'] == 1
        finally:
            source.release()
    ''', cleanup)
