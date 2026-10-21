"""Inspect actual subscriber buffers without a transport-side payload copy."""

import ctypes

import numpy as np


def receipt_settled(subscription, expected):
    """Require the native receipt, reference resets, and live budget to agree."""
    stats = subscription.stats
    observation = subscription._observation
    return (
        stats.received_total >= expected
        and observation.received == stats.received_total
        and observation.received - observation.released == stats.outstanding_leases
    )


class _PyBuffer(ctypes.Structure):
    _fields_ = [
        ("buf", ctypes.c_void_p), ("obj", ctypes.c_void_p),
        ("length", ctypes.c_ssize_t), ("itemsize", ctypes.c_ssize_t),
        ("readonly", ctypes.c_int), ("ndim", ctypes.c_int),
        ("format", ctypes.c_char_p), ("shape", ctypes.POINTER(ctypes.c_ssize_t)),
        ("strides", ctypes.POINTER(ctypes.c_ssize_t)),
        ("suboffsets", ctypes.POINTER(ctypes.c_ssize_t)),
        ("internal", ctypes.c_void_p),
    ]


def buffer_address(view):
    """Read the CPython buffer pointer, including read-only exporters."""
    get_buffer = ctypes.pythonapi.PyObject_GetBuffer
    get_buffer.argtypes = [ctypes.py_object, ctypes.POINTER(_PyBuffer), ctypes.c_int]
    get_buffer.restype = ctypes.c_int
    release = ctypes.pythonapi.PyBuffer_Release
    release.argtypes = [ctypes.POINTER(_PyBuffer)]
    release.restype = None
    buffer = _PyBuffer()
    get_buffer(view, ctypes.byref(buffer), 0)
    try:
        return buffer.buf or 0
    finally:
        release(ctypes.byref(buffer))


def inspect(lease, observation):
    """Validate every byte, RO enforcement, and three addresses on one real Lease."""
    sequence = lease.sequence
    view = lease.buffer
    array = np.frombuffer(view, dtype=np.uint8)
    try:
        assert view.readonly and not array.flags.writeable
        assert view.format == "B" and view.itemsize == 1 and view.ndim == 1
        assert view.strides == (1,) and view.c_contiguous
        assert all(int(value) == ((sequence + index) & 255)
                   for index, value in enumerate(array))
        address = buffer_address(view)
        numpy_address = array.__array_interface__["data"][0]
        if len(view):
            assert address == numpy_address == lease._native_data_address
            try:
                view[0] = 0
            except TypeError:
                pass
            else:
                raise AssertionError("memoryview permitted a write")
            try:
                array[0] = 0
            except ValueError:
                pass
            else:
                raise AssertionError("NumPy permitted a write")
        try:
            array.setflags(write=True)
        except ValueError:
            pass
        else:
            raise AssertionError("NumPy permitted a writable alias")
        return dict(sequence=sequence, size=len(view), checksum=int(array.sum()),
                    native_address=lease._native_data_address,
                    callback_address=observation.native_data_address,
                    view_address=address, numpy_address=numpy_address, readonly=True)
    finally:
        del array
        view.release()


def check_held(value, sequence, offset=0):
    """Check retained view data after the original Lease wrapper was invalidated."""
    assert all(int(byte) == ((sequence + offset + index) & 255)
               for index, byte in enumerate(value))
