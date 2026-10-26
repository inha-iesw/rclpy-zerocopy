"""Owner-backed native field access for generated ROS message adapters."""

import operator
import sys


def _index(value):
    if isinstance(value, bool):
        raise TypeError('index or size must be an integer')
    value = operator.index(value)
    if value < 0 or value > sys.maxsize:
        raise ValueError('index or size must fit a non-negative Py_ssize_t')
    return value


class _Proxy:
    __slots__ = ('_owner', '_path', '_generation', '_readonly')

    def __init__(self, owner, path=(), generation=0, *, readonly=False):
        object.__setattr__(self, '_owner', owner)
        object.__setattr__(self, '_path', path)
        object.__setattr__(self, '_generation', generation)
        object.__setattr__(self, '_readonly', readonly)

    def _info(self):
        return self._owner.info(self._path, self._generation)

    def _child(self, index):
        info = self._info()
        path = (*self._path, index)
        generation = info['generation']
        child = self._owner.info(path, generation)
        if child['kind'] == 'scalar':
            return self._owner.read(path, generation)
        cls = Message if child['kind'] == 'message' else Field
        return cls(self._owner, path, generation, readonly=self._readonly)

    def _require_writable(self):
        if self._readonly:
            raise TypeError('received message is read-only')


class Message(_Proxy):
    """
    Native root or nested message; field access never constructs an owning ROS message.

    Resize every dynamic field before the first write or buffer export, then reacquire
    child proxies from the root. A successful resize invalidates all older child proxies.
    Use a borrowed root in a synchronous context on its creator thread. Unpublished context exit
    cancels on the worker, deferring cleanup while a writable buffer remains alive.
    Received messages are read-only; their fields and buffers share the native owner
    independently of callback lifetime and cannot be used as producer loan contexts.
    """

    __slots__ = ()

    def __getattr__(self, name):
        """Resolve a named native member with lifetime and generation checks."""
        info = self._info()
        try:
            index = info['members'].index(name)
        except ValueError:
            raise AttributeError(name) from None
        return self._child(index)

    def __setattr__(self, name, value):
        """Assign a scalar directly and freeze the root structure on success."""
        self._require_writable()
        info = self._info()
        try:
            index = info['members'].index(name)
        except ValueError:
            raise AttributeError(name) from None
        self._owner.write((*self._path, index), info['generation'], value)

    def __enter__(self):
        """Validate a root loan before entering its synchronous context."""
        self._require_writable()
        if self._path:
            raise TypeError('only the borrowed root is a loan context')
        self._info()
        return self

    def __exit__(self, exception_type, exception, traceback):
        """Cancel unpublished storage while preserving the body exception."""
        self._require_writable()
        del exception, traceback
        try:
            self._owner.cancel(self._owner.publisher_id, True)
        except BaseException:
            if exception_type is None:
                raise
        return False


class Field(_Proxy):
    """Container proxy storing a checked path and generation, never an element pointer."""

    __slots__ = ()

    def __len__(self):
        """Read the current native length after validating this proxy."""
        return self._info()['size']

    def __getitem__(self, index):
        """Resolve one native element without retaining its address."""
        return self._child(_index(index))

    def __setitem__(self, index, value):
        """Write a checked scalar element directly into native storage."""
        self._require_writable()
        info = self._info()
        self._owner.write((*self._path, _index(index)), info['generation'], value)

    def resize(self, size):
        """Resize on the native worker in PREPARE; reacquire field proxies afterward."""
        self._require_writable()
        self._owner.resize(self._path, self._generation, _index(size))

    def buffer(self):
        """
        Export native-endian elements, read-only for a received message.

        A producer export freezes structure. Release views and consumers before publish.
        Consumers must release before their source view; use as_tensor() for independent
        Tensor ownership. Packed boolean sequences reject export.
        String buffers hold UTF-8 bytes or native-endian UTF-16 code units.
        """
        return self._owner.buffer(self._path, self._generation)

    def as_tensor(self, *, shape=None):
        """
        Share a dense native-endian numeric field with a CPU Torch Tensor, without copying.

        Shape defaults to (len(field),); its product must equal the native element count.
        Contiguous stride volume must fit int64 even for empty shapes, so some empty
        shapes accepted by Torch are rejected before native storage is exported.
        Torch is optional until this call. Producer Tensor storage, including slice/detach
        aliases, must be released before publish or cancel. Received tensors require an
        actual read-only mapping from another process and retain the whole root budget.
        Use read-only operations or clone() on received tensors: in-place writes can fault
        at the OS mapping boundary. Direct raw-pointer/frombuffer escape is unsupported.
        """
        from ..tensor import as_tensor
        return as_tensor(self._owner, self._path, self._generation, shape=shape)

    def to_text(self):
        """Explicitly copy and decode a string without changing the preparation phase."""
        info = self._info()
        if info['kind'] not in ('string', 'wstring'):
            raise TypeError('to_text requires a string field')
        encoding = 'utf-8' if info['kind'] == 'string' else (
            'utf-16-le' if sys.byteorder == 'little' else 'utf-16-be')
        return self._owner.text_bytes(self._path, self._generation).decode(encoding)
