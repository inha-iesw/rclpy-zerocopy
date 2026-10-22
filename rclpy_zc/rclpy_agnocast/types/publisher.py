"""Explicit typed publisher API preceding the Node transport facade."""

from .message import Message


class TypedPublisher:
    """Own a generated native endpoint with a capacity-based per-root storage limit."""

    __slots__ = ('_handle', '_runtime_owner')

    def __init__(self, handle, runtime_owner):
        """Retain the endpoint and its existing process runtime."""
        self._handle = handle
        self._runtime_owner = runtime_owner

    def borrow_message(self):
        """Borrow a default native root; prepare sizes before writing or exporting."""
        return Message(self._handle.borrow_message())

    def _root(self, message):
        if not isinstance(message, Message) or message._path:
            raise TypeError('publish/cancel requires a borrowed native root')
        if message._owner.publisher_id != self._handle.id:
            raise TypeError('native root belongs to another publisher')
        return message._owner

    def publish(self, message):
        """Transfer a root with no live writable exports; all proxies then expire."""
        self._root(message).publish(self._handle.id)

    def cancel(self, message):
        """Cancel on the worker; reject while writable buffers are still alive."""
        self._root(message).cancel(self._handle.id)

    def close(self):
        """Stop new borrows and close, failing retryably while any loan is active."""
        with self._runtime_owner._lifecycle_lock:
            self._handle.close()

    @property
    def max_storage_bytes(self):
        """Return the limit including root size and capacity of nested allocations."""
        return self._handle.max_payload_bytes
