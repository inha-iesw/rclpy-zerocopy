"""Process-global Agnocast runtime integration."""

from __future__ import annotations

import ctypes
import inspect
import os
from pathlib import Path
from threading import RLock
from typing import Any, Optional, Set

from . import _native
from .publisher import Publisher
from .subscription import Subscription
from .subscription.endpoint import _close_all


class _DlInfo(ctypes.Structure):
    _fields_ = [
        ("filename", ctypes.c_char_p),
        ("base", ctypes.c_void_p),
        ("symbol_name", ctypes.c_char_p),
        ("symbol_address", ctypes.c_void_p),
    ]


_active_runtime: Optional["AgnocastRuntime"] = None


def _preload_hook_path() -> Path:
    entries = os.environ.get("LD_PRELOAD", "").replace(":", " ").split()
    matches = [
        Path(entry)
        for entry in entries
        if Path(entry).name == "libagnocast_heaphook.so"
    ]
    if len(matches) != 1 or not matches[0].is_absolute():
        raise RuntimeError("LD_PRELOAD must contain one absolute Agnocast heaphook path")
    try:
        hook = matches[0].resolve(strict=True)
    except OSError as error:
        raise RuntimeError("configured Agnocast heaphook does not exist") from error
    if not hook.is_file():
        raise RuntimeError("configured Agnocast heaphook is not a file")
    return hook


def _loaded_object_paths() -> Set[Path]:
    try:
        mappings = Path("/proc/self/maps").read_text(encoding="utf-8")
    except OSError as error:
        raise RuntimeError("cannot inspect loaded Agnocast artifacts") from error
    loaded = set()
    for line in mappings.splitlines():
        fields = line.split(maxsplit=5)
        if len(fields) != 6 or not fields[5].startswith("/"):
            continue
        loaded.add(Path(fields[5].removesuffix(" (deleted)")).resolve())
    return loaded


def _malloc_provider_path() -> Path:
    process = ctypes.CDLL(None)
    libdl = ctypes.CDLL("libdl.so.2")
    dladdr = libdl.dladdr
    dladdr.argtypes = [ctypes.c_void_p, ctypes.POINTER(_DlInfo)]
    dladdr.restype = ctypes.c_int
    info = _DlInfo()
    address = ctypes.cast(process.malloc, ctypes.c_void_p)
    if dladdr(address, ctypes.byref(info)) == 0 or not info.filename:
        raise RuntimeError("cannot identify the active malloc provider")
    return Path(os.fsdecode(info.filename)).resolve()


def _verify_loaded_agnocast() -> None:
    hook = _preload_hook_path()
    try:
        library = hook.with_name("libagnocast.so").resolve(strict=True)
    except OSError as error:
        raise RuntimeError("heaphook has no sibling fork libagnocast.so") from error
    loaded = _loaded_object_paths()
    if hook not in loaded:
        raise RuntimeError("configured Agnocast heaphook is not loaded")
    if library not in loaded:
        raise RuntimeError("_native did not load the heaphook's sibling libagnocast.so")
    if _malloc_provider_path() != hook:
        raise RuntimeError("configured Agnocast heaphook does not provide the active malloc")


def _validate_rclpy_node(rclpy_node: Any) -> str:
    if rclpy_node is None or not callable(getattr(rclpy_node, "get_name", None)):
        raise TypeError("rclpy_node must provide get_name()")
    context = getattr(rclpy_node, "context", None)
    if context is None or not callable(getattr(context, "ok", None)):
        raise TypeError("rclpy_node must provide an rclpy Context")
    if not context.ok():
        raise RuntimeError("rclpy_node context is not active")
    node_name = rclpy_node.get_name()
    if not isinstance(node_name, str) or not node_name:
        raise ValueError("rclpy_node name must be a non-empty string")
    return node_name


def _device_path() -> Path:
    # Mirror libagnocast: an unset or empty AGNOCAST_DEVICE_PATH means the default.
    return Path(os.environ.get("AGNOCAST_DEVICE_PATH") or "/dev/agnocast")


def _preflight() -> None:
    _verify_loaded_agnocast()
    if os.environ.get("AGNOCAST_BRIDGE_MODE", "").lower() not in {"0", "off"}:
        raise RuntimeError("AGNOCAST_BRIDGE_MODE must be 'off' for the initial adapter")
    if os.environ.get("AGNOCAST_NO_DISCOVERY_AGENT", "").lower() not in {
        "1",
        "true",
        "yes",
    }:
        raise RuntimeError("AGNOCAST_NO_DISCOVERY_AGENT must be enabled")
    device = _device_path()
    if not device.exists() or not os.access(device, os.R_OK | os.W_OK):
        raise RuntimeError(f"{device} is missing or not readable/writable")


def _positive_integer(name: str, value: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise TypeError(f"{name} must be an integer")
    if value <= 0:
        raise ValueError(f"{name} must be positive")
    return value


class AgnocastRuntime:
    """Own the native Agnocast session while retaining the supplied rclpy Node."""

    __slots__ = (
        "_closed", "_native_runtime", "_rclpy_node", "_lifecycle_lock", "_subscriptions"
    )

    def __init__(self, rclpy_node: Any) -> None:
        global _active_runtime
        if _active_runtime is not None:
            raise RuntimeError("an AgnocastRuntime is already active")
        rclpy_name = _validate_rclpy_node(rclpy_node)
        _preflight()
        node_name = f"{rclpy_name}_agnocast_adapter"
        self._rclpy_node = rclpy_node
        self._native_runtime = _native.RuntimeCore.create(node_name)
        self._closed = False
        self._lifecycle_lock = RLock()
        self._subscriptions = set()
        _active_runtime = self

    @property
    def state(self) -> _native.RuntimeState:
        return self._native_runtime.state

    def create_publisher(
        self,
        topic: str,
        *,
        depth: int,
        max_payload_bytes: int,
    ) -> Publisher:
        """Create a typed SharedBuffer endpoint with a mandatory payload limit."""
        if self._closed:
            raise RuntimeError("runtime is closed")
        if not isinstance(topic, str):
            raise TypeError("topic must be a string")
        if not topic:
            raise ValueError("topic must not be empty")
        depth = _positive_integer("depth", depth)
        limit = _positive_integer("max_payload_bytes", max_payload_bytes)
        with self._lifecycle_lock:
            handle = self._native_runtime.create_publisher(topic, depth, limit)
            return Publisher(handle, runtime_owner=self)

    def close(self) -> None:
        """Quiesce and close; active Loans or subscriber views make this retryably fail."""
        global _active_runtime
        with self._lifecycle_lock:
            if self._closed:
                return
            if self._subscriptions:
                _close_all(tuple(self._subscriptions), self._native_runtime.prepare_close,
                           self._native_runtime.close)
            else:
                self._native_runtime.close()
            self._closed = True
            self._rclpy_node = None
            if _active_runtime is self:
                _active_runtime = None

    def create_typed_publisher(
        self, message_type, topic: str, *, depth: int, max_storage_bytes: int
    ):
        """Create an installed generated endpoint with capacity-based root accounting.

        Discovery and type validation finish before native endpoint creation. This
        explicit API uses the existing process runtime; Node transport routing is separate.
        """
        from .types import resolve_adapter
        from .types.publisher import TypedPublisher

        if not isinstance(topic, str):
            raise TypeError("topic must be a string")
        if not topic:
            raise ValueError("topic must not be empty")
        depth = _positive_integer("depth", depth)
        limit = _positive_integer("max_storage_bytes", max_storage_bytes)
        descriptor = resolve_adapter(message_type)
        with self._lifecycle_lock:
            if self._closed:
                raise RuntimeError("runtime is closed")
            handle = self._native_runtime.create_typed_publisher(
                descriptor['ros_type'], topic, depth, limit)
            return TypedPublisher(handle, self)

    def create_subscription(
        self,
        topic: str,
        callback,
        *,
        depth: int,
        max_outstanding_leases: Optional[int] = None,
        max_payload_bytes: int,
    ) -> Subscription:
        """
        Deliver SharedBuffer Leases through the Node's executor, with bounded retention.

        The callback must be synchronous. ``max_outstanding_leases`` defaults to depth;
        ``max_payload_bytes`` bounds aggregate queued, delivered and view-held bytes.
        Close the returned subscription or this runtime before destroying the Node.
        """
        if not isinstance(topic, str):
            raise TypeError("topic must be a string")
        if not topic:
            raise ValueError("topic must not be empty")
        if not callable(callback):
            raise TypeError("callback must be callable")
        if (inspect.iscoroutinefunction(callback)
                or inspect.isasyncgenfunction(callback)
                or inspect.iscoroutinefunction(getattr(callback, "__call__", None))
                or inspect.isasyncgenfunction(getattr(callback, "__call__", None))):
            raise TypeError("subscription callback must be synchronous")
        depth = _positive_integer("depth", depth)
        count = depth if max_outstanding_leases is None else _positive_integer(
            "max_outstanding_leases", max_outstanding_leases
        )
        limit = _positive_integer("max_payload_bytes", max_payload_bytes)
        with self._lifecycle_lock:
            if self._closed:
                raise RuntimeError("runtime is closed")
            _validate_rclpy_node(self._rclpy_node)
            for name in ("create_guard_condition", "destroy_guard_condition"):
                if not callable(getattr(self._rclpy_node, name, None)):
                    raise TypeError(f"rclpy_node must provide {name}()")
            handle = self._native_runtime.create_subscription(topic, depth, count, limit)
            subscription = Subscription(handle, callback, runtime_owner=self)
            self._subscriptions.add(subscription)
            return subscription

    def __enter__(self) -> "AgnocastRuntime":
        return self

    def __exit__(self, exception_type: Any, exception: Any, traceback: Any) -> bool:
        del exception, traceback
        if exception_type is None:
            self.close()
        else:
            try:
                self.close()
            except BaseException:
                # Cleanup must never replace the exception raised by the context body.
                pass
        return False

    def __del__(self) -> None:
        if getattr(self, "_closed", True):
            return
        try:
            self.close()
        except BaseException:
            pass
