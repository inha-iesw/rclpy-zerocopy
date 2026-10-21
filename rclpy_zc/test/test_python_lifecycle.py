"""Device-independent tests for the Python runtime ownership contract."""

from pathlib import Path
import re

import pytest

from rclpy_agnocast import AgnocastRuntime
from rclpy_agnocast import runtime as runtime_module


class _BusyRuntime:
    def __init__(self) -> None:
        self.close_calls = 0

    def close(self) -> None:
        self.close_calls += 1
        raise RuntimeError("active Loan keeps runtime busy")


class _IdleRuntime:
    def __init__(self) -> None:
        self.close_calls = 0
        self.created = []

    def create_publisher(self, topic: str, depth: int, limit: int):
        handle = _IdlePublisherHandle()
        self.created.append((topic, depth, limit, handle))
        return handle

    def close(self) -> None:
        self.close_calls += 1


class _IdlePublisherHandle:
    max_payload_bytes = 64

    def __init__(self) -> None:
        self.close_calls = 0

    def close(self) -> None:
        self.close_calls += 1


class _Context:
    def __init__(self, active: bool) -> None:
        self._active = active

    def ok(self) -> bool:
        return self._active


class _Node:
    def __init__(self, active: bool) -> None:
        self.context = _Context(active)

    def get_name(self) -> str:
        return "python_node"


def _runtime_with(native_runtime):
    runtime = AgnocastRuntime.__new__(AgnocastRuntime)
    runtime._closed = False
    runtime._native_runtime = native_runtime
    runtime._rclpy_node = object()
    return runtime


def test_runtime_context_preserves_body_exception_when_close_is_busy():
    native_runtime = _BusyRuntime()
    runtime = _runtime_with(native_runtime)

    with pytest.raises(LookupError, match="original failure"):
        with runtime:
            raise LookupError("original failure")

    assert native_runtime.close_calls == 1
    assert not runtime._closed


def test_publisher_retains_python_runtime_until_successful_close():
    native_runtime = _IdleRuntime()
    runtime = _runtime_with(native_runtime)
    publisher = runtime.create_publisher("/topic", depth=1, max_payload_bytes=64)

    assert publisher._runtime_owner is runtime

    publisher.close()
    assert publisher._runtime_owner is None
    assert native_runtime.created[0][3].close_calls == 1


def test_successful_close_releases_process_global_python_keepalive(monkeypatch):
    native_runtime = _IdleRuntime()
    runtime = _runtime_with(native_runtime)
    monkeypatch.setattr(runtime_module, "_active_runtime", runtime, raising=False)

    runtime.close()

    assert runtime_module._active_runtime is None


def test_rclpy_node_context_must_be_active():
    with pytest.raises(RuntimeError, match="context is not active"):
        runtime_module._validate_rclpy_node(_Node(active=False))

    assert runtime_module._validate_rclpy_node(_Node(active=True)) == "python_node"


def test_loaded_artifact_check_rejects_non_interposed_heaphook(
    monkeypatch, tmp_path: Path
):
    hook = tmp_path / "libagnocast_heaphook.so"
    library = tmp_path / "libagnocast.so"
    hook.touch()
    library.touch()
    monkeypatch.setenv("LD_PRELOAD", str(hook))
    monkeypatch.setattr(runtime_module, "_malloc_provider_path", lambda: library)
    monkeypatch.setattr(runtime_module, "_loaded_object_paths", lambda: {hook, library})

    with pytest.raises(RuntimeError, match="does not provide the active malloc"):
        runtime_module._verify_loaded_agnocast()


def test_loaded_artifact_check_accepts_one_coherent_loaded_pair(
    monkeypatch, tmp_path: Path
):
    hook = tmp_path / "libagnocast_heaphook.so"
    library = tmp_path / "libagnocast.so"
    hook.touch()
    library.touch()
    monkeypatch.setenv("LD_PRELOAD", str(hook))
    monkeypatch.setattr(runtime_module, "_malloc_provider_path", lambda: hook)
    monkeypatch.setattr(runtime_module, "_loaded_object_paths", lambda: {hook, library})

    runtime_module._verify_loaded_agnocast()


def _isolate_device_preflight(monkeypatch, device: Path) -> None:
    monkeypatch.setenv("AGNOCAST_DEVICE_PATH", str(device))
    monkeypatch.setenv("AGNOCAST_BRIDGE_MODE", "off")
    monkeypatch.setenv("AGNOCAST_NO_DISCOVERY_AGENT", "1")
    monkeypatch.setattr(runtime_module, "_verify_loaded_agnocast", lambda: None)


def test_device_path_defaults_when_unset_or_empty(monkeypatch):
    monkeypatch.delenv("AGNOCAST_DEVICE_PATH", raising=False)
    assert runtime_module._device_path() == Path("/dev/agnocast")

    monkeypatch.setenv("AGNOCAST_DEVICE_PATH", "")
    assert runtime_module._device_path() == Path("/dev/agnocast")


def test_preflight_accepts_device_selected_by_agnocast_device_path(
    monkeypatch, tmp_path: Path
):
    device = tmp_path / "agnocast_rclpy"
    device.touch()
    _isolate_device_preflight(monkeypatch, device)

    runtime_module._preflight()


def test_preflight_names_missing_selected_device(monkeypatch, tmp_path: Path):
    device = tmp_path / "agnocast_rclpy"
    _isolate_device_preflight(monkeypatch, device)

    with pytest.raises(RuntimeError, match=re.escape(f"{device} is missing")):
        runtime_module._preflight()
