"""Subprocess protocol and exact-artifact environments for publisher/subscriber TLD."""

from __future__ import annotations

from contextlib import contextmanager
from dataclasses import dataclass
import os
from pathlib import Path
import queue
import stat
import subprocess
import tempfile
import threading
import time
from urllib.parse import quote


EVENT_PREFIX = "EVENT "
SYSTEM_TLD_ENV = "RCLPY_AGNOCAST_RUN_SYSTEM_TLD"
_WORKSPACE = Path(__file__).resolve().parents[2]


class PreflightError(RuntimeError):
    """Report that an explicitly requested system TLD is unsafe to run."""


@dataclass(frozen=True)
class SystemConfig:
    """Hold one exact kmod, userspace library, and test-program artifact set."""

    heaphook: Path
    library: Path
    kmod: Path
    publisher_process: Path
    subscriber: Path
    device: Path
    device_major: int
    device_minor: int
    kmod_srcversion: str

    def child_environment(self, domain_id: int) -> dict[str, str]:
        """Build a fresh-process environment with the selected fork first."""
        environment = os.environ.copy()
        environment["AGNOCAST_BRIDGE_MODE"] = "off"
        environment["AGNOCAST_NO_DISCOVERY_AGENT"] = "1"
        environment["AGNOCAST_DEVICE_PATH"] = str(self.device)
        environment["ROS_DOMAIN_ID"] = str(domain_id)
        environment["PYTHONUNBUFFERED"] = "1"
        environment["RCUTILS_COLORIZED_OUTPUT"] = "0"
        # Child entry points live below test/ and import this shared protocol module.
        test_path = str(Path(__file__).resolve().parent)
        python_paths = [path for path in environment.get("PYTHONPATH", "").split(":")
                        if path and path != test_path]
        environment["PYTHONPATH"] = ":".join([test_path, *python_paths])

        preload = _path_entries(environment.get("LD_PRELOAD", ""))
        preload = [entry for entry in preload if Path(entry).name != self.heaphook.name]
        environment["LD_PRELOAD"] = ":".join([str(self.heaphook), *preload])

        library_paths = environment.get("LD_LIBRARY_PATH", "").split(":")
        library_paths = [entry for entry in library_paths if entry]
        selected = str(self.library.parent)
        library_paths = [entry for entry in library_paths if entry != selected]
        environment["LD_LIBRARY_PATH"] = ":".join([selected, *library_paths])
        return environment


def system_tld_requested() -> bool:
    """Return true only for the explicit system-TLD opt-in value."""
    return os.environ.get(SYSTEM_TLD_ENV) == "1"


def _path_entries(value: str) -> list[str]:
    return value.replace(":", " ").split()


def _required_file(variable: str, default: Path) -> Path:
    candidate = Path(os.environ.get(variable, str(default))).expanduser()
    try:
        resolved = candidate.resolve(strict=True)
    except OSError as error:
        raise PreflightError(f"{variable} artifact is missing: {candidate}") from error
    if not resolved.is_file():
        raise PreflightError(f"{variable} is not a file: {resolved}")
    return resolved


def _subscriber_path() -> Path:
    configured = os.environ.get("RCLPY_AGNOCAST_SUBSCRIBER")
    candidates = [Path(configured)] if configured else [
        _WORKSPACE / "build/rclpy_agnocast/rclpy_agnocast_shared_buffer_subscriber",
    ]
    for candidate in candidates:
        if candidate.is_file() and os.access(candidate, os.X_OK):
            return candidate.resolve()
    rendered = ", ".join(str(candidate) for candidate in candidates)
    raise PreflightError(f"subscriber helper is missing or not executable: {rendered}")


def _device_numbers(device: Path, module: str) -> tuple[int, int]:
    try:
        metadata = device.stat()
    except OSError as error:
        raise PreflightError(f"{device} is missing") from error
    if not stat.S_ISCHR(metadata.st_mode):
        raise PreflightError(f"{device} is not a character device")
    if not os.access(device, os.R_OK | os.W_OK):
        raise PreflightError(f"{device} is not readable and writable")
    numbers = (os.major(metadata.st_rdev), os.minor(metadata.st_rdev))
    if numbers[0] <= 0 or numbers[1] != 0:
        raise PreflightError(f"unexpected {device} major/minor: {numbers}")
    try:
        # The kmod names its device and class after the module (KBUILD_MODNAME).
        sysfs_root = Path(f"/sys/class/{module}_class/{module}")
        sysfs_text = (sysfs_root / "dev").read_text(
            encoding="ascii"
        ).strip()
        sysfs_numbers = tuple(int(value) for value in sysfs_text.split(":"))
        uevent = dict(
            line.split("=", 1)
            for line in (sysfs_root / "uevent").read_text(encoding="ascii").splitlines()
            if "=" in line
        )
    except (OSError, ValueError) as error:
        raise PreflightError("cannot read the Agnocast sysfs device identity") from error
    if sysfs_numbers != numbers:
        raise PreflightError(
            f"device/sysfs major/minor mismatch: {numbers} != {sysfs_numbers}"
        )
    if uevent.get("DEVNAME") != module or uevent.get("DEVMODE") != "0666":
        raise PreflightError(f"unexpected Agnocast uevent identity/mode: {uevent}")
    if stat.S_IMODE(metadata.st_mode) != int(uevent["DEVMODE"], 8):
        raise PreflightError(f"{device} mode does not match its uevent")
    return numbers


def _verify_shm_rw() -> None:
    root = Path("/dev/shm")
    if not root.is_dir() or not os.access(root, os.R_OK | os.W_OK | os.X_OK):
        raise PreflightError("/dev/shm is not a readable and writable directory")
    try:
        with tempfile.NamedTemporaryFile(prefix="rclpy_agnocast_", dir=root) as probe:
            probe.write(b"x")
            probe.flush()
    except OSError as error:
        raise PreflightError("/dev/shm write probe failed") from error


def _module_info(kmod: Path, field: str) -> str:
    result = subprocess.run(
        ["modinfo", "-F", field, str(kmod)],
        check=False,
        capture_output=True,
        text=True,
    )
    value = result.stdout.strip()
    if result.returncode != 0 or not value:
        detail = result.stderr.strip() or f"empty {field}"
        raise PreflightError(f"cannot inspect workspace kmod {field}: {detail}")
    return value


def _verify_kmod(kmod: Path, module: str) -> str:
    try:
        loaded = Path(f"/sys/module/{module}/srcversion").read_text(
            encoding="ascii"
        ).strip().upper()
    except OSError as error:
        raise PreflightError(f"the {module} module is not loaded") from error
    workspace = _module_info(kmod, "srcversion").upper()
    if loaded != workspace:
        raise PreflightError(
            f"loaded/workspace kmod srcversion mismatch: {loaded} != {workspace}"
        )
    return workspace


def preflight() -> SystemConfig:
    """Validate the exact device and fork artifact set before child launch."""
    kmod = _required_file(
        "RCLPY_AGNOCAST_KMOD", _WORKSPACE / "agnocast/agnocast_kmod/agnocast_rclpy.ko"
    )
    hook = _required_file(
        "RCLPY_AGNOCAST_HEAPHOOK",
        _WORKSPACE / "agnocast/install/agnocastlib/lib/libagnocast_heaphook.so",
    )
    library = _required_file(
        "RCLPY_AGNOCAST_LIBRARY", hook.with_name("libagnocast.so")
    )
    if library != hook.with_name("libagnocast.so"):
        raise PreflightError("heaphook and libagnocast.so must be sibling artifacts")
    publisher_process = _required_file(
        "RCLPY_AGNOCAST_PUBLISHER_PROCESS",
        Path(__file__).resolve().parent / "processes/publisher_process.py",
    )
    subscriber = _subscriber_path()
    module = _module_info(kmod, "name")
    device = Path("/dev") / module
    numbers = _device_numbers(device, module)
    _verify_shm_rw()
    srcversion = _verify_kmod(kmod, module)
    return SystemConfig(
        hook, library, kmod, publisher_process, subscriber,
        device, numbers[0], numbers[1], srcversion,
    )


def emit(name: str, **fields) -> None:
    """Write the strict URL-escaped EVENT protocol consumed by every Probe."""
    tokens = [f"name={name}"]
    for key, value in fields.items():
        rendered = str(value).lower() if isinstance(value, bool) else str(value)
        tokens.append(f"{key}={quote(rendered, safe='._-/:+')}")
    print(EVENT_PREFIX + " ".join(tokens), flush=True)


def parse_event(line: str) -> dict[str, str] | None:
    """Parse one strict space-delimited EVENT record."""
    if not line.startswith(EVENT_PREFIX):
        return None
    fields: dict[str, str] = {}
    for token in line[len(EVENT_PREFIX):].split():
        key, separator, value = token.partition("=")
        if not separator or not key or key in fields:
            raise RuntimeError(f"malformed event token: {token!r}")
        fields[key] = value
    return fields


class Probe:
    """Own one interactive test child and retain its diagnostic transcript."""

    def __init__(self, command: list[str], environment: dict[str, str]) -> None:
        self.command = command
        self.process = subprocess.Popen(
            command,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            env=environment,
        )
        self.lines: queue.Queue[str | None] = queue.Queue()
        self.transcript: list[str] = []
        self.reader = threading.Thread(target=self._read_output, daemon=True)
        self.reader.start()

    def _read_output(self) -> None:
        assert self.process.stdout is not None
        for raw_line in self.process.stdout:
            line = raw_line.rstrip("\n")
            self.transcript.append(line)
            self.lines.put(line)
        self.lines.put(None)

    def send(self, command: str) -> None:
        """Send one line to a live child."""
        if self.process.poll() is not None or self.process.stdin is None:
            raise RuntimeError(self._failure("child is not running"))
        self.process.stdin.write(command + "\n")
        self.process.stdin.flush()

    def wait_event(self, name: str, timeout: float = 30.0) -> dict[str, str]:
        """Wait for one named event and fail immediately on a child ERROR event."""
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise RuntimeError(self._failure(f"timeout waiting for {name}"))
            try:
                line = self.lines.get(timeout=remaining)
            except queue.Empty as error:
                raise RuntimeError(self._failure(f"timeout waiting for {name}")) from error
            if line is None:
                raise RuntimeError(self._failure(f"child exited before {name}"))
            event = parse_event(line)
            if event is None:
                continue
            if event.get("name") == "ERROR":
                raise RuntimeError(self._failure(f"child ERROR: {event}"))
            if event.get("name") == name:
                return event

    def shutdown(self) -> None:
        """Request graceful shutdown, falling back to this exact child only."""
        if self.process.poll() is None:
            try:
                self.send("quit")
                self.process.wait(timeout=15.0)
            except (OSError, RuntimeError, subprocess.TimeoutExpired):
                self.process.kill()
                self.process.wait(timeout=5.0)

    def _failure(self, message: str) -> str:
        tail = "\n".join(self.transcript[-30:])
        return f"{message}: {' '.join(self.command)}\n{tail}"


def command(probe: Probe, text: str, event: str) -> dict[str, str]:
    """Send a command and wait for its named acknowledgement."""
    probe.send(text)
    return probe.wait_event(event)


@contextmanager
def launch_pair(subscriber_command: list[str], publisher_command: list[str],
                environment: dict[str, str]):
    """Own one connected pair, including cleanup when either child fails startup."""
    probes = []
    try:
        for arguments in (subscriber_command, publisher_command):
            probe = Probe(arguments, environment)
            probes.append(probe)
            probe.wait_event("READY")
        subscriber, publisher = probes
        wait_connected(subscriber)
        yield publisher, subscriber
    finally:
        for probe in reversed(probes):
            probe.shutdown()
        for probe in probes:
            assert probe.process.returncode == 0, "\n".join(probe.transcript[-30:])


def wait_connected(subscriber: Probe, timeout: float = 15.0) -> None:
    """Poll the subscriber's public endpoint count until a publisher is visible."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        event = command(subscriber, "connections", "CONNECTED")
        if int(event["count"]) >= 1 and event.get("connected") == "true":
            return
        time.sleep(0.05)
    raise RuntimeError("subscriber did not discover the publisher")


def payload_checksum(sequence: int, size: int) -> int:
    """Return the uint64 checksum used by both system child programs."""
    return sum((sequence + index) & 0xFF for index in range(size))
