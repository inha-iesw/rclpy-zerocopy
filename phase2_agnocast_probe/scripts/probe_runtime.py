"""Process and resource helpers for the Agnocast Phase 2 probe."""

from pathlib import Path
import queue
import subprocess
import threading
import time


EVENT_PREFIX = "EVENT "


def require(condition, message):
    """Raise a stable failure when a characterization invariant is false."""
    if not condition:
        raise RuntimeError(message)


def parse_event(line):
    """Parse one probe event line without accepting ambiguous fields."""
    if not line.startswith(EVENT_PREFIX):
        return None
    fields = {}
    for token in line[len(EVENT_PREFIX):].split():
        key, separator, value = token.partition("=")
        if not separator or not key or key in fields:
            raise RuntimeError(f"malformed event token: {token!r}")
        fields[key] = value
    return fields


class Probe:
    """Interactive probe child with event-oriented stdout handling."""

    active = []

    def __init__(self, binary, mode, arguments, environment):
        self.command_line = [str(binary), mode, *map(str, arguments)]
        self.process = subprocess.Popen(
            self.command_line,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            env=environment,
        )
        self.lines = queue.Queue()
        self.transcript = []
        self.reader = threading.Thread(target=self._read_output, daemon=True)
        self.reader.start()
        Probe.active.append(self)

    @property
    def pid(self):
        """Return the exact child PID used for targeted signals and shm lookup."""
        return self.process.pid

    def _read_output(self):
        for raw_line in self.process.stdout:
            line = raw_line.rstrip("\n")
            self.transcript.append(line)
            self.lines.put(line)
        self.lines.put(None)

    def send(self, command):
        """Send one line-oriented command to a running child."""
        require(self.process.poll() is None, f"probe already exited: {self.command_line}")
        self.process.stdin.write(command + "\n")
        self.process.stdin.flush()

    def wait_event(self, name, timeout=15.0):
        """Wait for one named event while retaining a diagnostic transcript."""
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise RuntimeError(self._failure_text(f"timeout waiting for {name}"))
            try:
                line = self.lines.get(timeout=remaining)
            except queue.Empty as error:
                raise RuntimeError(self._failure_text(f"timeout waiting for {name}")) from error
            if line is None:
                raise RuntimeError(self._failure_text(f"exited before {name}"))
            event = parse_event(line)
            if event is not None and event.get("name") == name:
                return event

    def wait(self, timeout=10.0, expected=0):
        """Wait for the exact expected exit status."""
        status = self.process.wait(timeout=timeout)
        require(status == expected, self._failure_text(f"exit={status}, expected={expected}"))
        self._forget()
        return status

    def quit(self):
        """Request the probe's normal shutdown path."""
        if self.process.poll() is None:
            self.send("quit")
        self.wait()

    def kill(self):
        """Send SIGKILL only to this exact probe PID."""
        if self.process.poll() is None:
            self.process.kill()
        status = self.process.wait(timeout=10.0)
        require(status < 0, self._failure_text(f"SIGKILL expected, exit={status}"))
        self._forget()

    def force_cleanup(self):
        """Best-effort cleanup used only after a failed characterization."""
        if self.process.poll() is None:
            self.process.kill()
            self.process.wait(timeout=5.0)
        self._forget()

    def _forget(self):
        if self in Probe.active:
            Probe.active.remove(self)

    def _failure_text(self, message):
        tail = "\n".join(self.transcript[-20:])
        return f"{message}: {' '.join(self.command_line)}\n{tail}"

    @classmethod
    def cleanup_all(cls):
        """Terminate only probe children still owned by this runner."""
        for probe in list(reversed(cls.active)):
            probe.force_cleanup()


def wait_for_connection(probe, timeout=10.0):
    """Wait until the opposite endpoint is visible through the public API."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        probe.send("connections")
        event = probe.wait_event("CONNECTIONS")
        if int(event["count"]) >= 1:
            return
        time.sleep(0.1)
    raise RuntimeError("endpoint connection was not established")


def shm_allocated_bytes(pid):
    """Return physical blocks allocated to one sparse Agnocast pool."""
    path = Path(f"/dev/shm/agnocast@{pid}")
    deadline = time.monotonic() + 3.0
    while not path.exists() and time.monotonic() < deadline:
        time.sleep(0.05)
    require(path.exists(), f"missing Agnocast shm pool: {path}")
    return path.stat().st_blocks * 512


def wait_gone(path, timeout=5.0):
    """Wait for the unlink daemon to remove one resource name."""
    deadline = time.monotonic() + timeout
    while Path(path).exists() and time.monotonic() < deadline:
        time.sleep(0.1)
    return not Path(path).exists()


def child_pids(parent_pid):
    """Return the exact direct children currently reported for one process."""
    path = Path(f"/proc/{parent_pid}/task/{parent_pid}/children")
    return [int(pid) for pid in path.read_text(encoding="ascii").split()]


def wait_for_only_child(parent_pid, timeout=3.0):
    """Return a controlled probe's sole daemon child PID."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        children = child_pids(parent_pid)
        if children:
            require(len(children) == 1, f"expected one daemon child, found {children}")
            return children[0]
        time.sleep(0.05)
    raise RuntimeError(f"daemon child was not found for PID {parent_pid}")


def process_is_running(pid):
    """Return false for an absent process or an unreaped zombie."""
    path = Path(f"/proc/{pid}/stat")
    try:
        fields = path.read_text(encoding="ascii").split()
    except FileNotFoundError:
        return False
    return len(fields) > 2 and fields[2] != "Z"


def wait_until_not_running(pid, timeout=3.0):
    """Wait until a signaled process is gone or retained only as a zombie."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if not process_is_running(pid):
            return
        time.sleep(0.02)
    raise RuntimeError(f"PID {pid} remained runnable")


def topic_mqueues(topic):
    """List only message queues whose normalized name contains this topic."""
    marker = topic.strip("/").replace("/", "_")
    root = Path("/dev/mqueue")
    require(root.is_dir(), "/dev/mqueue is not mounted")
    return sorted(str(path) for path in root.glob("agnocast@*") if marker in path.name)
