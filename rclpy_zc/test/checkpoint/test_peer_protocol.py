"""Checkpoint commands must not require a particular rclpy dispatch implementation."""

import io
from types import SimpleNamespace

import pytest

from checkpoint.harness import Peers
from subscriber import guard_process


class PublicSurface:
    """Poison dispatch internals, including access through getattr with a default."""

    def __getattr__(self, name):
        if name in {"_guard", "_notifier", "guards", "_subscriptions", "create_guard_condition"}:
            raise AssertionError(f"checkpoint inspected dispatch internals: {name}")
        raise AttributeError(name)


class Subscription(PublicSurface):
    """Expose only the checkpoint's public counters and native observation seam."""

    def __init__(self, callback):
        self.callback = callback
        self.publisher_count = 1
        self.closed = False
        self._handle = SimpleNamespace(
            stats=SimpleNamespace(received_total=1, delivered_total=0, queued_leases=1,
                                  outstanding_leases=1, outstanding_payload_bytes=1,
                                  dropped_outstanding=0, dropped_oversize=0),
            _observation=SimpleNamespace(received=1, released=0),
        )

    @property
    def stats(self):
        return self._handle.stats

    def close(self):
        self.closed = True


class Runtime(PublicSurface):
    """Retain the callback for one executor-driven delivery in the protocol test."""

    def create_subscription(self, topic, callback, **kwargs):
        self.subscription = Subscription(callback)
        return self.subscription

    def close(self):
        assert self.subscription.closed


@pytest.mark.parametrize("mode", ["inspect", "hold-lease", "hold-view"])
def test_checkpoint_commands_do_not_access_guard_notifier_or_registries(monkeypatch, mode):
    runtime = Runtime()
    node = PublicSurface()
    node.destroy_node = lambda: None
    executor = SimpleNamespace(add_node=lambda node: None, shutdown=lambda **kwargs: True)
    lease = SimpleNamespace(sequence=1, buffer=memoryview(b"\x01"), release=lambda: None)

    def spin(**kwargs):
        runtime.subscription.callback(lease)
        runtime.subscription.stats.delivered_total += 1
        runtime.subscription.stats.queued_leases = 0

    executor.spin_once = spin
    monkeypatch.setattr(guard_process.rclpy, "init", lambda **kwargs: None)
    monkeypatch.setattr(guard_process.rclpy, "create_node", lambda name: node)
    monkeypatch.setattr(guard_process.rclpy, "try_shutdown", lambda: None)
    monkeypatch.setattr(guard_process, "AgnocastRuntime", lambda node: runtime)
    monkeypatch.setattr(guard_process, "SingleThreadedExecutor", lambda: executor)
    monkeypatch.setattr(guard_process, "inspect", lambda lease, observation: {"sequence": 1})
    events = []
    monkeypatch.setattr(guard_process, "emit",
                        lambda name, **fields: events.append((name, fields)))
    # Keep constructor/run/dispatch/stats/recreate/close real; only transport and rclpy are fake.
    arguments = SimpleNamespace(topic="/test", count=2, limit=8, checkpoint=True)
    session = guard_process.Session(arguments)
    lines = ["connections", "stats", f"mode {mode}", "wait 1", "spin 1", "details"]
    if mode != "inspect":
        lines.append("release")
    lines.extend(["recreate", "quit"])
    monkeypatch.setattr(guard_process.sys, "stdin", io.StringIO("\n".join(lines) + "\n"))
    try:
        session.run()
        assert session.received == []  # recreate resets cumulative callback count.
        names = [name for name, fields in events]
        assert names == ["READY", "CONNECTED", "STATS", "MODE", "STATS", "SPUN", "DETAILS",
                         *(["RELEASED"] if mode != "inspect" else []), "RECREATED"]
        stats = next(fields for name, fields in events if name == "SPUN")
        assert stats["received_total"] == stats["delivered_total"] == 1
        assert stats["sequences"] == "1"
        assert "caller_tid" in stats and "received_refs" in stats
        assert not {"triggers", "dispatches", "notifier_tid", "guards", "registry"} & stats.keys()
        with pytest.raises(ValueError, match="checkpoint"):
            session.dispatch(["close-pending"])
        with pytest.raises(ValueError, match="checkpoint"):
            session.dispatch(["mode", "fairness"])
    finally:
        session.close()


def test_checkpoint_harness_selects_dispatch_independent_child_mode():
    peers = Peers(SimpleNamespace(child_environment=lambda domain: {}), 2, 8)
    peers._start = lambda arguments: arguments
    assert "--checkpoint" in peers.subscriber()
