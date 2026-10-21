"""Expose interpreter shutdown with intentionally omitted adapter cleanup."""

import argparse
import atexit
import os
from threading import RLock
import time
from types import SimpleNamespace

import rclpy

from rclpy_agnocast import AgnocastRuntime, _native
from rclpy_agnocast.subscription import Subscription


LIVE = []


def main():
    """Leave ownership live when main returns or propagates its original exception."""
    parser = argparse.ArgumentParser()
    parser.add_argument("backend", choices=("synthetic", "native"))
    parser.add_argument(
        "scenario", choices=(
            "return", "shutdown-return", "exception", "late-close", "queued-return", "held-return",
        )
    )
    arguments = parser.parse_args()
    rclpy.init(args=[])
    node = rclpy.create_node(f"subscription_shutdown_{os.getpid()}")
    held = []

    def callback(lease):
        if arguments.scenario == "held-return":
            held.extend((lease, lease.buffer))

    topic = f"/shutdown_{os.getpid()}"
    if arguments.backend == "native":
        owner = AgnocastRuntime(node)
        subscription = owner.create_subscription(
            topic, callback, depth=2, max_payload_bytes=16,
        )
    else:
        owner = SimpleNamespace(_rclpy_node=node, _subscriptions=set(), _lifecycle_lock=RLock())
        subscription = Subscription(
            _native._TestSubscription(2, 16), callback, runtime_owner=owner,
        )
        owner._subscriptions.add(subscription)
    LIVE.extend((node, owner, subscription, held))
    assert not subscription._notifier.daemon
    if arguments.scenario in ("queued-return", "held-return"):
        if arguments.backend == "native":
            publisher = owner.create_publisher(topic, depth=2, max_payload_bytes=8)
            LIVE.append(publisher)
            with publisher.borrow(8) as loan:
                loan.sequence = 7
                view = loan.buffer
                view[:] = b"payload!"
                view.release()
                loan.publish()
        else:
            assert subscription._handle.push(7, b"payload!")
        deadline = time.monotonic() + 2
        while subscription.stats.queued_leases != 1:
            assert time.monotonic() < deadline, "initial receipt timed out"
            time.sleep(0.001)
        if arguments.scenario == "held-return":
            while not held:
                assert time.monotonic() < deadline, "held callback timed out"
                rclpy.spin_once(node, timeout_sec=0.02)

        def check_owners_after_notifier_shutdown():
            # Python atexit follows threading shutdown, before module globals disappear.
            assert not subscription._notifier.is_alive()
            assert subscription.stats.outstanding_leases == 1
            assert subscription.stats.outstanding_payload_bytes == 8
            if arguments.scenario == "held-return":
                assert subscription.stats.queued_leases == 0
                assert held[0].sequence == 7
                assert held[1].tobytes() == b"payload!"
            else:
                assert subscription.stats.queued_leases == 1
            print("OWNERS_PRESERVED", flush=True)

        atexit.register(check_owners_after_notifier_shutdown)
    print("READY", flush=True)
    if arguments.scenario in ("shutdown-return", "late-close"):
        node.destroy_node()
        rclpy.shutdown()
    if arguments.scenario == "late-close":
        subscription.close()
        if arguments.backend == "native":
            owner.close()
        assert not subscription._notifier.is_alive()
        assert not owner._subscriptions
        print("CLOSED", flush=True)
    if arguments.scenario == "exception":
        raise LookupError("shutdown-original-sentinel")
    print("MAIN_RETURNED", flush=True)


if __name__ == "__main__":
    main()
