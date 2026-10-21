"""Fresh-process native subscriber TLD; GuardCondition dispatch belongs to Phase 3.4."""

import argparse
import gc
import os
import shlex
import signal
import sys
import threading
import time

import numpy as np
import rclpy

from rclpy_agnocast import AgnocastRuntime
from rclpy_agnocast import _native

from checks import check_held
from checks import inspect
from checks import receipt_settled
from system_runtime import emit


class Session:
    """Own real subscriber resources and controlled held aliases."""

    def __init__(self, arguments):
        self.arguments = arguments
        rclpy.init(args=[])
        self.node = rclpy.create_node(f"subscriber_tld_{os.getpid()}")
        self.runtime = AgnocastRuntime(self.node)
        self.subscription = self.create()
        self.side = None
        self.held = []
        self.sequence = 0
        self.offset = 0

    def create(self):
        return self.runtime._native_runtime.create_subscription(
            self.arguments.topic, 128, self.arguments.count, self.arguments.limit
        )

    def stats(self, name="STATS"):
        stats = self.subscription.stats
        observation = self.subscription._observation
        emit(name, **{field: getattr(stats, field) for field in (
            "received_total", "delivered_total", "dropped_outstanding", "dropped_oversize",
            "queued_leases", "outstanding_leases", "outstanding_payload_bytes",
        )}, received_refs=observation.received, released_refs=observation.released,
             callback_tid=observation.callback_tid, release_tid=observation.release_tid,
             caller_tid=threading.get_native_id())

    def wait(self, expected):
        deadline = time.monotonic() + 10
        while True:
            if receipt_settled(self.subscription, expected):
                return
            if time.monotonic() >= deadline:
                raise AssertionError("native receipt did not reach expected count")
            time.sleep(0.001)

    def take(self, subscription):
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            lease = subscription.take()
            if lease is not None:
                return lease
            time.sleep(0.001)
        raise AssertionError("expected a queued Lease")

    def stop_executor(self, reason):
        if reason in ("cancel", "failure"):
            self.subscription._test_stop_executor(reason == "failure")
        else:
            signum = signal.SIGINT if reason == "sigint" else signal.SIGTERM
            try:
                os.kill(os.getpid(), signum)
            except KeyboardInterrupt:
                pass
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            try:
                self.subscription.take()
            except RuntimeError as error:
                emit("EXECUTOR_STOPPED", message=str(error))
                return
            time.sleep(0.001)
        raise AssertionError("executor remained active after shutdown")

    def inspect(self, subscription):
        with self.take(subscription) as lease:
            # Last callback observation addresses are valid for single outstanding receipt.
            details = inspect(lease, subscription._observation)
        lease.release()
        for getter in (lambda: lease.sequence, lambda: lease.buffer):
            try:
                getter()
            except RuntimeError:
                pass
            else:
                raise AssertionError("released Lease remained accessible")
        emit("INSPECTED", **details)

    def hold(self, kind):
        assert not self.held
        lease = self.take(self.subscription)
        self.sequence = lease.sequence
        self.offset = 0
        source = lease.buffer
        if kind == "numpy":
            value = np.frombuffer(source, dtype=np.uint8)
            del source
        elif kind == "slice":
            value = source[1:]
            self.offset = 1
            source.release()
        else:
            assert kind == "view"
            value = source
        lease.release()
        self.held.append(value)
        check_held(value, self.sequence, self.offset)
        self.stats("HELD")

    def release(self, foreign=False):
        tids = []

        def drop():
            tids.append(threading.get_native_id())
            value = self.held.pop()
            if isinstance(value, memoryview):
                value.release()
            del value
            gc.collect()

        if foreign:
            thread = threading.Thread(target=drop)
            thread.start()
            thread.join(timeout=5)
            assert not thread.is_alive()
        else:
            drop()
        assert self.subscription._observation.release_tid == tids[0]
        self.stats("RELEASED")

    def failure_priority(self, reason):
        def failure(operation):
            try:
                operation()
            except RuntimeError as error:
                return str(error)
            raise AssertionError("operation did not expose its terminal failure")

        handle = self.subscription
        handle._test_fail_receive()
        receive_take = failure(handle.take)
        receive_wait = failure(lambda: handle.wait_for_wake(0))
        handle._test_stop_executor(reason == "failure")
        executor_take = failure(handle.take)
        preserved_wait = failure(lambda: handle.wait_for_wake(0))
        handle.prepare_close()
        stopped_wait = handle.wait_for_wake(0)
        closing_take = failure(handle.take)
        handle.close()
        emit("FAILURE_PRIORITY", receive_take=receive_take, receive_wait=receive_wait,
             executor_take=executor_take, preserved_wait=preserved_wait,
             stopped_wait=stopped_wait, closing_take=closing_take)

    def dispatch(self, fields):
        command, *arguments = fields
        if command == "connections":
            count = self.subscription.publisher_count
            emit("CONNECTED", count=count, connected=count > 0)
        elif command == "wait":
            self.wait(int(arguments[0]))
            self.stats()
        elif command == "inspect":
            self.inspect(self.subscription)
        elif command == "hold":
            self.hold(arguments[0])
        elif command == "check-held":
            check_held(self.held[0], self.sequence, self.offset)
            emit("HELD_VALID")
        elif command == "release":
            self.release(bool(arguments))
        elif command == "stats":
            self.stats()
        elif command in ("close", "runtime-close"):
            target = self.subscription if command == "close" else self.runtime
            try:
                target.close()
            except RuntimeError as error:
                assert "Lease" in str(error)
                self.stats("BUSY")
            else:
                self.stats("CLOSED_ENDPOINT")
        elif command == "recreate":
            self.subscription.close()
            self.subscription = self.create()
            emit("RECREATED")
        elif command == "side-create":
            self.side = self.create()
            emit("SIDE_READY")
        elif command == "side-inspect":
            self.inspect(self.side)
        elif command == "block-gil":
            assert _native._test_wait_for_receipts_holding_gil(
                self.subscription, int(arguments[0]), 10000, sys.stdout.fileno()
            ), "native receipt required the GIL"
            self.stats("GIL_PROGRESS")
        elif command == "stop-executor":
            self.stop_executor(arguments[0])
        elif command == "fail-next-control":
            self.subscription._test_fail_next_control()
            emit("CONTROL_ARMED")
        elif command == "failure-priority":
            self.failure_priority(arguments[0])
        else:
            raise ValueError(f"unknown command: {command}")

    def run(self):
        emit("READY", pid=os.getpid())
        for line in sys.stdin:
            fields = shlex.split(line)
            if not fields:
                continue
            if fields[0] == "quit":
                return
            self.dispatch(fields)

    def close(self):
        if self.held:
            self.release()
        if self.side:
            self.side.close()
        self.subscription.close()
        self.runtime.close()
        self.node.destroy_node()
        rclpy.try_shutdown()


def main():
    """Start only in a fresh child with its exact heaphook preloaded."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", required=True)
    parser.add_argument("--count", type=int, required=True)
    parser.add_argument("--limit", type=int, required=True)
    session = Session(parser.parse_args())
    try:
        session.run()
    finally:
        session.close()


if __name__ == "__main__":
    try:
        main()
    except Exception as error:  # Expected signal interruptions are handled by stop_executor.
        emit("ERROR", error=type(error).__name__, message=str(error))
        raise
