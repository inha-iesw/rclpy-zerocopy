"""Fresh-process rclpy GuardCondition delivery and lifecycle scenarios."""

import argparse
import os
import shlex
import signal
import sys
import threading
import time

import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

from rclpy_agnocast import AgnocastRuntime

from checks import check_held, inspect, receipt_settled
from system_runtime import emit


def wait_until(predicate):
    """Wait for a native observation without taking or dispatching a Lease."""
    deadline = time.monotonic() + 10
    while not predicate():
        assert time.monotonic() < deadline, "observation timed out"
        time.sleep(0.001)


class Session:
    """Own a real Node, instrumented public guards, and explicit executor spins."""

    def __init__(self, arguments):
        self.arguments = arguments
        rclpy.init(args=[])
        self.node = rclpy.create_node(f"guard_tld_{os.getpid()}")
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.records = {}
        self._instrument_guards()
        self.runtime = AgnocastRuntime(self.node)
        self.mode = "inspect"
        self.received = []
        self.side_received = []
        self.errors = []
        self.busy = []
        self.held = []
        self.held_sequence = None
        self.subscription = self.create(self.callback)
        self.subscription_guard = self.subscription._guard
        self.side = None
        self.timer_count = 0
        self.service_count = 0
        self.service_samples = []
        self.response = None

    def _instrument_guards(self):
        create = self.node.create_guard_condition

        def instrument(callback, callback_group=None):
            record = {"triggers": [], "dispatches": []}

            def dispatch():
                record["dispatches"].append(threading.get_native_id())
                return callback()

            guard = create(dispatch, callback_group=callback_group)
            trigger = guard.trigger

            def observed_trigger():
                record["triggers"].append(threading.get_native_id())
                trigger()

            guard.trigger = observed_trigger
            self.records[guard] = record
            return guard

        self.node.create_guard_condition = instrument

    def create(self, callback):
        return self.runtime.create_subscription(
            self.arguments.topic, callback, depth=256,
            max_outstanding_leases=self.arguments.count, max_payload_bytes=self.arguments.limit,
        )

    def callback(self, lease):
        sequence = lease.sequence
        details = inspect(lease, self.subscription._handle._observation)
        details["python_tid"] = threading.get_native_id()
        self.received.append(details)
        if self.mode == "error":
            self.mode = "inspect"
            raise LookupError("guard callback sentinel")
        if self.mode == "hold-lease":
            self.held.append(lease)
            self.held_sequence = sequence
        elif self.mode == "hold-view":
            self.held.append(lease.buffer)
            self.held_sequence = sequence
        elif self.mode in ("close", "runtime-close"):
            lease.release()
            target = self.subscription if self.mode == "close" else self.runtime
            try:
                target.close()
            except RuntimeError as error:
                assert "dispatcher=1" in str(error)
                self.busy.append(str(error))
            else:
                raise AssertionError("close accepted its own active dispatcher")
        elif self.mode == "fairness":
            self.fairness_handler(lease)
        assert details["sequence"] == sequence

    def stats(self, name="STATS"):
        handle = self.subscription._handle
        stats = handle.stats
        observation = handle._observation
        record = self.records[self.subscription_guard]
        fields = {field: getattr(stats, field) for field in (
            "received_total", "delivered_total", "queued_leases", "outstanding_leases",
            "outstanding_payload_bytes", "dropped_outstanding", "dropped_oversize",
        )}
        emit(name, **fields, received_refs=observation.received,
             released_refs=observation.released, native_tid=observation.callback_tid,
             notifier_tid=self.subscription._notifier.native_id,
             notifier_alive=self.subscription._notifier.is_alive(),
             caller_tid=threading.get_native_id(), triggers=len(record["triggers"]),
             dispatches=len(record["dispatches"]),
             trigger_tids=",".join(map(str, set(record["triggers"]))),
             dispatch_tids=",".join(map(str, set(record["dispatches"]))),
             callbacks=len(self.received), side_callbacks=len(self.side_received),
             sequences=",".join(str(item["sequence"]) for item in self.received),
             guards=len(list(self.node.guards)), registry=len(self.runtime._subscriptions),
             timer_count=self.timer_count, service_count=self.service_count,
             service_samples=",".join(map(str, self.service_samples)),
             busy_count=len(self.busy))

    def spin(self, expected):
        deadline = time.monotonic() + 10
        while (len(self.received) < expected or (self.response is not None and (
                not self.response.done() or not self.timer_count))):
            assert time.monotonic() < deadline, "executor delivery timed out"
            self.executor.spin_once(timeout_sec=0.05)
        if self.response is not None:
            assert self.response.result() is not None
        self.stats("SPUN")

    def spin_error(self):
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            try:
                self.executor.spin_once(timeout_sec=0.05)
            except (LookupError, RuntimeError) as error:
                # Keep the traceback alive to expose a missing explicit Lease release.
                self.errors.append(error)
                emit("CALLBACK_ERROR", error=type(error).__name__, message=str(error))
                return
        raise AssertionError("executor did not propagate the expected error")

    def coexist(self):
        def tick():
            self.timer_count += 1
            if self.response is None or self.response.done():
                self.response = client.call_async(request)

        def respond(request, response):
            assert request.names == ["guard_tld"]
            self.service_count += 1
            self.service_samples.append(len(self.received))
            return response

        service_name = f"/guard_service_{os.getpid()}"
        self.node.create_service(GetParameters, service_name, respond)
        client = self.node.create_client(GetParameters, service_name)
        assert client.wait_for_service(timeout_sec=5)
        request = GetParameters.Request()
        request.names = ["guard_tld"]
        self.response = client.call_async(request)
        self.node.create_timer(0.001, tick)
        emit("COEXIST_READY")

    def release(self):
        value = self.held.pop()
        if isinstance(value, memoryview):
            check_held(value, self.held_sequence)
        else:
            assert value.sequence == self.held_sequence
            view = value.buffer
            check_held(view, value.sequence)
            view.release()
        value.release()
        del value
        self.stats("RELEASED")

    def close_endpoint(self, runtime=False):
        target = self.runtime if runtime else self.subscription
        try:
            target.close()
        except RuntimeError as error:
            emit("BUSY", message=str(error))
        else:
            self.stats("CLOSED_ENDPOINT")

    def stop(self, reason):
        if reason in ("cancel", "failure"):
            self.subscription._handle._test_stop_executor(reason == "failure")
        else:
            try:
                os.kill(os.getpid(), signal.SIGINT if reason == "sigint" else signal.SIGTERM)
            except KeyboardInterrupt:
                pass
            wait_until(lambda: not self.node.context.ok())
        self.subscription._notifier.join(timeout=5)
        assert not self.subscription._notifier.is_alive(), "idle notifier did not wake"
        emit("STOPPED", context_ok=self.node.context.ok())

    def competing_borrow(self):
        self.subscription.close()
        self.executor.remove_node(self.node)
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(self.node)
        publisher = self.runtime.create_publisher(
            self.arguments.topic + "/competing", depth=2, max_payload_bytes=16,
        )
        entered, attempted, finished = threading.Event(), threading.Event(), threading.Event()
        recovered = threading.Event()
        failures, tids = [], []
        result = {}

        def holder(lease):
            sequence = lease.sequence
            lease.release()
            try:
                if sequence == 700:
                    tids.append(threading.get_native_id())
                    with publisher.borrow(8):
                        entered.set()
                        assert attempted.wait(5), "independent callback did not run"
                else:
                    assert sequence == 701
                    with publisher.borrow(8) as loan:
                        loan.sequence = sequence
                    recovered.set()
            except BaseException as error:
                failures.append(error)
            finally:
                finished.set()

        def contender(lease):
            sequence = lease.sequence
            lease.release()
            if sequence != 700:
                return
            try:
                assert entered.wait(5), "first callback did not borrow"
                tids.append(threading.get_native_id())
                try:
                    loan = publisher.borrow(8)
                except RuntimeError as error:
                    result["error"] = str(error)
                else:
                    loan.cancel()
                    raise AssertionError("competing borrow unexpectedly succeeded")
            except BaseException as error:
                failures.append(error)
            finally:
                attempted.set()

        self.subscription = self.create(holder)
        self.subscription_guard = self.subscription._guard
        self.side = self.create(contender)
        assert self.subscription._guard.callback_group is not self.side._guard.callback_group
        try:
            emit("COMPETING_READY")
            deadline = time.monotonic() + 10
            while not finished.is_set():
                assert time.monotonic() < deadline
                executor.spin_once(timeout_sec=0.05)
            assert not failures, repr(failures)
            assert len(set(tids)) == 2
            assert result["error"]
            emit("COMPETING_RETRY_READY")
            deadline = time.monotonic() + 10
            while not recovered.is_set():
                assert time.monotonic() < deadline
                executor.spin_once(timeout_sec=0.05)
            assert not failures, repr(failures)
            emit("COMPETING", threads=len(set(tids)), message=result["error"], recovered=True)
        finally:
            assert executor.shutdown(timeout_sec=5)
            executor.remove_node(self.node)
            publisher.close()
            self.executor.add_node(self.node)

    def close_pending_guard(self):
        ready = self.executor.wait_for_ready_callbacks

        def close_after_schedule(*args, **kwargs):
            task, entity, node = ready(*args, **kwargs)
            assert entity is self.subscription_guard
            self.subscription.close()
            return task, entity, node

        # Close after rclpy constructs its task but before spin_once invokes it.
        self.executor.wait_for_ready_callbacks = close_after_schedule
        try:
            self.executor.spin_once(timeout_sec=5)
        finally:
            self.executor.wait_for_ready_callbacks = ready
        assert not self.received
        assert len(self.records[self.subscription_guard]["dispatches"]) == 1
        self.executor.spin_once(timeout_sec=0.01)
        self.stats("STALE_SUPPRESSED")

    def fairness(self):
        progress = {"timer": 0, "sibling": 0, "response": None}
        queued = []

        def slow(lease):
            lease.release()
            if progress["response"] is None:
                progress["response"] = client.call_async(GetParameters.Request())
                emit("FAIRNESS_STARTED")
            time.sleep(0.005)
            queued.append(self.subscription.stats.queued_leases)

        def tick():
            if progress["response"] is not None:
                progress["timer"] += 1

        def sibling(lease):
            lease.release()
            if progress["response"] is not None:
                progress["sibling"] += 1

        name = f"/guard_fairness_{os.getpid()}"
        service = self.node.create_service(
            GetParameters, name, lambda request, response: response,
        )
        client = self.node.create_client(GetParameters, name)
        assert client.wait_for_service(timeout_sec=3)
        timer = self.node.create_timer(0.01, tick)
        self.side = self.create(sibling)
        self.fairness_handler = slow
        self.mode = "fairness"
        emit("FAIRNESS_READY")
        try:
            deadline = time.monotonic() + 10
            while not (
                len(queued) >= 8 and progress["timer"] >= 2 and progress["sibling"]
                and progress["response"] is not None and progress["response"].done()
            ):
                assert time.monotonic() < deadline, "executor fairness timed out"
                self.executor.spin_once(timeout_sec=0.05)
            assert progress["response"].result() is not None
            emit("FAIRNESS_PROGRESS", callbacks=len(queued), timer=progress["timer"],
                 sibling=progress["sibling"], min_queued=min(queued), service=True)
        finally:
            self.mode = "inspect"
            self.fairness_handler = None
            self.node.destroy_timer(timer)
            self.node.destroy_client(client)
            self.node.destroy_service(service)

    def dispatch(self, fields):
        command, *arguments = fields
        if command == "connections":
            count = self.subscription.publisher_count
            emit("CONNECTED", count=count, connected=count > 0)
        elif command == "wait":
            wait_until(lambda: receipt_settled(self.subscription._handle, int(arguments[0])))
            wait_until(lambda: bool(self.records[self.subscription._guard]["triggers"]))
            self.stats()
        elif command == "stats":
            self.stats()
        elif command == "spin":
            self.spin(int(arguments[0]))
        elif command == "spin-forget":
            self.spin(int(arguments[0]))
            self.received.clear()
        elif command == "spin-error":
            self.spin_error()
        elif command == "details":
            emit("DETAILS", **self.received[-1])
        elif command == "mode":
            self.mode = arguments[0]
            emit("MODE")
        elif command == "coexist":
            self.coexist()
        elif command == "release":
            self.release()
        elif command in ("close", "runtime-close"):
            self.close_endpoint(command == "runtime-close")
        elif command == "side-create":
            self.side = self.create(lambda lease: self.side_received.append(lease.sequence))
            emit("SIDE_READY")
        elif command == "side-spin":
            deadline = time.monotonic() + 10
            while len(self.side_received) < int(arguments[0]):
                assert time.monotonic() < deadline
                self.executor.spin_once(timeout_sec=0.05)
            self.stats("SIDE_SPUN")
        elif command == "recreate":
            previous = self.subscription
            previous_guard = self.subscription_guard
            previous.close()
            assert not previous._notifier.is_alive()
            assert previous_guard not in list(self.node.guards)
            self.subscription = self.create(self.callback)
            self.subscription_guard = self.subscription._guard
            self.received.clear()
            self.stats("RECREATED")
        elif command == "stop":
            self.stop(arguments[0])
        elif command == "competing-borrow":
            self.competing_borrow()
        elif command == "close-pending":
            self.close_pending_guard()
        elif command == "fairness":
            self.fairness()
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
        if self.side is not None:
            self.side.close()
        self.subscription.close()
        self.runtime.close()
        assert not list(self.node.guards)
        assert not self.runtime._subscriptions
        assert not self.subscription._notifier.is_alive()
        assert self.executor.shutdown(timeout_sec=5)
        self.node.destroy_node()
        rclpy.try_shutdown()


def main():
    """Start the instrumented public adapter in the preflight-selected process."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", required=True)
    parser.add_argument("--count", type=int, default=256)
    parser.add_argument("--limit", type=int, default=2 * 1024 * 1024)
    session = Session(parser.parse_args())
    try:
        session.run()
    finally:
        session.close()


if __name__ == "__main__":
    try:
        main()
    except BaseException as error:  # Also report uncaught signal/callback interrupts to Probe.
        emit("ERROR", error=type(error).__name__, message=str(error))
        raise
