"""Real rclpy scheduling while a native test queue stays continuously supplied."""

import os
from pathlib import Path
import sys
from threading import Event, Lock, RLock, Thread
import time
from types import SimpleNamespace
import uuid

import pytest
import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

from rclpy_agnocast import _native
from rclpy_agnocast.subscription import Subscription

from system_runtime import launch_pair, preflight, system_tld_requested


@pytest.mark.parametrize("threads", [1, 4])
def test_supplied_subscription_yields_to_timer_service_and_sibling_without_overlap(
        threads, tmp_path, monkeypatch):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros-log"))
    context = Context()
    rclpy.init(context=context)
    node = rclpy.create_node(f"fairness_{os.getpid()}", context=context)
    owner = SimpleNamespace(_rclpy_node=node, _subscriptions=set(), _lifecycle_lock=RLock())
    executor = (SingleThreadedExecutor(context=context) if threads == 1
                else MultiThreadedExecutor(num_threads=threads, context=context))
    executor.add_node(node)
    stopped, started, sibling_seen = Event(), Event(), Event()
    lock = Lock()
    counts = {"active": 0, "max_active": 0, "callbacks": 0, "timer": 0}
    failures = []
    primary = _native._TestSubscription(4, 32)
    sibling = _native._TestSubscription(1, 8)

    def callback(lease):
        sequence = lease.sequence
        lease.release()
        with lock:
            counts["active"] += 1
            counts["max_active"] = max(counts["max_active"], counts["active"])
            counts["callbacks"] += 1
        try:
            started.set()
            time.sleep(0.005)
            if not stopped.is_set():
                assert primary.push(sequence + 4, b"payload!")
        finally:
            with lock:
                counts["active"] -= 1

    def sibling_callback(lease):
        lease.release()
        if started.is_set() and not stopped.is_set():
            sibling_seen.set()

    def tick():
        if started.is_set() and not stopped.is_set():
            counts["timer"] += 1

    node.create_timer(0.01, tick)
    service_name = f"/fairness_service_{os.getpid()}"
    node.create_service(GetParameters, service_name, lambda request, response: response)
    client = node.create_client(GetParameters, service_name)
    assert client.wait_for_service(timeout_sec=3)
    subscriptions = [
        Subscription(primary, callback, runtime_owner=owner),
        Subscription(sibling, sibling_callback, runtime_owner=owner),
    ]
    owner._subscriptions.update(subscriptions)

    def spin():
        try:
            while not stopped.is_set():
                executor.spin_once(timeout_sec=0.02)
        except BaseException as error:
            failures.append(error)

    worker = Thread(target=spin)
    for sequence in range(4):
        assert primary.push(sequence, b"payload!")
    worker.start()
    try:
        assert started.wait(2), "primary callback did not start"
        response = client.call_async(GetParameters.Request())
        assert sibling.push(1, b"sibling!")
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline and not (
                response.done() and sibling_seen.is_set() and counts["timer"] >= 2
                and counts["callbacks"] >= 12):
            assert not failures, repr(failures)
            if threads > 1:
                subscriptions[0]._guard.trigger()
            time.sleep(0.001)
        assert counts["timer"] >= 2, "timer starved while primary queue stayed supplied"
        assert response.done(), "service starved while primary queue stayed supplied"
        assert response.result() is not None
        assert sibling_seen.is_set(), "sibling starved while primary queue stayed supplied"
        assert primary.stats.queued_leases >= 2
        assert counts["max_active"] == 1
        assert counts["callbacks"] >= 12
    finally:
        stopped.set()
        worker.join(timeout=3)
        assert not worker.is_alive()
        assert executor.shutdown(timeout_sec=3)
        for subscription in subscriptions:
            subscription.close()
        node.destroy_node()
        rclpy.shutdown(context=context)
    assert not failures, repr(failures)


def test_native_slow_callback_allows_ros_progress_before_sustained_publisher_stops(tmp_path):
    if not system_tld_requested():
        pytest.skip("set RCLPY_AGNOCAST_RUN_SYSTEM_TLD=1 for actual fairness TLD")
    config = preflight()
    environment = config.child_environment(20 + uuid.uuid4().int % 180)
    environment["ROS_LOG_DIR"] = str(tmp_path / "ros-log")
    topic = f"/fairness_tld_{uuid.uuid4().hex}"
    subscriber_command = [
        sys.executable, str(Path(__file__).with_name("guard_process.py")),
        "--topic", topic, "--count", "8", "--limit", "64",
    ]
    publisher_command = [
        sys.executable, str(config.publisher_process), "--topic", topic,
        "--depth", "256", "--max-payload-bytes", "64",
    ]
    failures = []
    stopped = Event()
    producer = None
    with launch_pair(subscriber_command, publisher_command, environment) as pair:
        publisher, subscriber = pair
        try:
            subscriber.send("fairness")
            subscriber.wait_event("FAIRNESS_READY")

            def publish():
                sequence = 0
                try:
                    while not stopped.is_set():
                        publisher.send(f"publish 8 {sequence}")
                        publisher.wait_event("PUBLISHED", timeout=3)
                        sequence += 1
                except BaseException as error:
                    failures.append(error)

            producer = Thread(target=publish)
            producer.start()
            subscriber.wait_event("FAIRNESS_STARTED")
            result = subscriber.wait_event("FAIRNESS_PROGRESS", timeout=2)
            assert producer.is_alive() and not stopped.is_set()
            assert not failures, repr(failures)
            assert int(result["callbacks"]) >= 8
            assert int(result["timer"]) >= 2
            assert int(result["sibling"]) >= 1
            assert int(result["min_queued"]) >= 1
            assert result["service"] == "true"
        finally:
            stopped.set()
            if producer is not None:
                producer.join(timeout=4)
                assert not producer.is_alive()
    assert not failures, repr(failures)
