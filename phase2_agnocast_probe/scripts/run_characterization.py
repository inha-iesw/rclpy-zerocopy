#!/usr/bin/env python3
"""Run the Phase 2 Agnocast characterization matrix."""

import argparse
import json
import os
from pathlib import Path
import signal
import time
import uuid

from probe_runtime import (
    Probe,
    child_pids,
    process_is_running,
    require,
    shm_allocated_bytes,
    topic_mqueues,
    wait_for_connection,
    wait_for_only_child,
    wait_gone,
    wait_until_not_running,
)


def start_pair(binary, environment, topic, payload_bytes, sequence, hold_count):
    subscriber = Probe(binary, "subscriber", [topic, 1, hold_count], environment)
    subscriber_ready = subscriber.wait_event("READY")
    publisher = Probe(binary, "publisher", [topic, 1, payload_bytes, sequence], environment)
    publisher_ready = publisher.wait_event("READY")
    require(int(subscriber_ready["pid"]) == subscriber.pid, "subscriber PID mismatch")
    require(int(publisher_ready["pid"]) == publisher.pid, "publisher PID mismatch")
    wait_for_connection(publisher)
    return publisher, subscriber


def publish_and_receive(publisher, subscriber):
    publisher.send("publish")
    published = publisher.wait_event("PUB", timeout=30.0)
    received = subscriber.wait_event("RX", timeout=30.0)
    for field in ("sequence", "bytes", "checksum", "msg_addr", "data_addr"):
        require(published[field] == received[field], f"roundtrip {field} mismatch")
    require(received["valid"] == "true", "subscriber payload validation failed")
    return published, received


def run_roundtrip(binary, environment, results):
    topic = f"/phase2_probe_roundtrip_{uuid.uuid4().hex}"
    publisher, subscriber = start_pair(binary, environment, topic, 1024 * 1024, 100, 0)
    try:
        published, received = publish_and_receive(publisher, subscriber)
        require(publisher.pid != subscriber.pid, "roundtrip did not use separate processes")
        results["roundtrip"] = {
            "publisher_pid": publisher.pid,
            "subscriber_pid": subscriber.pid,
            "bytes": int(published["bytes"]),
            "checksum": int(published["checksum"]),
            "msg_addr": published["msg_addr"],
            "data_addr": published["data_addr"],
            "entry_id": int(received["entry_id"]),
        }
    finally:
        publisher.quit()
        subscriber.quit()


def run_held_qos(binary, environment, results):
    topic = f"/phase2_probe_held_{uuid.uuid4().hex}"
    payload_bytes = 8 * 1024 * 1024
    publisher, subscriber = start_pair(binary, environment, topic, payload_bytes, 500, 8)
    try:
        baseline = shm_allocated_bytes(publisher.pid)
        entry_ids = []
        for _ in range(8):
            _, received = publish_and_receive(publisher, subscriber)
            require(received["held"] == "true", "subscriber did not hold expected lease")
            entry_ids.append(int(received["entry_id"]))
        peak = shm_allocated_bytes(publisher.pid)
        subscriber.send("release")
        released = subscriber.wait_event("RELEASE")
        after_release = shm_allocated_bytes(publisher.pid)
        trigger_allocated = []
        for _ in range(4):
            publish_and_receive(publisher, subscriber)
            time.sleep(0.1)
            trigger_allocated.append(shm_allocated_bytes(publisher.pid))
        require(int(released["count"]) == 8, "held lease release count mismatch")
        require(len(set(entry_ids)) == 8, "held messages did not have distinct entry IDs")
        require(peak > baseline, "held leases did not increase publisher shm allocation")
        require(
            trigger_allocated[-1] < peak,
            "trigger publishes did not reclaim released messages",
        )
        require(
            trigger_allocated[-1] <= baseline + 2 * payload_bytes,
            "released backlog did not converge near the depth-one steady state",
        )
        results["held_qos_release"] = {
            "depth": 1,
            "held": 8,
            "payload_bytes": payload_bytes,
            "baseline_allocated": baseline,
            "peak_allocated": peak,
            "after_release_allocated": after_release,
            "trigger_allocated": trigger_allocated,
            "entry_ids": entry_ids,
        }
    finally:
        publisher.quit()
        subscriber.quit()


def run_exhaustion(binary, environment, reserve_bytes, results):
    topic = f"/phase2_probe_exhaust_{uuid.uuid4().hex}"
    probe = Probe(binary, "exhaust", [topic, reserve_bytes], environment)
    failed = probe.wait_event("BAD_ALLOC", timeout=30.0)
    recovered = probe.wait_event("RECOVERY_OK", timeout=30.0)
    probe.wait(timeout=10.0)
    results["allocator_exhaustion"] = {
        "reserve_bytes": int(failed["reserve_bytes"]),
        "recovery_bytes": int(recovered["bytes"]),
        "recovery_checksum": int(recovered["checksum"]),
    }


def run_exit_recreate(binary, environment, results):
    topic = f"/phase2_probe_exit_{uuid.uuid4().hex}"
    publisher, subscriber = start_pair(binary, environment, topic, 1024 * 1024, 1000, 1)
    first_publisher_pid, first_subscriber_pid = publisher.pid, subscriber.pid
    publish_and_receive(publisher, subscriber)
    publisher.kill()
    publisher_shm_cleaned = wait_gone(f"/dev/shm/agnocast@{first_publisher_pid}")
    subscriber.send("check")
    held_check = subscriber.wait_event("HELD_CHECK")
    require(held_check["valid"] == "true", "held lease failed after publisher SIGKILL")

    publisher = Probe(binary, "publisher", [topic, 1, 1024 * 1024, 1001], environment)
    publisher.wait_event("READY")
    wait_for_connection(publisher)
    publish_and_receive(publisher, subscriber)
    subscriber.kill()
    subscriber_shm_cleaned = wait_gone(f"/dev/shm/agnocast@{first_subscriber_pid}")
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        publisher.send("connections")
        event = publisher.wait_event("CONNECTIONS")
        if int(event["count"]) == 0:
            break
        time.sleep(0.1)
    require(int(event["count"]) == 0, "killed subscriber remained registered")
    publisher.send("publish")
    publisher.wait_event("PUB")

    subscriber = Probe(binary, "subscriber", [topic, 1, 0], environment)
    subscriber.wait_event("READY")
    wait_for_connection(publisher)
    publish_and_receive(publisher, subscriber)
    recreated_publisher_pid, recreated_subscriber_pid = publisher.pid, subscriber.pid
    subscriber.quit()
    publisher.quit()
    graceful_sub_cleaned = wait_gone(f"/dev/shm/agnocast@{recreated_subscriber_pid}")
    graceful_pub_cleaned = wait_gone(f"/dev/shm/agnocast@{recreated_publisher_pid}")
    deadline = time.monotonic() + 5.0
    while topic_mqueues(topic) and time.monotonic() < deadline:
        time.sleep(0.1)
    require(publisher_shm_cleaned, "SIGKILL publisher shm was not cleaned")
    require(subscriber_shm_cleaned, "SIGKILL subscriber shm was not cleaned")
    require(graceful_sub_cleaned and graceful_pub_cleaned, "graceful shm cleanup failed")
    require(not topic_mqueues(topic), "topic mqueue remained after cleanup")
    results["exit_recreate"] = {
        "first_publisher_pid": first_publisher_pid,
        "first_subscriber_pid": first_subscriber_pid,
        "recreated_publisher_pid": recreated_publisher_pid,
        "recreated_subscriber_pid": recreated_subscriber_pid,
        "publisher_sigkill_shm_cleaned": publisher_shm_cleaned,
        "subscriber_sigkill_shm_cleaned": subscriber_shm_cleaned,
        "subscriber_sigkill_held_lease": True,
        "graceful_shm_cleaned": graceful_sub_cleaned and graceful_pub_cleaned,
        "mqueue_cleaned": True,
    }


def run_daemon_registry_recovery(binary, environment, results):
    """Verify live-app recovery and singleton preservation across a quick restart."""
    topic = f"/phase2_probe_daemon_{uuid.uuid4().hex}"
    publisher = Probe(binary, "publisher", [topic, 1, 1024, 2000], environment)
    publisher.wait_event("READY")
    old_shm = Path(f"/dev/shm/agnocast@{publisher.pid}")
    require(old_shm.exists(), "daemon-loss publisher shm was not created")
    dead_daemon_pid = wait_for_only_child(publisher.pid)
    os.kill(dead_daemon_pid, signal.SIGKILL)
    wait_until_not_running(dead_daemon_pid)

    peer = Probe(binary, "publisher", [topic, 1, 1024, 2001], environment)
    peer.wait_event("READY")
    replacement_daemon_pid = wait_for_only_child(peer.pid)
    require(replacement_daemon_pid != dead_daemon_pid, "unlink daemon was not replaced")

    publisher.quit()
    stale_cleaned = wait_gone(old_shm)
    require(stale_cleaned, "replacement daemon did not clean the stale shm name")
    peer_shm = Path(f"/dev/shm/agnocast@{peer.pid}")
    peer.quit()

    quick = Probe(binary, "publisher", [topic, 1, 1024, 2002], environment)
    quick.wait_event("READY")
    quick_children = child_pids(quick.pid)
    require(len(quick_children) <= 1, f"quick restart spawned children {quick_children}")
    active_daemon_pid = quick_children[0] if quick_children else replacement_daemon_pid
    time.sleep(1.25)
    candidates = {replacement_daemon_pid, *quick_children}
    running_daemons = [pid for pid in candidates if process_is_running(pid)]
    require(
        running_daemons == [active_daemon_pid],
        f"daemon registry lost singleton: {candidates}",
    )

    peer_cleaned = wait_gone(peer_shm)
    quick_shm = Path(f"/dev/shm/agnocast@{quick.pid}")
    require(quick_shm.exists(), "quick-restart publisher shm was not created")
    quick.quit()
    quick_cleaned = wait_gone(quick_shm)
    wait_until_not_running(active_daemon_pid)
    require(peer_cleaned and quick_cleaned, "quick-restart shm cleanup failed")
    results["unlink_daemon_registry"] = {
        "killed_daemon_pid": dead_daemon_pid,
        "replacement_daemon_pid": replacement_daemon_pid,
        "replacement_started_while_live_app": True,
        "quick_restart_spawned_daemon": bool(quick_children),
        "quick_restart_singleton_count": len(running_daemons),
        "stale_shm_cleaned": stale_cleaned,
        "peer_shm_cleaned": peer_cleaned,
        "quick_restart_shm_cleaned": quick_cleaned,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--binary", required=True, type=Path)
    parser.add_argument("--heaphook", required=True, type=Path)
    parser.add_argument("--reserve-bytes", type=int, default=16 * 1024**3)
    arguments = parser.parse_args()
    require(arguments.binary.is_file(), f"probe binary not found: {arguments.binary}")
    require(arguments.heaphook.is_file(), f"heaphook not found: {arguments.heaphook}")

    environment = os.environ.copy()
    environment["AGNOCAST_BRIDGE_MODE"] = "off"
    environment["AGNOCAST_NO_DISCOVERY_AGENT"] = "1"
    prior_preload = environment.get("LD_PRELOAD", "")
    environment["LD_PRELOAD"] = str(arguments.heaphook) + (
        ":" + prior_preload if prior_preload else ""
    )
    results = {"binary": str(arguments.binary), "heaphook": str(arguments.heaphook)}
    try:
        # A fresh module makes the first process the unambiguous initial registry owner.
        run_daemon_registry_recovery(arguments.binary, environment, results)
        run_roundtrip(arguments.binary, environment, results)
        run_held_qos(arguments.binary, environment, results)
        run_exhaustion(arguments.binary, environment, arguments.reserve_bytes, results)
        run_exit_recreate(arguments.binary, environment, results)
    finally:
        Probe.cleanup_all()
    print("RESULT " + json.dumps(results, sort_keys=True))


if __name__ == "__main__":
    try:
        main()
    except Exception as error:  # noqa: BLE001 - CLI boundary reports all failures.
        Probe.cleanup_all()
        print(f"FAIL {error}", flush=True)
        raise
