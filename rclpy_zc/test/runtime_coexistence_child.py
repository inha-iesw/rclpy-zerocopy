#!/usr/bin/env python3
"""Fresh-process scenarios for rclpy and Agnocast global-state coexistence."""

from __future__ import annotations

import argparse
import json
import os
import time
import traceback
from typing import Any, Callable, Set


EVENT_PREFIX = "TLD_EVENT "
SKIP_EXIT = 77
PREFLIGHT_MESSAGES = (
    "LD_PRELOAD must contain",
    "configured Agnocast heaphook",
    "cannot inspect loaded Agnocast artifacts",
    "cannot identify the active malloc provider",
    "heaphook has no sibling fork libagnocast.so",
    "_native did not load the heaphook's sibling libagnocast.so",
    "does not provide the active malloc",
    "is missing or not readable/writable",
)


class BackendUnavailable(RuntimeError):
    """Indicate that this host cannot execute the opt-in system scenario."""


def emit(name: str, **fields: Any) -> None:
    """Write one unambiguous event for the parent pytest process."""
    print(EVENT_PREFIX + json.dumps({"name": name, **fields}, sort_keys=True), flush=True)


def backend_import() -> Any:
    """Import the adapter while distinguishing loader unavailability from test failure."""
    try:
        from rclpy_agnocast import AgnocastRuntime

        return AgnocastRuntime
    except (ImportError, OSError) as error:
        message = str(error)
        loader_failure = (
            "libagnocast" in message
            or "cannot open shared object file" in message
            or "undefined symbol" in message
        )
        if loader_failure:
            raise BackendUnavailable(message) from error
        raise


def create_runtime(runtime_type: Any, node: Any) -> Any:
    """Create the runtime, converting only known environment preflight failures to skip."""
    try:
        return runtime_type(node)
    except RuntimeError as error:
        if any(fragment in str(error) for fragment in PREFLIGHT_MESSAGES):
            raise BackendUnavailable(str(error)) from error
        raise


def wait_until(predicate: Callable[[], bool], timeout: float, spin: Any = None) -> None:
    """Wait for a bounded observable state transition."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        if spin is None:
            time.sleep(0.02)
        else:
            spin.spin_once(timeout_sec=0.05)
    if not predicate():
        raise RuntimeError("timed out waiting for subprocess observation")


class Scenario:
    """Own one rclpy context and all resources created by a child scenario."""

    def __init__(self) -> None:
        import rclpy
        from rclpy.context import Context
        from rclpy.signals import SignalHandlerOptions

        self.rclpy = rclpy
        self.context = Context()
        rclpy.init(context=self.context, signal_handler_options=SignalHandlerOptions.ALL)
        self.node = rclpy.create_node(
            f"rclpy_agnocast_tld_{os.getpid()}", context=self.context
        )
        self.listener = None
        self.executor = None
        self.runtime = None

    def start_runtime(self) -> None:
        """Construct the adapter only after rclpy and its signal handlers are active."""
        self.runtime = create_runtime(backend_import(), self.node)

    def close_runtime(self) -> None:
        """Close the adapter and require the rclpy context to remain usable."""
        from rclpy_agnocast import _native

        if self.runtime is None:
            return
        self.runtime.close()
        if self.runtime.state != _native.RuntimeState.CLOSED:
            raise RuntimeError("adapter did not reach CLOSED")
        self.runtime = None
        if not self.context.ok():
            raise RuntimeError("adapter close shut down the rclpy context")

    def prepare_rosout(self) -> Set[str]:
        """Create an in-process /rosout observer for logging continuity checks."""
        from rcl_interfaces.msg import Log
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

        # Humble rclpy does not export rcl's rosout QoS; mirror rmw_qos_profile_rosout_default.
        rosout_qos = QoSProfile(
            depth=1000,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        observed: Set[str] = set()
        self.listener = self.rclpy.create_node(
            f"rclpy_agnocast_tld_listener_{os.getpid()}",
            context=self.context,
            enable_rosout=False,
        )
        self.listener.create_subscription(
            Log, "/rosout", lambda message: observed.add(message.msg), rosout_qos
        )
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.listener)
        wait_until(lambda: self.listener.count_publishers("/rosout") >= 1, 8.0, self.executor)
        return observed

    def log_and_observe(self, marker: str, observed: Set[str]) -> None:
        """Require one node log to traverse the live /rosout path."""
        self.node.get_logger().info(marker)
        wait_until(lambda: marker in observed, 8.0, self.executor)

    def shutdown_normally(self) -> None:
        """Finish the supported lifecycle without suppressing logging teardown errors."""
        if self.executor is not None:
            if not self.executor.shutdown(timeout_sec=2.0):
                raise RuntimeError("rclpy executor did not shut down")
            self.executor = None
        for attribute in ("listener", "node"):
            node = getattr(self, attribute)
            if node is not None:
                node.destroy_node()
                setattr(self, attribute, None)
        self.rclpy.shutdown(context=self.context, uninstall_handlers=True)
        if self.context.ok():
            raise RuntimeError("rclpy shutdown did not invalidate its context")

    def finish_after_signal(self) -> None:
        """Remove rclpy resources after its signal handler shut down the context."""
        for attribute in ("listener", "node"):
            node = getattr(self, attribute)
            if node is not None:
                node.destroy_node()
                setattr(self, attribute, None)
        self.rclpy.uninstall_signal_handlers()

    def cleanup(self) -> None:
        """Best-effort cleanup without converting a scenario failure into a false success."""
        if self.runtime is not None:
            try:
                self.runtime.close()
            except BaseException:
                pass
            self.runtime = None
        if self.executor is not None:
            try:
                self.executor.shutdown(timeout_sec=1.0)
            except BaseException:
                pass
        for node in (self.listener, self.node):
            if node is not None:
                try:
                    node.destroy_node()
                except BaseException:
                    pass
        try:
            if self.context.ok():
                self.rclpy.shutdown(context=self.context, uninstall_handlers=True)
            else:
                self.rclpy.uninstall_signal_handlers()
        except BaseException:
            pass


def normal_logging() -> None:
    """Observe rclpy logging before, during, and after one adapter lifetime."""
    scenario = Scenario()
    try:
        observed = scenario.prepare_rosout()
        pid = os.getpid()
        markers = [
            f"RCLPY_AGNOCAST_TLD_BEFORE_{pid}",
            f"RCLPY_AGNOCAST_TLD_DURING_{pid}",
            f"RCLPY_AGNOCAST_TLD_AFTER_{pid}",
        ]
        emit("ready", pid=pid, phase="normal_logging")
        scenario.log_and_observe(markers[0], observed)
        scenario.start_runtime()
        scenario.log_and_observe(markers[1], observed)
        scenario.close_runtime()
        scenario.log_and_observe(markers[2], observed)
        scenario.shutdown_normally()
        emit("result", pid=pid, rosout_markers=3, rclpy_shutdown=True)
    finally:
        scenario.cleanup()


def wait_for_signal_shutdown(context: Any) -> bool:
    """Wait for rclpy's signal-driven shutdown and report whether it interrupted."""
    # Humble rclpy also forwards SIGINT to Python's default handler, so the main
    # thread first raises KeyboardInterrupt and the context shuts down shortly
    # after. SIGTERM only shuts the context down.
    try:
        wait_until(lambda: not context.ok(), 8.0)
        return False
    except KeyboardInterrupt:
        wait_until(lambda: not context.ok(), 8.0)
        return True


def signal_scenario(close_before_ready: bool) -> None:
    """Observe rclpy signal behavior with the adapter open or already closed."""
    scenario = Scenario()
    try:
        scenario.start_runtime()
        phase = "after_close" if close_before_ready else "while_open"
        if close_before_ready:
            scenario.close_runtime()
        emit("ready", pid=os.getpid(), phase=phase)
        interrupted = wait_for_signal_shutdown(scenario.context)
        if not close_before_ready:
            scenario.runtime.close()
            scenario.runtime = None
        scenario.finish_after_signal()
        emit(
            "result",
            pid=os.getpid(),
            phase=phase,
            rclpy_signal_observed=True,
            agnocast_state_observed=False,
            keyboard_interrupt=interrupted,
        )
    finally:
        scenario.cleanup()


def main() -> int:
    """Dispatch one scenario and encode backend unavailability as an explicit skip."""
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "scenario", choices=("normal_logging", "signal_while_open", "signal_after_close")
    )
    arguments = parser.parse_args()
    try:
        if arguments.scenario == "normal_logging":
            normal_logging()
        else:
            signal_scenario(arguments.scenario == "signal_after_close")
        return 0
    except BackendUnavailable as error:
        emit("skip", reason=str(error))
        return SKIP_EXIT


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except SystemExit:
        raise
    except BaseException as error:
        emit("error", error=repr(error), traceback=traceback.format_exc())
        raise SystemExit(1)
