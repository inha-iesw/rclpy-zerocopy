#!/usr/bin/env python3
"""Run one interactive real-backend rclpy Agnocast publisher child."""

import argparse
import ctypes
import os
import shlex
import sys
from urllib.parse import quote


def _value_text(value):
    if isinstance(value, bool):
        return str(value).lower()
    return quote(str(value), safe="._-/:+")


def emit(name, **fields):
    """Write one atomic line-oriented event for the parent harness."""
    tokens = [f"name={name}"]
    tokens.extend(f"{key}={_value_text(value)}" for key, value in fields.items())
    print("EVENT " + " ".join(tokens), flush=True)


def _expect(arguments, count, command):
    if len(arguments) != count:
        raise ValueError(f"{command} expects {count} arguments")


class PublisherSession:
    """Own the public rclpy adapter objects used by interactive commands."""

    def __init__(self, arguments):
        import rclpy
        from rclpy_agnocast import AgnocastRuntime

        self._rclpy = rclpy
        self._topic = arguments.topic
        self._depth = arguments.depth
        self._limit = arguments.max_payload_bytes
        rclpy.init(args=[])
        self._node = rclpy.create_node(f"rclpy_agnocast_system_{os.getpid()}")
        self._runtime = AgnocastRuntime(self._node)
        self._publisher = self._create_publisher()

    def _create_publisher(self):
        return self._runtime.create_publisher(
            self._topic,
            depth=self._depth,
            max_payload_bytes=self._limit,
        )

    @staticmethod
    def _fill(loan, size, sequence):
        loan.sequence = sequence
        view = loan.buffer
        try:
            address = (
                ctypes.addressof(ctypes.c_uint8.from_buffer(view)) if size else 0
            )
            checksum = 0
            for index in range(size):
                value = (sequence + index) & 0xFF
                view[index] = value
                checksum += value
            details = {
                "bytes": view.nbytes,
                "checksum": checksum,
                "data_addr": hex(address),
                "format": view.format,
                "itemsize": view.itemsize,
                "ndim": view.ndim,
                "stride": view.strides[0],
                "readonly": view.readonly,
                "contiguous": view.c_contiguous,
            }
        finally:
            view.release()
        return details

    @staticmethod
    def _invalidated(loan):
        try:
            _ = loan.sequence
        except RuntimeError:
            return True
        return False

    def publish(self, arguments):
        _expect(arguments, 2, "publish")
        size, sequence = map(int, arguments)
        with self._publisher.borrow(size) as loan:
            details = self._fill(loan, size, sequence)
            loan.publish()
        details["invalidated"] = self._invalidated(loan)
        emit("PUBLISHED", sequence=sequence, **details)

    def cancel(self, arguments):
        _expect(arguments, 2, "cancel")
        size, sequence = map(int, arguments)
        with self._publisher.borrow(size) as loan:
            details = self._fill(loan, size, sequence)
            loan.cancel()
        emit("CANCELLED", sequence=sequence, **details)

    def reject(self, arguments):
        _expect(arguments, 1, "reject")
        size = int(arguments[0])
        try:
            loan = self._publisher.borrow(size)
        except Exception as error:  # Expected validation boundary.
            emit("REJECTED", size=size, error=type(error).__name__)
            return
        loan.cancel()
        raise AssertionError(f"borrow unexpectedly accepted {size}")

    def alias_barrier(self, arguments):
        _expect(arguments, 2, "alias-barrier")
        size, sequence = map(int, arguments)
        loan = self._publisher.borrow(size)
        loan.sequence = sequence
        root = loan.buffer
        alias = root[1:]
        root.release()
        try:
            try:
                loan.publish()
            except Exception as error:  # BufferError is reported to the parent.
                blocked = type(error).__name__
            else:
                raise AssertionError("publish accepted a live writable alias")
        finally:
            alias.release()
        details = self._fill(loan, size, sequence)
        loan.publish()
        details["invalidated"] = self._invalidated(loan)
        emit("ALIAS_BARRIER", sequence=sequence, blocked=blocked, **details)

    def context_error(self, arguments):
        _expect(arguments, 1, "context-error")
        size = int(arguments[0])
        try:
            with self._publisher.borrow(size) as loan:
                self._fill(loan, size, 700)
                raise LookupError("context-sentinel")
        except LookupError as error:
            emit("CONTEXT_ERROR", error=type(error).__name__, original=str(error))

    def recreate(self, arguments):
        _expect(arguments, 0, "recreate")
        self._publisher.close()
        self._publisher = self._create_publisher()
        emit("RECREATED", topic=self._topic)

    def close_active(self, arguments):
        _expect(arguments, 1, "close-active")
        size = int(arguments[0])
        loan = self._publisher.borrow(size)
        view = loan.buffer
        try:
            try:
                self._publisher.close()
            except Exception as error:
                blocked = type(error).__name__
            else:
                raise AssertionError("publisher close accepted an active Loan")
        finally:
            view.release()
        loan.cancel()
        self._publisher.close()
        self._publisher = self._create_publisher()
        emit("ACTIVE_CLOSE_RECOVERED", blocked=blocked)

    def run(self):
        """Serve commands until the parent requests orderly shutdown."""
        emit("READY", pid=os.getpid(), max_payload_bytes=self._limit)
        handlers = {
            "publish": self.publish,
            "cancel": self.cancel,
            "reject": self.reject,
            "alias-barrier": self.alias_barrier,
            "context-error": self.context_error,
            "recreate": self.recreate,
            "close-active": self.close_active,
        }
        for raw_line in sys.stdin:
            fields = shlex.split(raw_line)
            if not fields:
                continue
            if fields[0] == "quit":
                return
            try:
                handlers[fields[0]](fields[1:])
            except Exception as error:
                emit(
                    "ERROR",
                    operation=fields[0],
                    error=type(error).__name__,
                    message=str(error),
                )

    def close(self):
        """Close in the supported adapter-before-rclpy order."""
        try:
            self._publisher.close()
            self._runtime.close()
        finally:
            self._node.destroy_node()
            self._rclpy.shutdown()


def _arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", required=True)
    parser.add_argument("--depth", type=int, required=True)
    parser.add_argument("--max-payload-bytes", type=int, required=True)
    return parser.parse_args()


def main():
    """Create a session only after this preloaded child has started."""
    session = PublisherSession(_arguments())
    try:
        session.run()
    finally:
        session.close()
    emit("CLOSED")


if __name__ == "__main__":
    try:
        main()
    except Exception as error:  # CLI boundary must retain startup failures.
        emit("ERROR", operation="startup", error=type(error).__name__, message=str(error))
        raise
