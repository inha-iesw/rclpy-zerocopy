"""Confine deliberate Tensor writes to an owned fork child."""

import os
import signal
import time


def assert_write_fault(alias, index=0):
    """Require a real write to fault, with bounded waiting and child reaping."""
    import faulthandler
    import resource

    pid = os.fork()
    if pid == 0:
        try:
            faulthandler.disable()
            resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
            alias[index] = 99
        except BaseException:
            os._exit(2)
        os._exit(0)
    status = None
    deadline = time.monotonic() + 5
    try:
        while status is None:
            child, result = os.waitpid(pid, os.WNOHANG)
            if child:
                status = result
                break
            assert time.monotonic() < deadline, 'isolated in-place write did not finish'
            time.sleep(0.01)
    finally:
        if status is None:
            os.kill(pid, signal.SIGKILL)
            os.waitpid(pid, 0)
    assert os.WIFSIGNALED(status), f'in-place write child exited with status {status}'
    signum = os.WTERMSIG(status)
    assert signum in (signal.SIGSEGV, signal.SIGBUS)
    return signum
