"""Exact-artifact opt-in and peer lifetime for the CPU checkpoint."""

import pytest

from checkpoint.harness import PAYLOAD_LIMIT, Peers
from system_runtime import preflight, system_tld_requested


@pytest.fixture(scope="module")
def config():
    if not system_tld_requested():
        pytest.skip("set RCLPY_AGNOCAST_RUN_SYSTEM_TLD=1 for the two-process checkpoint")
    return preflight()


@pytest.fixture
def peers(config, request):
    count, limit = getattr(request, "param", (256, PAYLOAD_LIMIT))
    peers = Peers(config, count, limit)
    try:
        yield peers
    finally:
        peers.close()
