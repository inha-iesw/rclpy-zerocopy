"""Exercise real installed plugins against the shared native registry."""

import json

import pytest

from rclpy_agnocast import _native
from rclpy_agnocast.types import resolve_adapter
from sensor_msgs.msg import Image, PointCloud2


@pytest.mark.parametrize('message_type', [Image, PointCloud2])
def test_generated_provider_registers_in_runtime_shared_registry(message_type):
    descriptor = resolve_adapter(message_type)
    registered = _native._registered_type(descriptor['ros_type'])
    assert json.loads(registered['fields_json']) == descriptor['fields']
    assert descriptor['native_size'] > 0
    assert descriptor['native_alignment'] > 0
    assert resolve_adapter(message_type) == descriptor


def test_compiled_provider_rejects_manifest_different_from_generated_bytes():
    resolve_adapter(Image)
    from rclpy_agnocast_adapters import _types
    with pytest.raises(ValueError, match='manifest'):
        _types._register('0' * 64)
