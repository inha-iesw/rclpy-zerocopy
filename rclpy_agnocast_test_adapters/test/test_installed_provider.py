"""Exercise fixture providers and their isolation from standard adapters."""

import json
from pathlib import Path
import shutil

from ament_index_python.packages import PackageNotFoundError
import pytest

from rclpy_agnocast import _native
from rclpy_agnocast.types import discovery, resolve_adapter
from rclpy_agnocast_test_msgs.msg import Containers, Nested, Scalars
from sensor_msgs.msg import Image, PointCloud2


@pytest.mark.parametrize('message_type', [Scalars, Containers, Nested])
def test_generated_provider_registers_in_runtime_shared_registry(message_type):
    """Register all fixture types in the same runtime registry used by the facade."""
    descriptor = resolve_adapter(message_type)
    registered = _native._registered_type(descriptor['ros_type'])
    assert json.loads(registered['fields_json']) == descriptor['fields']
    assert descriptor['native_size'] > 0
    assert descriptor['native_alignment'] > 0
    assert resolve_adapter(message_type) == descriptor


def test_nested_fixture_preserves_field_shapes_and_dependency_tree():
    """Retain nested sequence, array, boolean and bounded string descriptors."""
    descriptor = resolve_adapter(Nested)
    fields = {field['name']: field['type'] for field in descriptor['fields']}
    assert fields['children']['kind'] == 'sequence'
    assert fields['children']['element']['type'] == 'rclpy_agnocast_test_msgs/msg/Scalars'
    assert fields['fixed_children']['length'] == 2
    containers = {field['name']: field['type'] for field in fields['containers']['fields']}
    assert containers['fixed_flags']['element']['type'] == 'boolean'
    assert containers['bounded_flags']['bound'] == 8
    assert containers['bounded_wide_texts']['element'] == {'kind': 'wstring', 'bound': 16}


@pytest.mark.parametrize('fixture_state', ['missing', 'stale'])
def test_invalid_fixture_schema_does_not_block_standard_types(
        tmp_path, monkeypatch, fixture_state):
    """Keep standard lookup usable while failing closed on an invalid fixture provider."""
    package = 'rclpy_agnocast_test_msgs'
    get_share = discovery.get_package_share_directory
    share = tmp_path / package
    if fixture_state == 'stale':
        shutil.copytree(Path(get_share(package)) / 'msg', share / 'msg')
        schema = share / 'msg' / 'Scalars.idl'
        schema.write_text(schema.read_text() + '\n// changed fixture schema\n')

    def selected_share(name):
        if name == package:
            if fixture_state == 'missing':
                raise PackageNotFoundError(package)
            return str(share)
        return get_share(name)

    monkeypatch.setattr(discovery, 'get_package_share_directory', selected_share)
    for message_type in (Image, PointCloud2):
        expected = f'sensor_msgs/msg/{message_type.__name__}'
        assert resolve_adapter(message_type)['ros_type'] == expected
    error = PackageNotFoundError if fixture_state == 'missing' else ValueError
    diagnostic = package if fixture_state == 'missing' else 'adapter schema mismatch'
    with pytest.raises(error, match=diagnostic):
        resolve_adapter(Scalars)
