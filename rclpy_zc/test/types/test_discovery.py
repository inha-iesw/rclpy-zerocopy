"""Reject incompatible providers before loading their native extension."""

import hashlib
import json
import sys
from types import SimpleNamespace

import pytest

from rclpy_agnocast.types import discovery


TYPE = 'example_msgs/msg/Sample'


@pytest.fixture
def provider(tmp_path, monkeypatch):
    """Provide independent installed-schema bytes and a spy for native loading."""
    share = tmp_path / 'share' / 'example_msgs'
    (share / 'msg').mkdir(parents=True)
    source = share / 'msg' / 'Sample.idl'
    source.write_text('module example_msgs { module msg { struct Sample { int32 x; }; }; };')
    schemas = {'example_msgs/msg/Sample.idl': hashlib.sha256(source.read_bytes()).hexdigest()}
    digest = hashlib.sha256(json.dumps(
        schemas, sort_keys=True, separators=(',', ':')).encode()).hexdigest()
    fields = [{'name': 'x', 'type': {'kind': 'scalar', 'type': 'int32'}}]
    manifest = {
        'manifest_version': 1, 'adapter_abi': 1, 'build_abi': 'test-build',
        'provider': 'example_adapters', 'module': 'example_adapters._types',
        'library': 'lib/python/example_adapters/_types.so',
        'types': {TYPE: {'schema_digest': digest, 'schemas': schemas, 'fields': fields}},
    }
    resources = {}
    loaded = []
    registered = []
    library = tmp_path / manifest['library']
    library.parent.mkdir(parents=True)
    library.touch()

    def write(data=manifest):
        name = data['provider']
        relative = f'share/{name}/adapters/manifest.json'
        path = tmp_path / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(data))
        resources[name] = relative
        return path

    def load(module, path):
        assert path == library
        loaded.append(module)
        return SimpleNamespace(_register=registered.append)

    monkeypatch.setattr(discovery, 'get_resources',
                        lambda _: dict.fromkeys(resources, str(tmp_path)))
    monkeypatch.setattr(discovery, 'get_resource',
                        lambda _, name: (resources[name], str(tmp_path)))
    monkeypatch.setattr(discovery, 'get_package_share_directory', lambda _: str(share))
    monkeypatch.setattr(discovery, '_load_plugin', load)
    monkeypatch.setattr(discovery, '_native', SimpleNamespace(
        _adapter_abi=lambda: {'version': 1, 'build': 'test-build'},
        _registered_type=lambda _: {
            'ros_type': TYPE, 'schema_digest': digest, 'fields_json': json.dumps(fields),
            'native_size': 4, 'native_alignment': 4}))
    path = write()
    return SimpleNamespace(manifest=manifest, write=write, source=source, path=path,
                           loaded=loaded, registered=registered, resources=resources,
                           prefix=tmp_path, library=library)


def test_valid_provider_registers_exact_manifest_and_returns_native_metadata(provider):
    """Bind the bytes validated before import to the native registration call."""
    result = discovery._resolve_type(TYPE)
    assert result['ros_type'] == TYPE
    assert result['native_size'] == 4
    assert result['fields'][0]['name'] == 'x'
    assert provider.loaded == ['example_adapters._types']
    assert provider.registered == [hashlib.sha256(provider.path.read_bytes()).hexdigest()]


@pytest.mark.parametrize('field,value,diagnostic', [
    ('manifest_version', 2, 'unsupported adapter manifest version'),
    ('adapter_abi', 2, 'adapter ABI mismatch'),
    ('build_abi', 'other-compiler', 'adapter ABI mismatch'),
    ('module', '../bad', 'invalid adapter module'),
])
def test_incompatible_manifest_is_rejected_before_import(provider, field, value, diagnostic):
    """Prevent execution when the provider's contract is incompatible."""
    provider.manifest[field] = value
    provider.write()
    with pytest.raises(ValueError, match=diagnostic):
        discovery._resolve_type(TYPE)
    assert provider.loaded == []


def test_changed_root_schema_is_rejected_before_import(provider):
    """Reject a stale root even when its Python class name has not changed."""
    provider.source.write_text(provider.source.read_text().replace('int32', 'int64'))
    with pytest.raises(ValueError, match='schema'):
        discovery._resolve_type(TYPE)
    assert provider.loaded == []


def test_changed_dependency_is_rejected_before_import(provider):
    """Treat nested schema changes as invalidating the provider."""
    dependency = provider.source.with_name('Child.idl')
    dependency.write_text('old dependency')
    entry = provider.manifest['types'][TYPE]
    entry['schemas']['example_msgs/msg/Child.idl'] = hashlib.sha256(
        dependency.read_bytes()).hexdigest()
    entry['schema_digest'] = hashlib.sha256(json.dumps(
        entry['schemas'], sort_keys=True, separators=(',', ':')).encode()).hexdigest()
    provider.write()
    dependency.write_text('new dependency')
    with pytest.raises(ValueError, match='schema'):
        discovery._resolve_type(TYPE)
    assert provider.loaded == []


def test_duplicate_provider_is_rejected_before_either_import(provider):
    """Keep provider selection independent of discovery ordering."""
    duplicate = dict(provider.manifest, provider='other_adapters', module='other_adapters._types')
    provider.write(duplicate)
    with pytest.raises(ValueError, match='provider'):
        discovery._resolve_type(TYPE)
    assert provider.loaded == []


def test_registry_miss_names_type_and_build_action(provider):
    """Explain how to supply the adapter without constructing a fallback."""
    with pytest.raises(LookupError, match='missing_msgs/msg/Missing.*build'):
        discovery._resolve_type('missing_msgs/msg/Missing')
    assert provider.loaded == []


@pytest.mark.parametrize('logical', ['../Sample.idl', 'example_msgs/../../Sample.idl'])
def test_schema_path_escape_is_rejected_before_import(provider, logical):
    """Resolve only package-relative ROSIDL paths from schema manifests."""
    entry = provider.manifest['types'][TYPE]
    entry['schemas'][logical] = '0' * 64
    entry['schema_digest'] = hashlib.sha256(json.dumps(
        entry['schemas'], sort_keys=True, separators=(',', ':')).encode()).hexdigest()
    provider.write()
    with pytest.raises(ValueError, match='invalid adapter resource path'):
        discovery._resolve_type(TYPE)
    assert provider.loaded == []


def test_manifest_resource_cannot_escape_selected_prefix(provider):
    """Reject resource paths outside the ament-selected installation."""
    provider.resources['example_adapters'] = '../manifest.json'
    with pytest.raises(ValueError):
        discovery._resolve_type(TYPE)
    assert provider.loaded == []


def test_manifest_digest_is_validated_independently_of_file_hashes(provider):
    """Check aggregate schema identity as well as each source file."""
    provider.manifest['types'][TYPE]['schema_digest'] = '0' * 64
    provider.write()
    with pytest.raises(ValueError, match='digest'):
        discovery._resolve_type(TYPE)
    assert provider.loaded == []


def test_provider_identity_must_match_resource_name(provider):
    """Prevent a resource from claiming another provider's registration."""
    provider.manifest['provider'] = 'wrong_provider'
    provider.path.write_text(json.dumps(provider.manifest))
    with pytest.raises(ValueError, match='provider'):
        discovery._resolve_type(TYPE)
    assert provider.loaded == []


def test_duplicate_json_keys_are_rejected_before_import(provider):
    """Avoid interpreting ambiguous metadata differently across readers."""
    data = provider.path.read_text().replace('"manifest_version": 1',
                                             '"manifest_version": 2, "manifest_version": 1')
    provider.path.write_text(data)
    with pytest.raises(ValueError, match='duplicate'):
        discovery._resolve_type(TYPE)
    assert provider.loaded == []


def test_public_lookup_accepts_ros_message_class_only(provider):
    """Use the class as identity without accepting an owning message object."""
    message_class = type('Sample', (), {'__module__': 'example_msgs.msg._sample'})
    assert discovery.resolve_adapter(message_class)['ros_type'] == TYPE
    with pytest.raises(TypeError):
        discovery.resolve_adapter(message_class())
    with pytest.raises(TypeError):
        discovery.resolve_adapter(str)


def test_preloaded_module_from_another_prefix_is_rejected_without_execution(tmp_path, monkeypatch):
    """Keep a previously imported overlay from replacing the selected binary."""
    module = 'conflicting_adapters._types'
    selected = tmp_path / 'selected' / '_types.so'
    selected.parent.mkdir()
    selected.touch()
    monkeypatch.setitem(sys.modules, module, SimpleNamespace(__file__='/other/prefix/_types.so'))
    with pytest.raises(ValueError, match='origin'):
        discovery._load_plugin(module, selected)


def test_library_path_must_be_inside_provider_prefix(provider):
    """Reject a binary path that leaves the declared installation prefix."""
    relative = f'../{provider.prefix.name}_outside/_types.so'
    outside = provider.prefix / relative
    outside.parent.mkdir()
    outside.touch()
    provider.manifest['library'] = relative
    provider.write()
    with pytest.raises(ValueError, match='invalid adapter resource path'):
        discovery._resolve_type(TYPE)
    assert provider.loaded == []


def test_python_module_cannot_replace_native_extension(provider):
    """Require the installed artifact to be a native extension."""
    provider.manifest['library'] = 'lib/python/example_adapters/_types.py'
    provider.library.with_suffix('.py').touch()
    provider.write()
    with pytest.raises(ValueError, match='invalid adapter library'):
        discovery._resolve_type(TYPE)
    assert provider.loaded == []
