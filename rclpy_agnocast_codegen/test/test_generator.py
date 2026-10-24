"""Contract checks for build-time ROSIDL schemas and provider generation."""

import hashlib
import json

import pytest

from rclpy_agnocast_codegen.generate import generate
from rclpy_agnocast_codegen.schema import describe_type


def write_idl(share, name, members, includes=''):
    """Write an independently declared message fixture into a package share."""
    path = share / 'msg' / (name + '.idl')
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        includes + '\nmodule sample { module msg { struct ' + name + ' {\n' +
        members + '\n}; }; };\n', encoding='utf-8')
    return path


@pytest.fixture
def shares(tmp_path):
    """Build scalar, bounded, wide-string and nested schema inputs."""
    share = tmp_path / 'share' / 'sample'
    write_idl(share, 'Leaf', 'int32 value;')
    write_idl(
        share, 'Tree',
        'boolean enabled;\nfloat weight;\nint16 fixed[3];\n'
        'sequence<uint8, 16> bytes;\nsequence<int64> values;\n'
        'string<8> label;\nstring text;\nwstring<4> wide;\nwstring title;\n'
        'sample::msg::Leaf child;\nsequence<sample::msg::Leaf, 2> children;',
        '#include "sample/msg/Leaf.idl"')
    return {'sample': share}


def test_complete_field_metadata(shares):
    """Preserve every field's shape and hash the exact dependency contents."""
    result = describe_type('sample/msg/Tree', shares)
    fields = {field['name']: field['type'] for field in result['fields']}
    assert fields == {
        'enabled': {'kind': 'scalar', 'type': 'boolean'},
        'weight': {'kind': 'scalar', 'type': 'float'},
        'fixed': {
            'kind': 'array', 'length': 3,
            'element': {'kind': 'scalar', 'type': 'int16'}},
        'bytes': {
            'kind': 'sequence', 'bound': 16,
            'element': {'kind': 'scalar', 'type': 'uint8'}},
        'values': {
            'kind': 'sequence', 'bound': None,
            'element': {'kind': 'scalar', 'type': 'int64'}},
        'label': {'kind': 'string', 'bound': 8},
        'text': {'kind': 'string', 'bound': None},
        'wide': {'kind': 'wstring', 'bound': 4},
        'title': {'kind': 'wstring', 'bound': None},
        'child': {
            'kind': 'message', 'type': 'sample/msg/Leaf',
            'fields': [{'name': 'value', 'type': {'kind': 'scalar', 'type': 'int32'}}]},
        'children': {
            'kind': 'sequence', 'bound': 2,
            'element': {
                'kind': 'message', 'type': 'sample/msg/Leaf',
                'fields': [{'name': 'value', 'type': {
                    'kind': 'scalar', 'type': 'int32'}}]}},
    }
    expected_hashes = {
        'sample/msg/' + path.name: hashlib.sha256(path.read_bytes()).hexdigest()
        for path in shares['sample'].glob('msg/*.idl')}
    assert result['schemas'] == expected_hashes
    canonical = json.dumps(expected_hashes, sort_keys=True, separators=(',', ':'))
    assert result['schema_digest'] == hashlib.sha256(canonical.encode()).hexdigest()


def test_transitive_idl_change_changes_parent_digest(shares):
    """Invalidate a parent schema when only a referenced message changes."""
    before = describe_type('sample/msg/Tree', shares)
    write_idl(shares['sample'], 'Leaf', 'int64 value;')
    after = describe_type('sample/msg/Tree', shares)
    assert before['schema_digest'] != after['schema_digest']
    assert before['schemas']['sample/msg/Tree.idl'] == after['schemas']['sample/msg/Tree.idl']


def test_include_dependency_is_hashed_even_without_field(shares):
    """Include the full compiler dependency closure in schema identity."""
    write_idl(shares['sample'], 'Unused', 'uint32 marker;')
    path = shares['sample'] / 'msg/Leaf.idl'
    path.write_text('#include "sample/msg/Unused.idl"\n' + path.read_text())
    assert set(describe_type('sample/msg/Leaf', shares)['schemas']) == {
        'sample/msg/Leaf.idl', 'sample/msg/Unused.idl'}


@pytest.mark.parametrize('name', ['sample/srv/Tree', 'sample/action/Tree', '../msg/Tree'])
def test_unsupported_type_path_is_rejected(shares, name):
    """Reject services, actions and paths outside the message naming contract."""
    with pytest.raises(ValueError, match='pkg/msg/Name'):
        describe_type(name, shares)


def test_wrong_named_structure_is_rejected(shares):
    """Reject an IDL file whose declared type does not match its requested name."""
    path = shares['sample'] / 'msg/Leaf.idl'
    path.write_text(path.read_text().replace('struct Leaf', 'struct Other'))
    with pytest.raises(ValueError, match='sample/msg/Leaf'):
        describe_type('sample/msg/Leaf', shares)


def test_unknown_dependency_package_is_rejected(shares):
    """Require an explicit location for each dependency package."""
    write_idl(shares['sample'], 'Broken', 'int32 value;', '#include "other/msg/X.idl"')
    with pytest.raises(ValueError, match='other'):
        describe_type('sample/msg/Broken', shares)


def test_include_outside_message_schema_is_rejected(shares):
    """Reject arbitrary include paths before attempting to load them."""
    write_idl(shares['sample'], 'Broken', 'int32 value;', '#include "../outside.idl"')
    with pytest.raises(ValueError, match='include'):
        describe_type('sample/msg/Broken', shares)


def test_recursive_schema_is_rejected(shares):
    """Reject recursive fields instead of producing an incomplete descriptor."""
    write_idl(shares['sample'], 'Cycle', 'sequence<sample::msg::Cycle> values;')
    with pytest.raises(ValueError, match='[Rr]ecursive|[Cc]ycle'):
        describe_type('sample/msg/Cycle', shares)


def test_generation_is_deterministic_and_confined_to_output(shares, tmp_path):
    """Keep generated bytes independent of output roots and type request order."""
    source_before = {path: path.read_bytes() for path in shares['sample'].rglob('*')
                     if path.is_file()}
    for directory, types in [
            ('first', ['sample/msg/Tree', 'sample/msg/Leaf']),
            ('second', ['sample/msg/Leaf', 'sample/msg/Tree'])]:
        generate('sample_provider', 'sample_provider._adapter', types, shares,
                 'test-abi', tmp_path / directory, 'lib/python/sample_provider/_adapter.so')
    first = tmp_path / 'first'
    second = tmp_path / 'second'
    assert sorted(path.name for path in first.iterdir()) == ['manifest.json', 'provider.cpp']
    for name in ('manifest.json', 'provider.cpp'):
        assert (first / name).read_bytes() == (second / name).read_bytes()
        assert str(tmp_path) not in (first / name).read_text()
    assert source_before == {path: path.read_bytes() for path in source_before}
    manifest = json.loads((first / 'manifest.json').read_text())
    assert manifest == {
        'manifest_version': 1, 'adapter_abi': 3, 'build_abi': 'test-abi',
        'provider': 'sample_provider', 'module': 'sample_provider._adapter',
        'library': 'lib/python/sample_provider/_adapter.so',
        'types': {name: describe_type(name, shares) for name in
                  ['sample/msg/Leaf', 'sample/msg/Tree']}}
    source = (first / 'provider.cpp').read_text()
    assert 'PYBIND11_MODULE(_adapter, module)' in source
    assert '#include "sample/msg/detail/tree__struct.hpp"' in source
    assert '#include "sample/msg/detail/tree__traits.hpp"' in source
    assert 'sizeof(sample::msg::Tree)' in source
    assert '&create_subscription_endpoint<sample::msg::Tree>' in source
    assert 'alignof(sample::msg::Tree)' in source
    assert hashlib.sha256((first / 'manifest.json').read_bytes()).hexdigest() in source
    assert 'register_provider(provider, expected_manifest_digest)' in source
    assert 'compiled_abi_tag()' in source
    assert '::rclpy_agnocast_adapter_abi_version() != ADAPTER_ABI_VERSION' in source
    assert 'std::string(::rclpy_agnocast_adapter_build_abi()) != compiled_abi_tag()' in source
    assert 'compiled_abi_tag() != "test-abi"' in source
    assert 'throw pybind11::import_error("adapter ABI mismatch")' in source
    assert source.index('adapter ABI mismatch') < source.index('module.def(')
    assert 'decltype(std::declval<sample::msg::Leaf>().value), int32_t' in source
    assert 'decltype(std::declval<sample::msg::Tree>().enabled), bool' in source
    assert 'decltype(std::declval<sample::msg::Tree>().fixed), std::array<int16_t, 3>' in source
    assert 'rosidl_runtime_cpp::BoundedVector<uint8_t, 16,' in source
    assert 'sample::msg::Leaf_<ContainerAllocator>' in source


@pytest.mark.parametrize('module', ['foo.invalid-name', 'foo.', 'a;execute'])
def test_invalid_module_identifier_is_rejected(shares, tmp_path, module):
    """Validate native module identifiers before writing generated files."""
    with pytest.raises(ValueError, match='module'):
        generate('sample_provider', module, ['sample/msg/Leaf'], shares,
                 'test-abi', tmp_path / 'out', 'lib/python/sample_provider/_adapter.so')
    assert not (tmp_path / 'out').exists()


@pytest.mark.parametrize('library', [
    '/tmp/provider.so', '../provider.so', 'lib/../provider.so', ''])
def test_library_path_must_stay_inside_provider_prefix(shares, tmp_path, library):
    """Reject absolute paths and parent traversal before writing the manifest."""
    with pytest.raises(ValueError, match='library'):
        generate('sample_provider', 'sample_provider._adapter', ['sample/msg/Leaf'], shares,
                 'test-abi', tmp_path / 'out', library)
    assert not (tmp_path / 'out').exists()


def test_nested_fields_are_checked_without_separate_type_registration(shares, tmp_path):
    """Check nested message members even when only their parent is registered."""
    generate('sample_provider', 'sample_provider._adapter', ['sample/msg/Tree'], shares,
             'test-abi', tmp_path / 'out', 'lib/python/sample_provider/_adapter.so')
    source = (tmp_path / 'out' / 'provider.cpp').read_text()
    assert 'decltype(std::declval<sample::msg::Leaf>().value), int32_t' in source
    assert source.count('decltype(std::declval<sample::msg::Leaf>().value)') == 1


def test_generated_operations_cover_nested_fields_before_their_parent(shares, tmp_path):
    """Generate executable members for the closure and a factory for declared roots only."""
    write_idl(shares['sample'], 'Branch', 'sample::msg::Tree tree;',
              '#include "sample/msg/Tree.idl"')
    generate('sample_provider', 'sample_provider._adapter', ['sample/msg/Branch'], shares,
             'test-abi', tmp_path / 'out', 'lib/python/sample_provider/_adapter.so')
    source = (tmp_path / 'out' / 'provider.cpp').read_text()
    assert 'scalar_ops<int32_t>()' in source
    assert 'array_ops<std::array<int16_t, 3>>' in source
    assert 'sequence_ops<rosidl_runtime_cpp::BoundedVector<uint8_t, 16,' in source
    assert 'string_ops<std::basic_string<char,' in source
    assert 'string_ops<std::basic_string<char16_t,' in source
    assert 'static_cast<sample::msg::Tree *>(object)->children' in source
    assert source.index('message_ops<sample::msg::Leaf>') < source.index(
        'message_ops<sample::msg::Tree>')
    assert source.index('message_ops<sample::msg::Tree>') < source.index(
        'message_ops<sample::msg::Branch>')
    assert '&create_endpoint<sample::msg::Branch>' in source
    assert '&create_endpoint<sample::msg::Tree>' not in source
    assert '&create_endpoint<sample::msg::Leaf>' not in source
