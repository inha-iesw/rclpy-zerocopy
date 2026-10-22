"""Compile generated field assertions; registration uses a test-only ABI stub."""

import os
from pathlib import Path
import shlex
import subprocess
import sysconfig

import pybind11
import pytest

from rclpy_agnocast_codegen.generate import generate


_ADAPTER_HEADER = '''
#pragma once
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

extern "C" std::uint32_t rclpy_agnocast_adapter_abi_version();
extern "C" const char * rclpy_agnocast_adapter_build_abi();

namespace rclpy_agnocast { namespace types {
constexpr std::uint32_t ADAPTER_ABI_VERSION = 1;
inline std::string compiled_abi_tag() {return "test-abi";}
struct TypeDescriptor {
  std::string name;
  std::string schema_digest;
  std::string fields;
  std::size_t size;
  std::size_t alignment;
};
struct ProviderDescriptor {
  std::uint32_t version;
  std::string provider;
  std::string module;
  std::string build_abi;
  std::string manifest_digest;
  std::vector<TypeDescriptor> types;
};
void register_provider(const ProviderDescriptor &, const std::string &);
}}
'''


_MESSAGE_HEADER = '''
#pragma once
#include <cstdint>
#include <memory>
#include <vector>

namespace rosidl_runtime_cpp {
template<class T, std::size_t Bound, class Allocator>
struct BoundedVector : std::vector<T, Allocator> {};
}
namespace sample { namespace msg {
struct Sample {
  SCALAR_MEMBER
  rosidl_runtime_cpp::BoundedVector<std::uint8_t, SEQUENCE_BOUND,
    std::allocator<std::uint8_t>> values;
};
}}
'''


@pytest.fixture
def generated_source(tmp_path):
    """Generate from IDL and provide only enough ABI declarations for syntax checking."""
    share = tmp_path / 'share' / 'sample'
    (share / 'msg').mkdir(parents=True)
    (share / 'msg' / 'Sample.idl').write_text(
        'module sample { module msg { struct Sample {\n'
        'int32 value; sequence<uint8, 4> values;\n}; }; };\n')
    headers = tmp_path / 'include'
    adapter_header = headers / 'rclpy_agnocast' / 'types' / 'adapter.hpp'
    adapter_header.parent.mkdir(parents=True)
    adapter_header.write_text(_ADAPTER_HEADER)
    message_header = headers / 'sample' / 'msg' / 'detail' / 'sample__struct.hpp'
    message_header.parent.mkdir(parents=True)
    output = tmp_path / 'generated'
    generate('sample_provider', 'sample_provider._adapter', ['sample/msg/Sample'],
             {'sample': share}, 'test-abi', output, 'lib/python/sample_provider/_adapter.so')
    return output / 'provider.cpp', headers, message_header


def _compile(generated_source, scalar_member, sequence_bound):
    source, headers, message_header = generated_source
    message_header.write_text(
        _MESSAGE_HEADER.replace('SCALAR_MEMBER', scalar_member)
        .replace('SEQUENCE_BOUND', str(sequence_bound)))
    compiler = shlex.split(os.environ.get('CXX') or sysconfig.get_config_var('CXX') or 'c++')
    command = compiler + [
        '-std=c++17', '-fsyntax-only', '-fno-diagnostics-color',
        '-I' + str(headers), '-I' + pybind11.get_include(),
        '-I' + str(Path(sysconfig.get_path('include'))), str(source)]
    return subprocess.run(command, text=True, capture_output=True, timeout=30)


def test_matching_cpp_members_compile(generated_source):
    """Accept the scalar and bounded sequence types declared by the IDL."""
    result = _compile(generated_source, 'std::int32_t value;', 4)
    assert result.returncode == 0, result.stderr


def test_generator_rejects_unsupported_adapter_header_version(generated_source):
    """Do not label old generator output as supporting a new descriptor contract."""
    _, headers, _ = generated_source
    header = headers / 'rclpy_agnocast' / 'types' / 'adapter.hpp'
    header.write_text(header.read_text().replace(
        'ADAPTER_ABI_VERSION = 1', 'ADAPTER_ABI_VERSION = 2'))
    result = _compile(generated_source, 'std::int32_t value;', 4)
    assert result.returncode != 0, 'Unsupported adapter ABI was accepted'
    assert 'generator does not support this adapter ABI' in result.stderr, result.stderr


@pytest.mark.parametrize(('scalar_member', 'bound', 'diagnostic'), [
    ('float value;', 4, 'IDL/header mismatch: sample/msg/Sample.value'),
    ('std::int32_t removed_value;', 4, 'value'),
    ('std::int32_t value;', 8, 'IDL/header mismatch: sample/msg/Sample.values'),
])
def test_stale_cpp_member_type_is_rejected(generated_source, scalar_member, bound, diagnostic):
    """Reject same-size scalar changes, missing fields and changed sequence bounds."""
    result = _compile(generated_source, scalar_member, bound)
    assert result.returncode != 0, 'Stale generated message header was accepted'
    assert diagnostic in result.stderr, result.stderr
    if 'removed_value' in scalar_member:
        assert 'no member named' in result.stderr, result.stderr
