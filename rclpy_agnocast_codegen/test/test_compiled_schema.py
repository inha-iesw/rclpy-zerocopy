"""Compile generated field operations and assertions against the installed adapter ABI."""

from pathlib import Path
import subprocess
import sys

from ament_index_python.packages import get_package_prefix
import pytest

from rclpy_agnocast_codegen.generate import generate


_CMAKE_PROJECT = '''
cmake_minimum_required(VERSION 3.14)
project(compiled_schema LANGUAGES CXX)
find_package(rclpy_agnocast REQUIRED)
find_package(agnocastlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pybind11_vendor REQUIRED)
find_package(pybind11 CONFIG REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module)
add_library(provider OBJECT generated/provider.cpp)
target_include_directories(provider BEFORE PRIVATE include)
target_link_libraries(provider PRIVATE
  rclpy_agnocast::rclpy_agnocast_types agnocastlib::agnocast rclcpp::rclcpp
  pybind11::headers Python3::Module)
target_compile_options(provider PRIVATE -fno-diagnostics-color)
'''


_MESSAGE_HEADER = '''
#pragma once
#include <cstdint>
#include <memory>
#include "rosidl_runtime_cpp/bounded_vector.hpp"

namespace sample { namespace msg {
struct Sample {
  SCALAR_MEMBER
  rosidl_runtime_cpp::BoundedVector<std::uint8_t, SEQUENCE_BOUND,
    std::allocator<std::uint8_t>> values;
};
}}
'''


_MESSAGE_TRAITS = '''
#pragma once
#include "sample/msg/detail/sample__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"
namespace rosidl_generator_traits {
template<> struct is_message<sample::msg::Sample> : std::true_type {};
template<> inline const char * name<sample::msg::Sample>() {return "sample/msg/Sample";}
}
'''


@pytest.fixture
def generated_source(tmp_path):
    """Compile a synthetic message against the real ABI, field helpers and factory."""
    share = tmp_path / 'share' / 'sample'
    (share / 'msg').mkdir(parents=True)
    (share / 'msg' / 'Sample.idl').write_text(
        'module sample { module msg { struct Sample {\n'
        'int32 value; sequence<uint8, 4> values;\n}; }; };\n')
    headers = tmp_path / 'include'
    message_header = headers / 'sample' / 'msg' / 'detail' / 'sample__struct.hpp'
    message_header.parent.mkdir(parents=True)
    (message_header.parent / 'sample__traits.hpp').write_text(_MESSAGE_TRAITS)
    output = tmp_path / 'generated'
    generate('sample_provider', 'sample_provider._adapter', ['sample/msg/Sample'],
             {'sample': share}, 'test-abi', output, 'lib/python/sample_provider/_adapter.so')
    (tmp_path / 'CMakeLists.txt').write_text(_CMAKE_PROJECT)
    return output / 'provider.cpp', headers, message_header


def _compile(generated_source, scalar_member, sequence_bound):
    source, _, message_header = generated_source
    message_header.write_text(
        _MESSAGE_HEADER.replace('SCALAR_MEMBER', scalar_member)
        .replace('SEQUENCE_BOUND', str(sequence_bound)))
    root = source.parent.parent
    configured = subprocess.run(
        ['cmake', '-S', str(root), '-B', str(root / 'build'),
         '-DPython3_EXECUTABLE=' + sys.executable],
        text=True, capture_output=True, timeout=30)
    assert configured.returncode == 0, configured.stdout + configured.stderr
    return subprocess.run(
        ['cmake', '--build', str(root / 'build'), '-j1'],
        text=True, capture_output=True, timeout=45)


def test_matching_cpp_members_compile(generated_source):
    """Accept the scalar and bounded sequence types declared by the IDL."""
    result = _compile(generated_source, 'std::int32_t value;', 4)
    assert result.returncode == 0, result.stderr


def test_generator_rejects_unsupported_adapter_header_version(generated_source):
    """Do not label old generator output as supporting a new descriptor contract."""
    _, headers, _ = generated_source
    header = headers / 'rclpy_agnocast' / 'types' / 'adapter.hpp'
    header.parent.mkdir(parents=True)
    installed = (Path(get_package_prefix('rclpy_agnocast')) /
                 'include' / header.relative_to(headers))
    contents = installed.read_text()
    changed = contents.replace('ADAPTER_ABI_VERSION = 3', 'ADAPTER_ABI_VERSION = 4')
    assert changed != contents, 'The installed adapter ABI fixture is stale'
    header.write_text(changed)
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
