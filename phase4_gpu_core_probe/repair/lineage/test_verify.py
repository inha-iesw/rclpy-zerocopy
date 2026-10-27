"""Exercise lineage rejection and the CMake registration used by the probe."""

import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess

import pytest


HERE = Path(__file__).resolve().parent


@pytest.fixture
def scenario(tmp_path):
    repair = tmp_path / 'probe' / 'repair'
    (repair / 'lineage').mkdir(parents=True)
    (repair.parent / 'upstream.lock.json').write_text(json.dumps({'revision': 'base123'}))
    pristine = tmp_path / 'build/g3/upstream/ros2_cuda_ipc-base123'
    repaired = tmp_path / 'build/g3/repaired/ros2_cuda_ipc'
    files = {}
    for name in ('cuda_driver_context', 'gpu_buffer_pool', 'publisher_messages',
                 'buffer_metadata_cache', 'ipc_handle_cache', 'gpu_buffer_manager',
                 'buffer_ref', 'buffer_mapper', 'read_handle', 'vmm_fd_memory_driver'):
        original = f'test_{name}.cpp'
        if name == 'buffer_mapper':
            original = 'mapper/' + original
        record = {'disposition': 'reused_test', 'target': 'test_upstream_' + name}
        if name in ('gpu_buffer_manager', 'buffer_ref', 'buffer_mapper',
                    'read_handle', 'vmm_fd_memory_driver'):
            record.update(disposition='adapted_test', adapted=f'test/upstream/test_{name}.cpp')
        files['ros2_cuda_ipc_core/test/' + original] = record
    files['ros2_cuda_ipc_core/test/vmm_fd_import_helper.cpp'] = {
        'disposition': 'adapted_helper', 'target': 'g3_vmm_fd_import_helper',
        'adapted': 'test/upstream/vmm_fd_import_helper.cpp'}
    files['ros2_cuda_ipc_core/test/vmm_fd_memory_test_protocol.hpp'] = {
        'disposition': 'support'}
    files['ros2_cuda_ipc_core/test/test_mapper_utils.hpp'] = {
        'disposition': 'unused', 'reason': 'Legacy cache fixture is not used by strict mapping.'}
    for name, record in files.items():
        content = ('int main() {return 0;}\n// ' + name + '\n').encode()
        for root in (pristine, repaired):
            path = root / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(content)
        record['sha256'] = hashlib.sha256(content).hexdigest()
        if 'adapted' in record:
            path = repair / record['adapted']
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(content)
    lock = {'base_revision': 'base123', 'files': files}
    (repair / 'lineage/upstream-tests.lock.json').write_text(json.dumps(lock))
    return repair, pristine, repaired, lock


def verifier():
    spec = importlib.util.spec_from_file_location('g3_lineage', HERE / 'verify.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_all_original_files_have_a_disposition_and_six_adapted_sources(scenario):
    repair, pristine, _, _ = scenario
    entries = verifier().validate(repair, pristine)
    assert len(entries) == 13
    assert sum(record['disposition'].endswith('_test') for record in entries.values()) == 10
    assert sum('adapted' in record for record in entries.values()) == 6
    assert entries['ros2_cuda_ipc_core/test/mapper/test_buffer_mapper.cpp']['adapted'] == (
        'test/upstream/test_buffer_mapper.cpp')


@pytest.mark.parametrize(('fault', 'message'), [
    ('revision', 'revision'), ('hash', 'hash'), ('missing', 'inventory'),
    ('extra', 'inventory'), ('unclassified', 'inventory'),
    ('disposition', 'disposition'), ('duplicate_target', 'duplicate target'),
    ('duplicate_source', 'duplicate adapted'), ('missing_adapted', 'adapted source'),
    ('unused_reason', 'reason'), ('unsafe_path', 'relative path'),
    ('test_as_support', 'test disposition'),
])
def test_invalid_lineage_is_rejected(scenario, fault, message):
    repair, pristine, _, lock = scenario
    name = 'ros2_cuda_ipc_core/test/test_buffer_ref.cpp'
    record = lock['files'][name]
    if fault == 'revision':
        lock['base_revision'] = 'previous'
    elif fault == 'hash':
        record['sha256'] = '0' * 64
    elif fault == 'missing':
        (pristine / name).unlink()
    elif fault == 'extra':
        (pristine / 'ros2_cuda_ipc_core/test/new_test.cpp').write_text('new')
    elif fault == 'unclassified':
        del lock['files'][name]
    elif fault == 'disposition':
        record['disposition'] = 'forgotten'
    elif fault == 'duplicate_target':
        record['target'] = 'test_upstream_gpu_buffer_manager'
    elif fault == 'duplicate_source':
        record['adapted'] = 'test/upstream/test_gpu_buffer_manager.cpp'
    elif fault == 'missing_adapted':
        (repair / record['adapted']).unlink()
    elif fault == 'unused_reason':
        del lock['files']['ros2_cuda_ipc_core/test/test_mapper_utils.hpp']['reason']
    elif fault == 'test_as_support':
        record['disposition'] = 'support'
        del record['target']
        del record['adapted']
    else:
        record['adapted'] = '../outside.cpp'
    (repair / 'lineage/upstream-tests.lock.json').write_text(json.dumps(lock))
    with pytest.raises(ValueError, match=message):
        verifier().validate(repair, pristine)


@pytest.mark.parametrize('bad_hash', [False, True])
def test_cmake_registers_exact_sources_or_rejects_bad_lineage(scenario, tmp_path, bad_hash):
    repair, _, repaired, lock = scenario
    if bad_hash:
        lock['files']['ros2_cuda_ipc_core/test/test_buffer_ref.cpp']['sha256'] = '0' * 64
        (repair / 'lineage/upstream-tests.lock.json').write_text(json.dumps(lock))
    # Use the production registration, with only ament/ROS linkage stubbed out.
    (repair / 'CMakeLists.txt').write_text(f'''cmake_minimum_required(VERSION 3.16)
project(lineage_fixture LANGUAGES CXX)
add_library(ros2_cuda_ipc_core::ros2_cuda_ipc_core INTERFACE IMPORTED)
add_library(CUDA::cudart INTERFACE IMPORTED)
function(ament_add_gtest name source)
  add_executable(${{name}} "${{source}}")
endfunction()
include("{HERE / 'register.cmake'}")
file(WRITE "${{CMAKE_BINARY_DIR}}/groups.txt"
  "${{G3_UPSTREAM_TESTS}}\\n${{G3_UPSTREAM_HELPERS}}\\n")
foreach(name IN LISTS G3_UPSTREAM_TESTS G3_UPSTREAM_HELPERS)
  get_target_property(sources ${{name}} SOURCES)
  file(APPEND "${{CMAKE_BINARY_DIR}}/registered.txt" "${{name}}|${{sources}}\\n")
endforeach()
''')
    build = tmp_path / 'cmake-build'
    result = subprocess.run(['cmake', '-S', str(repair), '-B', str(build)],
                            capture_output=True, text=True, timeout=30)
    if bad_hash:
        assert result.returncode != 0
        assert 'hash' in result.stdout + result.stderr
        return
    assert result.returncode == 0, result.stdout + result.stderr
    lines = (build / 'registered.txt').read_text().splitlines()
    actual = dict(line.split('|', 1) for line in lines)
    expected = {
        record['target']: str(repair / record['adapted'] if 'adapted' in record
                              else repaired / name)
        for name, record in lock['files'].items() if 'target' in record}
    assert actual == expected
    assert len(actual) == 11  # Ten GTests plus the separate child helper.
    tests, helpers = (build / 'groups.txt').read_text().splitlines()
    assert set(tests.split(';')) == {name for name in expected if name.startswith('test_')}
    assert helpers == 'g3_vmm_fd_import_helper'
