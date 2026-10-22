"""Exercise installed adapter build rules through independent temporary packages."""

import hashlib
import json
import os
import subprocess
import sys
import textwrap

import pytest


_PROVIDER = 'build_contract_provider'


def _run(command, *, env=None, timeout=45, expected_returncode=0):
    result = subprocess.run(
        [str(argument) for argument in command], env=env,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=timeout)
    assert result.returncode == expected_returncode, (
        f'Command {command!r} returned {result.returncode}:\n{result.stdout}')
    return result.stdout


def _write_project(source, *, types=('std_msgs/msg/Int32',), target='_types', debug=False):
    source.mkdir(exist_ok=True)
    (source / 'package.xml').write_text(textwrap.dedent(f'''\
        <?xml version="1.0"?>
        <package format="3">
          <name>{_PROVIDER}</name>
          <version>0.1.0</version>
          <description>Temporary adapter build contract fixture.</description>
          <maintainer email="maintainers@example.com">Maintainers</maintainer>
          <license>Apache-2.0</license>
          <buildtool_depend>ament_cmake</buildtool_depend>
          <buildtool_depend>rclpy_agnocast_codegen</buildtool_depend>
          <depend>std_msgs</depend>
          <export><build_type>ament_cmake</build_type></export>
        </package>
        '''), encoding='utf-8')
    debug_definition = (
        f'target_compile_definitions({target} PRIVATE _GLIBCXX_DEBUG)\n' if debug else '')
    (source / 'CMakeLists.txt').write_text(textwrap.dedent(f'''\
        cmake_minimum_required(VERSION 3.14)
        project({_PROVIDER})
        find_package(ament_cmake REQUIRED)
        find_package(rclpy_agnocast_codegen REQUIRED)
        rclpy_agnocast_generate_adapters({target} TYPES {' '.join(types)})
        ''') + debug_definition + 'ament_package()\n', encoding='utf-8')


def _configure(source, build, *, env=None):
    return _run([
        'cmake', '-S', source, '-B', build, '-G', 'Unix Makefiles',
        f'-DPython3_EXECUTABLE={sys.executable}', '-DCMAKE_BUILD_TYPE=Debug'], env=env)


def _build(build, *, env=None):
    _run(['cmake', '--build', build, '--parallel', '2'], env=env, timeout=60)


def _snapshot(source):
    return {
        str(path.relative_to(source)): path.read_bytes()
        for path in source.rglob('*') if path.is_file()}


def _module_path(build, target):
    libraries = list(build.glob(f'{target}*.so'))
    assert len(libraries) == 1, libraries
    return libraries[0]


def test_type_selection_rebuilds_manifest_without_source_writes_or_overlay_generator(tmp_path):
    """Regenerate changed TYPES, preserve sources, and ignore a shadow Python generator."""
    source, build = tmp_path / 'source', tmp_path / 'build'
    shadow = tmp_path / 'shadow' / 'rclpy_agnocast_codegen'
    shadow.mkdir(parents=True)
    (shadow / '__init__.py').write_text(
        'raise RuntimeError("shadow generator must not be imported")\n', encoding='utf-8')
    environment = os.environ.copy()
    environment['PYTHONPATH'] = (
        str(shadow.parent) + os.pathsep + environment.get('PYTHONPATH', ''))
    _write_project(source)
    authored = _snapshot(source)
    _configure(source, build, env=environment)
    _build(build, env=environment)
    manifest = build / '_types' / 'manifest.json'
    assert set(json.loads(manifest.read_text())['types']) == {'std_msgs/msg/Int32'}
    assert _snapshot(source) == authored

    cmake = source / 'CMakeLists.txt'
    cmake.write_text(
        cmake.read_text().replace(
            'TYPES std_msgs/msg/Int32)', 'TYPES std_msgs/msg/Int32 std_msgs/msg/Float32)'),
        encoding='utf-8')
    authored['CMakeLists.txt'] = cmake.read_bytes()
    _configure(source, build, env=environment)
    _build(build, env=environment)
    assert set(json.loads(manifest.read_text())['types']) == {
        'std_msgs/msg/Int32', 'std_msgs/msg/Float32'}
    assert _snapshot(source) == authored

    outputs = [manifest, build / '_types' / 'provider.cpp', _module_path(build, '_types')]
    before = {path: hashlib.sha256(path.read_bytes()).hexdigest() for path in outputs}
    _build(build, env=environment)
    assert {path: hashlib.sha256(path.read_bytes()).hexdigest() for path in outputs} == before
    assert _snapshot(source) == authored


def test_plugin_with_incompatible_stl_abi_is_rejected_before_registration(tmp_path):
    """Import a debug-STL plugin safely and retain an empty runtime type registry."""
    source, build = tmp_path / 'source', tmp_path / 'build'
    _write_project(source, target='_bad', debug=True)
    _configure(source, build)
    _build(build)
    manifest = json.loads((build / '_bad' / 'manifest.json').read_text())
    assert ';libstdc++debug=0' in manifest['build_abi']
    probe = textwrap.dedent('''\
        import importlib.util
        import sys
        from rclpy_agnocast import _native

        def assert_unregistered():
            try:
                _native._registered_type('std_msgs/msg/Int32')
            except IndexError:
                return
            raise AssertionError('incompatible plugin registered a type')

        assert_unregistered()
        spec = importlib.util.spec_from_file_location(
            'build_contract_provider._bad', sys.argv[1])
        try:
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
        except ImportError as error:
            assert 'adapter ABI mismatch' in str(error), str(error)
        else:
            raise AssertionError('incompatible plugin imported successfully')
        assert_unregistered()
        print('ABI_REJECTED_REGISTRY_UNCHANGED')
        ''')
    output = _run([sys.executable, '-c', probe, _module_path(build, '_bad')], timeout=15)
    assert 'ABI_REJECTED_REGISTRY_UNCHANGED' in output


def test_in_source_configuration_fails_before_adapter_output(tmp_path):
    """Reject in-source configuration without generating provider artifacts."""
    source = tmp_path / 'source'
    _write_project(source)
    output = _run([
        'cmake', '-S', source, '-B', source,
        f'-DPython3_EXECUTABLE={sys.executable}'], expected_returncode=1)
    assert 'requires an out-of-source build' in output
    assert not (source / '_types').exists()
    assert not list(source.rglob('provider.cpp'))
    assert not list(source.rglob('manifest.json'))


@pytest.mark.parametrize('in_subdirectory', [False, True])
def test_second_provider_in_same_package_is_rejected(tmp_path, in_subdirectory):
    """Reject duplicate provider ownership even across CMake directory scopes."""
    source, build = tmp_path / 'source', tmp_path / 'build'
    _write_project(source)
    second = 'rclpy_agnocast_generate_adapters(_other TYPES std_msgs/msg/Float32)\n'
    if in_subdirectory:
        nested = source / 'nested'
        nested.mkdir()
        (nested / 'CMakeLists.txt').write_text(second)
        second = 'add_subdirectory(nested)\n'
    cmake = source / 'CMakeLists.txt'
    cmake.write_text(cmake.read_text().replace('ament_package()', second + 'ament_package()'))
    output = _run([
        'cmake', '-S', source, '-B', build,
        f'-DPython3_EXECUTABLE={sys.executable}'], expected_returncode=1)
    assert 'Only one adapter provider per package is supported' in output
