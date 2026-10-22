"""Validate the runtime extension against its actual shared registry dependency."""

import ctypes
import json
import os
import shlex
import subprocess
import sys
import sysconfig
import textwrap

import pytest

from rclpy_agnocast import _native


def test_runtime_reports_abi_of_loaded_registry():
    """Resolve symbols through the extension handle, including RTLD_LOCAL dependencies."""
    library = ctypes.CDLL(_native.__file__)
    library.rclpy_agnocast_adapter_abi_version.argtypes = []
    library.rclpy_agnocast_adapter_abi_version.restype = ctypes.c_uint32
    library.rclpy_agnocast_adapter_build_abi.argtypes = []
    library.rclpy_agnocast_adapter_build_abi.restype = ctypes.c_char_p
    assert _native._adapter_abi() == {
        'version': library.rclpy_agnocast_adapter_abi_version(),
        'build': library.rclpy_agnocast_adapter_build_abi().decode(),
    }


@pytest.mark.parametrize('mismatch', ['version', 'build'])
def test_runtime_import_rejects_incompatible_registry(tmp_path, mismatch):
    """Reject each mismatched C identity before publishing the runtime module."""
    abi = _native._adapter_abi()
    version = abi['version'] + (mismatch == 'version')
    build = abi['build'] + (';incompatible' if mismatch == 'build' else '')
    source = tmp_path / 'registry_identity.cpp'
    source.write_text(textwrap.dedent(f'''\
        #include <cstdint>
        extern "C" std::uint32_t rclpy_agnocast_adapter_abi_version() {{return {version};}}
        extern "C" const char * rclpy_agnocast_adapter_build_abi() {{return {json.dumps(build)};}}
        '''))
    library = tmp_path / 'registry_identity.so'
    compiler = shlex.split(os.environ.get('CXX') or sysconfig.get_config_var('CXX') or 'c++')
    compiled = subprocess.run(compiler + ['-shared', '-fPIC', str(source), '-o', str(library)],
                              capture_output=True, text=True, timeout=30)
    assert compiled.returncode == 0, compiled.stderr
    environment = os.environ.copy()
    environment['LD_PRELOAD'] = str(library) + ' ' + environment.get('LD_PRELOAD', '')
    probe = textwrap.dedent('''\
        import sys
        try:
            from rclpy_agnocast import _native
        except ImportError as error:
            assert 'runtime adapter ABI mismatch' in str(error), str(error)
        else:
            raise AssertionError('incompatible registry accepted')
        assert 'rclpy_agnocast._native' not in sys.modules
        print('RUNTIME_ABI_REJECTED')
        ''')
    result = subprocess.run([sys.executable, '-c', probe], env=environment,
                            capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stdout + result.stderr
    assert 'RUNTIME_ABI_REJECTED' in result.stdout
