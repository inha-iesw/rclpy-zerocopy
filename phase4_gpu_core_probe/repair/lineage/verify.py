"""Check adapted-test ancestry and emit the probe's CMake source registry."""

import hashlib
import json
from pathlib import Path, PurePosixPath
import re
import sys


def relative_path(value):
    """Require a literal relative source path suitable for CMake registration."""
    path = PurePosixPath(value)
    if (path.is_absolute() or '..' in path.parts or
            not re.fullmatch(r'[A-Za-z0-9_./-]+', value)):
        raise ValueError('invalid relative path: ' + value)
    return path


def validate(scenario, pristine):
    """Reject pin drift, unclassified originals, or broken adapted-source mappings."""
    pin = json.loads((scenario.parent / 'upstream.lock.json').read_text())
    lock = json.loads((scenario / 'lineage/upstream-tests.lock.json').read_text())
    if lock['base_revision'] != pin['revision']:
        raise ValueError('test lineage base revision differs from upstream pin')
    entries = lock['files']
    actual = {path.relative_to(pristine).as_posix()
              for path in (pristine / 'ros2_cuda_ipc_core/test').rglob('*') if path.is_file()}
    if actual != set(entries):
        raise ValueError('upstream test inventory differs: missing=' +
                         str(sorted(set(entries) - actual)) +
                         ', unclassified=' + str(sorted(actual - set(entries))))
    targets = set()
    adapted_sources = set()
    for name, record in entries.items():
        original = pristine / relative_path(name)
        if hashlib.sha256(original.read_bytes()).hexdigest() != record['sha256']:
            raise ValueError('upstream test base hash differs: ' + name)
        kind = record['disposition']
        if kind not in ('reused_test', 'adapted_test', 'adapted_helper', 'support', 'unused'):
            raise ValueError('unknown disposition: ' + name)
        if (original.name.startswith('test_') and original.suffix == '.cpp' and
                kind not in ('reused_test', 'adapted_test', 'unused')):
            raise ValueError('test source requires a test disposition or unused reason: ' + name)
        if kind in ('support', 'unused'):
            if 'target' in record or 'adapted' in record:
                raise ValueError('non-target disposition has a target/source: ' + name)
            if kind == 'unused' and not record.get('reason'):
                raise ValueError('unused source requires a reason: ' + name)
            continue
        target = record['target']
        if not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', target):
            raise ValueError('invalid target: ' + target)
        if target in targets:
            raise ValueError('duplicate target: ' + target)
        targets.add(target)
        if kind.startswith('adapted_'):
            adapted = relative_path(record['adapted'])
            if adapted in adapted_sources:
                raise ValueError('duplicate adapted source: ' + str(adapted))
            adapted_sources.add(adapted)
            if not (scenario / adapted).is_file():
                raise ValueError('missing adapted source: ' + str(adapted))
        elif 'adapted' in record:
            raise ValueError('reused source must not have an adapted path: ' + name)
    return entries


def cmake_registry(entries, scenario, repaired):
    """Map each classified executable to its original or adapted source."""
    lines = ['# Generated from the tracked upstream-test lineage; do not edit.',
             'set(G3_UPSTREAM_TESTS)', 'set(G3_UPSTREAM_HELPERS)']
    for name, record in entries.items():
        if 'target' not in record:
            continue
        target = record['target']
        source = scenario / record['adapted'] if 'adapted' in record else repaired / name
        group = 'HELPERS' if record['disposition'] == 'adapted_helper' else 'TESTS'
        lines.append(f'list(APPEND G3_UPSTREAM_{group} {target})')
        lines.append(f'set(G3_UPSTREAM_SOURCE_{target} "{source.as_posix()}")')
    return '\n'.join(lines) + '\n'


def main():
    """Print a registry for CMake; validation itself does not write build artifacts."""
    scenario = Path(sys.argv[1]).resolve()
    workspace = scenario.parents[1]
    pin = json.loads((scenario.parent / 'upstream.lock.json').read_text())
    pristine = workspace / 'build/g3/upstream' / ('ros2_cuda_ipc-' + pin['revision'])
    repaired = workspace / 'build/g3/repaired/ros2_cuda_ipc'
    print(cmake_registry(validate(scenario, pristine), scenario, repaired), end='')


if __name__ == '__main__':
    main()
