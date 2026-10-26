#!/usr/bin/env python3
"""Reconstruct the experimental core from locked upstream bytes and one patch."""

import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


def inventory(root):
    """Return source bytes only, rejecting silently modified prepared copies."""
    return {str(path.relative_to(root)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in root.rglob('*') if path.is_file() and '__pycache__' not in path.parts}


def main():
    """Apply the patch with zero fuzz and verify the complete reconstructed tree."""
    scenario = Path(__file__).resolve().parent
    workspace = scenario.parents[1]
    prepared = subprocess.run(
        [sys.executable, str(scenario.parent / 'scripts/prepare_upstream.py')],
        check=True, capture_output=True, text=True)
    pristine = Path(json.loads(prepared.stdout)['upstream'])
    patch = scenario / 'patches/core-lifecycle.patch'
    parent = workspace / 'build/g3/repaired'
    parent.mkdir(parents=True, exist_ok=True)
    destination = parent / 'ros2_cuda_ipc'
    with tempfile.TemporaryDirectory(prefix='reconstruct-', dir=parent) as temporary:
        staged = Path(temporary) / 'source'
        shutil.copytree(pristine, staged, ignore=shutil.ignore_patterns('__pycache__'))
        subprocess.run(
            ['patch', '--batch', '--fuzz=0', '-p1', '-i', str(patch)],
            cwd=staged, check=True, capture_output=True, text=True)
        expected = inventory(staged)
        if destination.exists():
            if inventory(destination) != expected:
                raise RuntimeError(
                    'prepared core differs from reconstructed patch; preserve and inspect it')
        else:
            shutil.copytree(staged, destination)
    print(json.dumps({'source': str(destination),
                      'upstream_revision': '0e8933c8e58ffa7f1240a4c9efda3a2b84e5398c',
                      'patch_sha256': hashlib.sha256(patch.read_bytes()).hexdigest(),
                      'source_sha256': expected, 'reconstruction_verified': True}, indent=2))


if __name__ == '__main__':
    main()
