#!/usr/bin/env python3
"""Prepare byte-verified source archives without changing any Git repository."""

import hashlib
import io
import json
from pathlib import Path, PurePosixPath
import tarfile
from urllib.request import urlopen
import xml.etree.ElementTree as ET


def prepare(pin, destination):
    """Fetch the locked archive and reject stale or modified extracted sources."""
    revision = pin['revision']
    repository = pin['repository'].removeprefix('https://github.com/')
    archive_path = destination / (revision + '.tar.gz')
    if not archive_path.exists():
        url = f'https://codeload.github.com/{repository}/tar.gz/{revision}'
        data = urlopen(url, timeout=60).read()
        if hashlib.sha256(data).hexdigest() != pin['archive_sha256']:
            raise RuntimeError('downloaded archive digest mismatch: ' + repository)
        archive_path.write_bytes(data)
    data = archive_path.read_bytes()
    if hashlib.sha256(data).hexdigest() != pin['archive_sha256']:
        raise RuntimeError('cached archive digest mismatch: ' + str(archive_path))
    target = destination / (repository.split('/')[-1] + '-' + revision)
    with tarfile.open(fileobj=io.BytesIO(data), mode='r:gz') as archive:
        members = archive.getmembers()
        root = PurePosixPath(members[0].name).parts[0]
        for member in members:
            path = PurePosixPath(member.name)
            if (path.is_absolute() or '..' in path.parts or path.parts[0] != root or
                    not (member.isfile() or member.isdir())):
                raise RuntimeError('unsupported archive entry: ' + member.name)
        if not target.exists():
            archive.extractall(destination)
            extracted = destination / root
            if extracted != target:
                extracted.rename(target)
        expected_files = set()
        for member in members:
            if member.isfile():
                relative = PurePosixPath(member.name).relative_to(root)
                path = target / relative
                expected_files.add(path)
                if path.is_symlink() or path.read_bytes() != archive.extractfile(member).read():
                    raise RuntimeError('modified source: ' + str(path))
        actual_files = {path for path in target.rglob('*') if not path.is_dir()}
        if actual_files != expected_files:
            raise RuntimeError('unexpected source files: ' + str(actual_files - expected_files))
    return target


def main():
    """Validate component licenses and descriptor ABI inputs as well as source bytes."""
    package = Path(__file__).resolve().parents[1]
    workspace = package.parent
    lock = json.loads((package / 'upstream.lock.json').read_text())
    destination = workspace / 'build/g3/upstream'
    destination.mkdir(parents=True, exist_ok=True)
    source = prepare(lock, destination)
    dlpack = prepare(lock['dlpack'], destination)
    for name, expected in lock['components'].items():
        metadata = ET.parse(source / name / 'package.xml').getroot()
        for field in ('version', 'license'):
            if metadata.findtext(field) != expected[field]:
                raise RuntimeError(f'{name}: {field} differs from lock')
    for name, digest in lock['contract_sha256'].items():
        if hashlib.sha256((source / name).read_bytes()).hexdigest() != digest:
            raise RuntimeError('contract digest mismatch: ' + name)
    print(json.dumps({'upstream': str(source), 'dlpack': str(dlpack),
                      'source_bytes_verified': True, 'patches': lock['patches']}, indent=2))


if __name__ == '__main__':
    main()
