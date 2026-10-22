"""Build-system entry point for deterministic native provider generation."""

import argparse
from pathlib import Path

from .generate import generate


def main():
    """Generate a provider from explicit schema paths and a compiled ABI tag."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--provider', required=True)
    parser.add_argument('--module', required=True)
    parser.add_argument('--type', dest='ros_types', action='append', required=True)
    parser.add_argument('--package', action='append', required=True, metavar='PKG=SHARE')
    parser.add_argument('--abi-file', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--library-relative', required=True)
    args = parser.parse_args()
    shares = {}
    for entry in args.package:
        package, separator, share = entry.partition('=')
        if not separator or not package or not share or package in shares:
            parser.error(f'Expected one unique PKG=SHARE per package, got {entry!r}')
        shares[package] = Path(share)
    try:
        generate(args.provider, args.module, args.ros_types, shares,
                 args.abi_file.read_text(encoding='utf-8').strip(), args.output_dir,
                 args.library_relative)
    except (OSError, ValueError) as error:
        parser.error(str(error))


if __name__ == '__main__':
    main()
