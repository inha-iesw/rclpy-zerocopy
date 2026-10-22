"""Validate installed schemas and provider identity before native plugin import."""

import hashlib
import importlib.machinery
import importlib.util
import json
from pathlib import Path, PurePosixPath
import re
import sys
import threading

from ament_index_python.packages import get_package_share_directory
from ament_index_python.resources import get_resource, get_resources

from .. import _native


_RESOURCE = 'rclpy_agnocast_adapters'
_TYPE = re.compile(r'[a-z][a-z0-9_]*/msg/[A-Z][a-zA-Z0-9]*')
_MODULE = re.compile(r'[a-zA-Z_]\w*(?:\.[a-zA-Z_]\w*)+')
_LOAD_LOCK = threading.RLock()


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f'duplicate manifest key: {key}')
        result[key] = value
    return result


def _relative_path(value):
    if not isinstance(value, str):
        raise ValueError('adapter resource path must be a string')
    path = PurePosixPath(value)
    if path.is_absolute() or '..' in path.parts or not path.parts:
        raise ValueError(f'invalid adapter resource path: {value}')
    return path


def _read_manifest(provider, prefix):
    resource, selected_prefix = get_resource(_RESOURCE, provider)
    if selected_prefix != prefix:
        raise ValueError(f'adapter provider selection changed: {provider}')
    relative = _relative_path(resource.strip())
    # Symlink installs may point outside the prefix; the resource path itself
    # must remain relative, and package selection follows ament overlay order.
    data = (Path(prefix) / relative).read_bytes()
    manifest = json.loads(data, object_pairs_hook=_unique_object)
    if not isinstance(manifest, dict) or manifest.get('provider') != provider:
        raise ValueError(f'adapter provider identity mismatch: {provider}')
    if manifest.get('manifest_version') != 1:
        raise ValueError(f'unsupported adapter manifest version: {provider}')
    module = manifest.get('module')
    if not isinstance(module, str) or not _MODULE.fullmatch(module):
        raise ValueError(f'invalid adapter module: {provider}')
    entries = manifest.get('types')
    if not isinstance(entries, dict) or not entries:
        raise ValueError(f'adapter provider has no types: {provider}')
    if any(not _TYPE.fullmatch(name) for name in entries):
        raise ValueError(f'invalid ROS type in provider: {provider}')
    return manifest, hashlib.sha256(data).hexdigest()


def _validate_schema(name, entry):
    if not isinstance(entry, dict):
        raise ValueError(f'invalid schema metadata: {name}')
    schemas = entry.get('schemas')
    if not isinstance(schemas, dict) or f'{name}.idl' not in schemas:
        raise ValueError(f'missing root schema: {name}')
    if not isinstance(entry.get('fields'), list):
        raise ValueError(f'missing field descriptors: {name}')
    for logical, expected in schemas.items():
        path = _relative_path(logical)
        if len(path.parts) != 3 or not logical.endswith('.idl') or not _TYPE.fullmatch(
                logical[:-4]):
            raise ValueError(f'invalid schema path: {logical}')
        package, category, filename = path.parts
        source = Path(get_package_share_directory(package)) / category / filename
        actual = hashlib.sha256(source.read_bytes()).hexdigest()
        if actual != expected:
            raise ValueError(f'adapter schema mismatch: {logical}; rebuild adapter')
    digest = hashlib.sha256(json.dumps(
        schemas, sort_keys=True, separators=(',', ':')).encode()).hexdigest()
    if entry.get('schema_digest') != digest:
        raise ValueError(f'adapter schema digest mismatch: {name}')


def _load_plugin(module_name, library):
    # Load the ament-selected binary directly. Python's search path may have a
    # different overlay order, and executing a parent package could import it.
    with _LOAD_LOCK:
        existing = sys.modules.get(module_name)
        if existing is not None:
            origin = getattr(existing, '__file__', None)
            if origin is None or Path(origin).resolve() != library.resolve():
                raise ValueError(f'adapter module origin mismatch: {module_name}')
            return existing
        spec = importlib.util.spec_from_file_location(module_name, library)
        plugin = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(plugin)
        sys.modules[module_name] = plugin
        return plugin


def _resolve_type(name):
    manifests = []
    providers = {}
    modules = {}
    for provider, prefix in sorted(get_resources(_RESOURCE).items()):
        manifest, digest = _read_manifest(provider, prefix)
        module = manifest['module']
        if module in modules:
            raise ValueError(f'adapter module has conflicting providers: {module}')
        modules[module] = provider
        for ros_type in manifest['types']:
            if ros_type in providers:
                raise ValueError(f'conflicting adapter providers for {ros_type}: '
                                 f'{providers[ros_type]} and {provider}')
            providers[ros_type] = provider
        manifests.append((manifest, digest, prefix))
    if name not in providers:
        raise LookupError(f'{name}: build and install an Agnocast adapter for this type')
    manifest, digest, prefix = next(item for item in manifests if name in item[0]['types'])
    abi = _native._adapter_abi()
    if manifest.get('adapter_abi') != abi['version'] or manifest.get('build_abi') != abi['build']:
        raise ValueError(f'adapter ABI mismatch: {name}; rebuild adapter')
    # Registering a provider is atomic, so validate every type it will register.
    for ros_type, entry in manifest['types'].items():
        _validate_schema(ros_type, entry)
    library = Path(prefix) / _relative_path(manifest.get('library'))
    stem = manifest['module'].rsplit('.', 1)[-1]
    if library.name not in [stem + suffix for suffix in importlib.machinery.EXTENSION_SUFFIXES]:
        raise ValueError(f'invalid adapter library: {library}')
    if not library.is_file():
        raise ValueError(f'missing adapter library: {library}; rebuild adapter')
    plugin = _load_plugin(manifest['module'], library)
    plugin._register(digest)
    result = _native._registered_type(name)
    if result['schema_digest'] != manifest['types'][name]['schema_digest']:
        raise ValueError(f'native adapter schema mismatch: {name}')
    result['fields'] = json.loads(result.pop('fields_json'))
    return result


def resolve_adapter(message_type):
    """Return native adapter metadata for a generated ROS message class.

    The original class is a lookup key only. No owning message is constructed
    or converted. Missing adapters and incompatible providers raise before an
    endpoint can be created; there is no serialization fallback.
    """
    if not isinstance(message_type, type):
        raise TypeError('adapter lookup requires a ROS message class')
    parts = message_type.__module__.split('.')
    if len(parts) != 3 or parts[1] != 'msg' or not parts[2].startswith('_'):
        raise TypeError('adapter lookup requires a generated ROS message class')
    name = f'{parts[0]}/msg/{message_type.__name__}'
    if not _TYPE.fullmatch(name):
        raise TypeError('adapter lookup requires a generated ROS message class')
    return _resolve_type(name)
