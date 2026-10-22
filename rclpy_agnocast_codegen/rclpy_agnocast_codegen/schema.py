"""Describe ROS message fields and hash their complete IDL dependencies."""

import hashlib
import json
from pathlib import Path
import re

from rosidl_parser import definition as idl
from rosidl_parser.parser import parse_idl_file


_MESSAGE_TYPE = re.compile(r'[A-Za-z_][A-Za-z_0-9]*/msg/[A-Za-z_][A-Za-z_0-9]*\Z')


def _logical_path(ros_type):
    if not _MESSAGE_TYPE.fullmatch(ros_type):
        raise ValueError(f'Expected a message type pkg/msg/Name, got {ros_type!r}')
    return ros_type + '.idl'


class _Schema:
    def __init__(self, package_shares):
        self.package_shares = package_shares
        self.messages = {}
        self.hashes = {}

    def load(self, ros_type):
        logical_path = _logical_path(ros_type)
        if ros_type in self.messages:
            return self.messages[ros_type]
        package, _, name = ros_type.split('/')
        if package not in self.package_shares:
            raise ValueError(f'Missing explicit package share for {package!r} ({ros_type})')
        share = Path(self.package_shares[package])
        locator = idl.IdlLocator(share, Path('msg') / (name + '.idl'))
        path = locator.get_absolute_path()
        try:
            raw = path.read_bytes()
            content = parse_idl_file(locator).content
        except (OSError, AssertionError, ValueError) as error:
            raise ValueError(f'Cannot read message schema {logical_path}: {error}') from error
        messages = content.get_elements_of_type(idl.Message)
        matching = [message for message in messages if
                    '/'.join(message.structure.namespaced_type.namespaced_name()) == ros_type]
        if len(matching) != 1 or len(messages) != 1:
            raise ValueError(f'Schema {logical_path} must define exactly {ros_type}')
        self.messages[ros_type] = matching[0]
        self.hashes[logical_path] = hashlib.sha256(raw).hexdigest()
        for include in content.get_elements_of_type(idl.Include):
            if not include.locator.endswith('.idl'):
                raise ValueError(f'Unsupported include {include.locator!r} in {logical_path}')
            included_type = include.locator[:-4]
            if not _MESSAGE_TYPE.fullmatch(included_type):
                raise ValueError(f'Unsupported include {include.locator!r} in {logical_path}')
            self.load(included_type)
        return matching[0]

    def fields(self, ros_type, stack=()):
        if ros_type in stack:
            raise ValueError('Recursive message schema: ' + ' -> '.join((*stack, ros_type)))
        message = self.load(ros_type)
        stack = (*stack, ros_type)
        return [{'name': member.name, 'type': self.field_type(member.type, stack)}
                for member in message.structure.members]

    def field_type(self, type_, stack):
        if isinstance(type_, idl.BasicType):
            return {'kind': 'scalar', 'type': type_.typename}
        if isinstance(type_, idl.Array):
            return {'kind': 'array', 'length': type_.size,
                    'element': self.field_type(type_.value_type, stack)}
        if isinstance(type_, idl.AbstractSequence):
            return {'kind': 'sequence', 'bound': getattr(type_, 'maximum_size', None),
                    'element': self.field_type(type_.value_type, stack)}
        if isinstance(type_, (idl.AbstractString, idl.AbstractWString)):
            return {'kind': 'wstring' if isinstance(type_, idl.AbstractWString) else 'string',
                    'bound': getattr(type_, 'maximum_size', None)}
        if isinstance(type_, idl.NamespacedType):
            ros_type = '/'.join(type_.namespaced_name())
            return {'kind': 'message', 'type': ros_type, 'fields': self.fields(ros_type, stack)}
        raise ValueError(f'Unsupported ROSIDL field type: {type(type_).__name__}')


def _describe_type(ros_type, package_shares):
    schema = _Schema(package_shares)
    fields = schema.fields(ros_type)
    hashes = dict(sorted(schema.hashes.items()))
    canonical = json.dumps(hashes, sort_keys=True, separators=(',', ':'))
    description = {'schema_digest': hashlib.sha256(canonical.encode('utf-8')).hexdigest(),
                   'schemas': hashes, 'fields': fields}
    return description, schema.messages


def describe_type(ros_type, package_shares):
    """Return fields and content hashes, resolving only explicitly supplied shares.

    ``schemas`` keys are logical ``pkg/msg/Name.idl`` paths. Its sorted compact
    JSON encoding defines ``schema_digest``, independently of installation paths.
    Services, actions, recursive fields and mismatched declarations are rejected.
    """
    return _describe_type(ros_type, package_shares)[0]
