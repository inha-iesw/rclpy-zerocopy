"""Direct native field access and independent layout evidence for typed facade TLD."""

from process import fill as fill_fixture
from process import fill_string, STRING_SEQUENCES, STRINGS


FRAME = 'typed_interop_frame_with_heap_storage'
ARRAYS = {
    'fixed': [1.25, -2.5, 0.0, 4.0], 'bounded': [0.5, -1.5, 2.5],
    'values': [0, 1, 127, 128, 255], 'fixed_flags': [True, False, True, False],
    'bounded_flags': [False, True, False], 'flags': [True, False, True, False, True],
}


def fill(message, kind):
    """Prepare every dynamic path before writing into native payload storage."""
    if kind == 'Containers':
        fill_fixture(message, kind)
        return
    message.header.frame_id.resize(len(FRAME))
    message.data.resize(18 if kind == 'Image' else 24)
    if kind == 'Image':
        message.encoding.resize(4)
    else:
        message.fields.resize(2)
        for index in range(2):
            message.fields[index].name.resize(1)
    message.header.stamp.sec = 123
    message.header.stamp.nanosec = 456
    fill_string(message.header.frame_id, 'frame_id', FRAME)
    message.height = 2 if kind == 'Image' else 1
    message.width = 3
    message.is_bigendian = 0 if kind == 'Image' else False
    if kind == 'Image':
        fill_string(message.encoding, 'encoding', 'rgb8')
        message.step = 9
    else:
        for index, name in enumerate(['x', 'y']):
            fill_string(message.fields[index].name, 'name', name)
            message.fields[index].offset = index * 4
            message.fields[index].datatype = 7
            message.fields[index].count = 1
        message.point_step = 8
        message.row_step = 24
        message.is_dense = True
    with message.data.buffer() as view:
        for index in range(len(view)):
            view[index] = (7 * index + 3) & 255


def validate(message, kind):
    """Check each field through the received proxy, including nested and wide strings."""
    if kind == 'Containers':
        for name, expected in ARRAYS.items():
            field = getattr(message, name)
            assert len(field) == len(expected)
            assert [field[index] for index in range(len(field))] == expected
        for name, expected in STRINGS.items():
            assert getattr(message, name).to_text() == expected
        for name, expected in STRING_SEQUENCES.items():
            field = getattr(message, name)
            assert len(field) == len(expected)
            assert [field[index].to_text() for index in range(len(field))] == expected
        return
    assert message.header.stamp.sec == 123
    assert message.header.stamp.nanosec == 456
    assert message.header.frame_id.to_text() == FRAME
    assert message.height == (2 if kind == 'Image' else 1)
    assert message.width == 3
    assert message.is_bigendian == 0
    if kind == 'Image':
        assert message.encoding.to_text() == 'rgb8'
        assert message.step == 9
    else:
        assert len(message.fields) == 2
        for index, name in enumerate(['x', 'y']):
            field = message.fields[index]
            assert field.name.to_text() == name
            assert field.offset == index * 4
            assert field.datatype == 7
            assert field.count == 1
        assert message.point_step == 8
        assert message.row_step == 24
        assert message.is_dense is True
    expected = [(7 * index + 3) & 255 for index in range(18 if kind == 'Image' else 24)]
    assert len(message.data) == len(expected)
    with message.data.buffer() as view:
        assert list(view) == expected


def payload(message, kind):
    """Select the native contiguous payload used for same-process pointer evidence."""
    return message.values if kind == 'Containers' else message.data


def observe(message, kind, root_address):
    """Compare actual Python buffer data with independent native member addresses."""
    import _typed_oracle

    with payload(message, kind).buffer() as view:
        result = _typed_oracle.inspect(kind, root_address, view)
    assert result['native_data_address'] == result['view_data_address'] != 0
    assert result['accounted_bytes'] == message._owner.snapshot['accounted_bytes']
    return result['accounted_bytes']
