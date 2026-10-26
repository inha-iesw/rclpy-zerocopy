"""Internal Torch adapter for Field.as_tensor() and its native DLPack owner."""

import operator
import sys


def as_tensor(owner, path, generation, *, shape=None):
    """Keep native storage until the last Torch storage alias, without an intermediate buffer."""
    info = owner.tensor_info(path, generation)
    if shape is None:
        shape = (info['size'],)
    # Reject malformed input before importing Torch. Native entry points repeat these
    # checks because they can be called directly without this facade.
    dimensions = []
    product = 1
    stride_volume = 1
    for dimension in shape:
        if isinstance(dimension, bool):
            raise TypeError('Tensor shape dimensions must be integers')
        dimension = operator.index(dimension)
        if dimension < 0 or dimension > sys.maxsize:
            raise ValueError('Tensor shape dimensions must fit non-negative int64')
        extent = max(dimension, 1)
        if stride_volume > sys.maxsize // extent:
            raise ValueError('Tensor shape stride volume overflows int64')
        stride_volume *= extent
        product *= dimension
        dimensions.append(dimension)
    if product != info['size']:
        raise ValueError('Tensor shape product must equal native field size')

    # Ordinary message imports and buffer users do not require Torch or load CUDA libraries.
    import torch
    from torch.utils.dlpack import from_dlpack

    dtype = getattr(torch, info['dtype'], None)
    if not isinstance(dtype, torch.dtype):
        raise NotImplementedError(f"installed Torch does not support {info['dtype']}")
    capsule = owner.dlpack(path, generation, dimensions)
    try:
        return from_dlpack(capsule)
    except BaseException:
        # A saved exception traceback may keep an unconsumed capsule alive indefinitely.
        # Consumed capsules belong to the consumer and must never be released here.
        owner.discard_dlpack(capsule)
        raise
