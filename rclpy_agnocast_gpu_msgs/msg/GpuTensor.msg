# GPU allocation, FD locator, publication UID and ready event remain core-owned.
# Experimental dense Tensor envelope; allocation_bytes must equal core.byte_size.
ros2_cuda_ipc_msgs/BufferCore core

# The probe accepts DLPack code 0/1/2 (int/uint/float), lanes 1 and bits
# 8/16/32/64, except float8. Shape extents are nonnegative; rank 0 is a scalar
# and a zero extent makes the Tensor empty. Empty strides imply dense layout.
# Explicit strides have the same rank as shape and describe dense layout;
# strides of size-one dimensions do not affect validation.
# Strides are in elements; byte_offset is in bytes and aligned to item size.
# The Tensor span must fit allocation_bytes without arithmetic overflow.
# Empty Tensors may have byte_offset == allocation_bytes. Aligned nonzero
# offsets are valid here; G4's initial offset-0 restriction belongs to its adapter.
uint8 dtype_code
uint8 dtype_bits
uint16 dtype_lanes
int64[] shape
int64[] strides
uint64 byte_offset
uint64 allocation_bytes
