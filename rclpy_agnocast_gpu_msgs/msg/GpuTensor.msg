# GPU allocation, FD locator, publication UID and ready event remain core-owned.
ros2_cuda_ipc_msgs/BufferCore core

# DLPack dtype and dense layout. Strides are in elements; byte_offset is in bytes.
uint8 dtype_code
uint8 dtype_bits
uint16 dtype_lanes
int64[] shape
int64[] strides
uint64 byte_offset
uint64 allocation_bytes
