#pragma once

#include <cuda_runtime_api.h>

#include <cstdint>

namespace g3_transport
{
void checksum(const uint8_t * data, unsigned * result, cudaStream_t stream);
}  // namespace g3_transport
