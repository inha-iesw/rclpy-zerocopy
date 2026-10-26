#include "checksum.hpp"

namespace g3_transport
{
namespace
{
__global__ void sum_bytes(const uint8_t * data, unsigned * result)
{
  atomicAdd(result, static_cast<unsigned>(data[threadIdx.x]));
}
}  // namespace

void checksum(const uint8_t * data, unsigned * result, cudaStream_t stream)
{
  sum_bytes<<<1, 64, 0, stream>>>(data, result);
}
}  // namespace g3_transport
