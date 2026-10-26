#include <gtest/gtest.h>

#include "ros2_cuda_ipc_core/publisher/gpu_buffer_manager.hpp"
#include "support.hpp"

namespace
{
using Manager = ros2_cuda_ipc_core::publisher::GpuBufferManager;

class ForeignContext
{
public:
  ForeignContext()
  {
    if (cuCtxGetCurrent(&previous_) != CUDA_SUCCESS) {throw std::runtime_error("get context");}
#if CUDA_VERSION >= 13000
    const auto result = cuCtxCreate(&value_, nullptr, 0, 0);
#else
    const auto result = cuCtxCreate(&value_, 0, 0);
#endif
    if (result != CUDA_SUCCESS) {throw std::runtime_error("create context");}
  }
  ~ForeignContext()
  {
    (void)cuCtxSetCurrent(value_);
    if (stream) {(void)cuStreamDestroy(stream);}
    (void)cuCtxDestroy(value_);
    (void)cuCtxSetCurrent(previous_);
  }
  CUstream stream = nullptr;

private:
  CUcontext value_ = nullptr;
  CUcontext previous_ = nullptr;
};

TEST(RepairedContext, PrimaryDefaultStreamCanPrepareAndClose)
{
  ASSERT_EQ(cudaSetDevice(0), cudaSuccess);
  Manager manager({1, 64, 0});
  ASSERT_TRUE(manager.initialise());
  auto block = manager.acquire_for_publish(nullptr);
  ASSERT_TRUE(block);
  ASSERT_EQ(cudaMemsetAsync(block->device_ptr(), 7, 64, nullptr), cudaSuccess);
  ASSERT_TRUE(block->prepare_publish(nullptr));
  ASSERT_EQ(cudaDeviceSynchronize(), cudaSuccess);
  EXPECT_TRUE(manager.try_close());
}

TEST(RepairedContext, PerThreadDefaultStreamIsRejected)
{
  ASSERT_EQ(cudaSetDevice(0), cudaSuccess);
  Manager manager({1, 64, 0});
  ASSERT_TRUE(manager.initialise());
  EXPECT_FALSE(manager.acquire_for_publish(CU_STREAM_PER_THREAD));
  EXPECT_TRUE(manager.try_close());
}

TEST(RepairedContext, SameDeviceForeignContextAndStreamAreRejected)
{
  ASSERT_EQ(cudaSetDevice(0), cudaSuccess);
  Manager manager({1, 64, 0});
  ASSERT_TRUE(manager.initialise());
  CUcontext primary = nullptr;
  ASSERT_EQ(cuCtxGetCurrent(&primary), CUDA_SUCCESS);
  {
    ForeignContext foreign;
    EXPECT_FALSE(manager.acquire_for_publish(nullptr));
    ASSERT_EQ(cuStreamCreate(&foreign.stream, CU_STREAM_NON_BLOCKING), CUDA_SUCCESS);
    ASSERT_EQ(cuCtxSetCurrent(primary), CUDA_SUCCESS);
    EXPECT_FALSE(manager.acquire_for_publish(foreign.stream));
  }
  EXPECT_TRUE(manager.try_close());
}
}  // namespace
