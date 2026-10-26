#include <cuda.h>
#include <cuda_runtime_api.h>
#include <gtest/gtest.h>
#include <dlfcn.h>

#include <atomic>

#include "ros2_cuda_ipc_core/publisher/gpu_buffer_manager.hpp"

namespace
{
std::atomic<bool> fail_release{false};
std::atomic<unsigned> failed_releases{0};
}  // namespace

// Interpose only this test process. The injected failure leaves the actual
// driver retain unresolved; process exit owns its eventual cleanup.
extern "C" CUresult CUDAAPI cuDevicePrimaryCtxRelease(CUdevice device)
{
  if (fail_release.load()) {
    failed_releases.fetch_add(1);
    return CUDA_ERROR_UNKNOWN;
  }
  using Release = CUresult (CUDAAPI *)(CUdevice);
  static const auto original = reinterpret_cast<Release>(
    dlsym(RTLD_NEXT, "cuDevicePrimaryCtxRelease_v2"));
  return original ? original(device) : CUDA_ERROR_UNKNOWN;
}

TEST(RepairedPrimaryRelease, FailedReleasePreventsRepeatedRetainsAcrossPoolRecreation)
{
  using Manager = ros2_cuda_ipc_core::publisher::GpuBufferManager;
  ASSERT_EQ(cudaSetDevice(0), cudaSuccess);
  Manager first({1, 64, 0});
  ASSERT_TRUE(first.initialise());
  fail_release.store(true);
  EXPECT_FALSE(first.try_close());
  EXPECT_TRUE(first.poisoned());
  ASSERT_EQ(failed_releases.load(), 1u);
  for (unsigned attempt = 0; attempt < 20; ++attempt) {
    Manager replacement({1, 64, 0});
    EXPECT_FALSE(replacement.initialise()) << attempt;
    EXPECT_TRUE(replacement.poisoned());
  }
  EXPECT_EQ(failed_releases.load(), 1u);
  fail_release.store(false);
}
