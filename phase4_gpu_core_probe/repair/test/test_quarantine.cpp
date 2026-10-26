#include <gtest/gtest.h>
#include <sys/mman.h>
#include <unistd.h>
#include <dlfcn.h>

#include <atomic>
#include <memory>
#include <thread>

#include "ros2_cuda_ipc_core/publisher/gpu_buffer_manager.hpp"
#include "support.hpp"

namespace
{
std::atomic<bool> fail_record{false};
std::atomic<unsigned> failed_records{0};
}  // namespace

extern "C" CUresult CUDAAPI cuEventRecord(CUevent event, CUstream stream)
{
  if (fail_record.load()) {
    failed_records.fetch_add(1);
    return CUDA_ERROR_UNKNOWN;
  }
  using Record = CUresult (CUDAAPI *)(CUevent, CUstream);
  static const auto original = reinterpret_cast<Record>(dlsym(RTLD_NEXT, "cuEventRecord"));
  return original ? original(event, stream) : CUDA_ERROR_UNKNOWN;
}

namespace
{
using Manager = ros2_cuda_ipc_core::publisher::GpuBufferManager;

// This executable is its own process: poisoning must not affect healthy cases.
TEST(RepairedQuarantine, FailedPrepareCannotCreateUnboundedReplacementPools)
{
  ASSERT_EQ(cudaSetDevice(0), cudaSuccess);
  auto manager = std::make_unique<Manager>(Manager::Config{1, 64, 0});
  ASSERT_TRUE(manager->initialise());
  g3_test::Stream owner;
  auto seed = manager->acquire_for_publish(owner.driver());
  ASSERT_TRUE(seed);
  auto descriptor = seed->prepare_publish(owner.driver());
  ASSERT_TRUE(descriptor);
  const auto metadata_name = ros2_cuda_ipc_core::buffer_metadata::shm_name_for_block(
    descriptor.value().publisher_pid, descriptor.value().block_id);
  ASSERT_EQ(cudaStreamSynchronize(owner.value), cudaSuccess);
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  auto block = manager->acquire_for_publish(owner.driver());
  ASSERT_TRUE(block);
  fail_record.store(true);
  EXPECT_FALSE(block->prepare_publish(owner.driver()));
  fail_record.store(false);
  EXPECT_EQ(failed_records.load(), 1u);
  EXPECT_FALSE(block->valid());
  EXPECT_TRUE(manager->poisoned());
  // Actual recording failure, independent of transient NOT_READY timing.
  ASSERT_EQ(cudaStreamSynchronize(owner.value), cudaSuccess);
  EXPECT_FALSE(manager->try_close());
  block.reset();
  manager.reset();
  for (int attempt = 0; attempt < 20; ++attempt) {
    Manager replacement({1, 64, 0});
    EXPECT_FALSE(replacement.initialise());
    EXPECT_TRUE(replacement.poisoned());
  }
  // The quarantined mapping stays live; only this test-owned IPC name is removed.
  EXPECT_EQ(shm_unlink(metadata_name.c_str()), 0);
  EXPECT_EQ(unlink(descriptor.value().vmm_socket_path.c_str()), 0);
}
}  // namespace
