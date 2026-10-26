#include <gtest/gtest.h>
#include <sys/mman.h>
#include <unistd.h>

#include <filesystem>
#include <atomic>
#include <memory>

#include "ros2_cuda_ipc_core/publisher/gpu_buffer_manager.hpp"
#include "../support.hpp"

namespace
{
using Manager = ros2_cuda_ipc_core::publisher::GpuBufferManager;
using Ref = ros2_cuda_ipc_core::buffer_metadata::BufferRef;
using Metadata = ros2_cuda_ipc_core::buffer_metadata::BufferMetadata;

bool initialise_when_drained(Manager & manager)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  do {
    if (manager.initialise()) {return true;}
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  } while (std::chrono::steady_clock::now() < deadline);
  return false;
}

// One process per RED invocation prevents one sticky error masking another.
class PoolRetirement : public ::testing::TestWithParam<int> {};

TEST_P(PoolRetirement, PendingProducerRetiresThenAllowsReplacement)
{
  ASSERT_EQ(cudaSetDevice(0), cudaSuccess);
  g3_test::Stream stream;
  auto manager = std::make_unique<Manager>(Manager::Config{1, 64, 0});
  ASSERT_TRUE(manager->initialise());
  auto block = manager->acquire_for_publish(stream.driver());
  ASSERT_TRUE(block);
  stream.gate();
  ASSERT_EQ(cudaMemsetAsync(block->device_ptr(), 7, 64, stream.value), cudaSuccess);
  std::string socket;
  std::string metadata_name;
  if (GetParam() == 0) {
    auto descriptor = block->prepare_publish(stream.driver());
    ASSERT_TRUE(descriptor);
    socket = descriptor.value().vmm_socket_path;
    metadata_name = ros2_cuda_ipc_core::buffer_metadata::shm_name_for_block(
      descriptor.value().publisher_pid, descriptor.value().block_id);
    manager.reset();
  } else if (GetParam() == 1) {
    block->cancel();
    manager.reset();
  } else {
    // The block's final owner may disappear after its manager.
    manager.reset();
    block.reset();
  }
  ASSERT_EQ(cudaStreamQuery(stream.value), cudaErrorNotReady);
  Manager replacement({1, 64, 0});
  EXPECT_FALSE(replacement.initialise());
  EXPECT_FALSE(replacement.poisoned());
  stream.release();
  ASSERT_EQ(cudaStreamSynchronize(stream.value), cudaSuccess);
  EXPECT_TRUE(initialise_when_drained(replacement));
  EXPECT_FALSE(replacement.poisoned());
  if (!socket.empty()) {EXPECT_FALSE(std::filesystem::exists(socket));}
  if (!metadata_name.empty()) {EXPECT_FALSE(std::filesystem::exists("/dev/shm" + metadata_name));}
  EXPECT_TRUE(replacement.try_close());
}

INSTANTIATE_TEST_SUITE_P(
  PrepareCancelAndOrphanBlock, PoolRetirement, ::testing::Values(0, 1, 2));

TEST(PoolRetirement, RetainedReaderDrainsBeforeReplacementAllocation)
{
  ASSERT_EQ(cudaSetDevice(0), cudaSuccess);
  g3_test::Stream stream;
  auto manager = std::make_unique<Manager>(Manager::Config{1, 64, 0});
  ASSERT_TRUE(manager->initialise());
  auto block = manager->acquire_for_publish(stream.driver());
  ASSERT_TRUE(block);
  auto descriptor = block->prepare_publish(stream.driver());
  ASSERT_TRUE(descriptor);
  ASSERT_EQ(cudaStreamSynchronize(stream.value), cudaSuccess);
  const auto name = ros2_cuda_ipc_core::buffer_metadata::shm_name_for_block(
    descriptor.value().publisher_pid, descriptor.value().block_id);
  auto metadata = Metadata::attach(name);
  ASSERT_TRUE(metadata);
  auto reader = std::make_unique<Ref>(Ref::acquire(metadata, descriptor.value().uid));
  ASSERT_TRUE(reader->valid());
  manager.reset();
  Manager replacement({1, 64, 0});
  EXPECT_FALSE(replacement.initialise());
  EXPECT_FALSE(replacement.poisoned());
  EXPECT_TRUE(std::filesystem::exists(descriptor.value().vmm_socket_path));
  reader.reset();
  EXPECT_TRUE(initialise_when_drained(replacement));
  EXPECT_FALSE(std::filesystem::exists(descriptor.value().vmm_socket_path));
  EXPECT_FALSE(std::filesystem::exists("/dev/shm" + name));
  EXPECT_TRUE(replacement.try_close());
}

TEST(PoolRetirement, ConcurrentReplacementAttemptsAdmitOnlyOnePoolAfterFinalRelease)
{
  ASSERT_EQ(cudaSetDevice(0), cudaSuccess);
  g3_test::Stream stream;
  auto original = std::make_unique<Manager>(Manager::Config{1, 64, 0});
  ASSERT_TRUE(original->initialise());
  auto block = original->acquire_for_publish(stream.driver());
  ASSERT_TRUE(block);
  stream.gate();
  ASSERT_TRUE(block->prepare_publish(stream.driver()));
  Manager first({1, 64, 0});
  Manager second({1, 64, 0});
  std::atomic<bool> go{false};
  std::atomic<unsigned> winners{0};
  auto compete = [&](Manager & candidate) {
      while (!go.load()) {std::this_thread::yield();}
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
      do {
        if (candidate.initialise()) {winners.fetch_add(1); return;}
        if (winners.load() != 0) {return;}
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      } while (std::chrono::steady_clock::now() < deadline);
    };
  std::thread a(compete, std::ref(first));
  std::thread b(compete, std::ref(second));
  go.store(true);
  original.reset();
  stream.release();
  EXPECT_EQ(cudaStreamSynchronize(stream.value), cudaSuccess);
  a.join();
  b.join();
  EXPECT_EQ(winners.load(), 1u);
  EXPECT_NE(first.is_initialised(), second.is_initialised());
  if (first.is_initialised()) {EXPECT_TRUE(first.try_close());}
  if (second.is_initialised()) {EXPECT_TRUE(second.try_close());}
}
}  // namespace
