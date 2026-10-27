// Copyright (c) 2026 Daisuke Kato
// SPDX-License-Identifier: MIT

#include <fcntl.h>
#include <gtest/gtest.h>
#include <sys/mman.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <set>
#include <string>
#include <vector>

#include "ros2_cuda_ipc_core/buffer_metadata/buffer_ref.hpp"
#include "ros2_cuda_ipc_core/publisher/gpu_buffer_manager.hpp"
#include "../support.hpp"

namespace {
std::atomic<int64_t> clock_us{0};
}

// Freeze only this executable's clock; GPU completion is observed separately.
extern "C" std::chrono::steady_clock::time_point __real__ZNSt6chrono3_V212steady_clock3nowEv();
extern "C" std::chrono::steady_clock::time_point __wrap__ZNSt6chrono3_V212steady_clock3nowEv() {
  const auto value = clock_us.load();
  return value == 0 ? __real__ZNSt6chrono3_V212steady_clock3nowEv() :
         std::chrono::steady_clock::time_point(std::chrono::microseconds(value));
}

namespace ros2_cuda_ipc_core::publisher {
namespace {
using Ref = buffer_metadata::BufferRef;
using Metadata = buffer_metadata::BufferMetadata;

class GpuBufferManagerTest : public ::testing::Test {
 protected:
  void SetUp() override { ASSERT_EQ(cudaSetDevice(0), cudaSuccess); }
};
}  // namespace

TEST_F(GpuBufferManagerTest, DescriptorDirectlyIdentifiesBlock) {
  g3_test::Stream stream;
  GpuBufferManager manager({2, 1024, 0});
  ASSERT_TRUE(manager.initialise());
  std::set<uint32_t> ids;
  std::vector<Ref> readers;
  for (int i = 0; i < 2; ++i) {
    auto block = manager.acquire_for_publish(stream.driver());
    ASSERT_TRUE(block);
    const auto descriptor = block->prepare_publish(stream.driver());
    ASSERT_TRUE(descriptor);
    EXPECT_EQ(descriptor.value().publisher_pid, static_cast<uint32_t>(::getpid()));
    EXPECT_TRUE(ids.insert(descriptor.value().block_id).second);
    EXPECT_NE(descriptor.value().uid, 0u);
    auto mapping = Metadata::attach(buffer_metadata::shm_name_for_block(
        descriptor.value().publisher_pid, descriptor.value().block_id));
    ASSERT_TRUE(mapping);
    readers.push_back(Ref::acquire(mapping, descriptor.value().uid));
    ASSERT_TRUE(readers.back().valid());
  }
  readers.clear();
  ASSERT_EQ(cudaStreamSynchronize(stream.value), cudaSuccess);
  EXPECT_TRUE(manager.try_close());
}

TEST_F(GpuBufferManagerTest, ResetUnlinksAllBlockMetadataObjects) {
  g3_test::Stream stream;
  GpuBufferManager manager({2, 1024, 0});
  ASSERT_TRUE(manager.initialise());
  auto first = manager.acquire_for_publish(stream.driver());
  auto second = manager.acquire_for_publish(stream.driver());
  ASSERT_TRUE(first);
  ASSERT_TRUE(second);
  std::vector<std::string> names;
  for (auto* block : {&*first, &*second}) {
    const auto descriptor = block->prepare_publish(stream.driver());
    ASSERT_TRUE(descriptor);
    names.push_back(buffer_metadata::shm_name_for_block(
        descriptor.value().publisher_pid, descriptor.value().block_id));
  }
  ASSERT_EQ(cudaStreamSynchronize(stream.value), cudaSuccess);
  manager.reset();
  EXPECT_FALSE(manager.is_initialised());
  for (const auto& name : names) {
    const int fd = ::shm_open(name.c_str(), O_RDONLY, 0);
    const int error = errno;
    EXPECT_EQ(fd, -1);
    EXPECT_EQ(error, ENOENT);
    if (fd != -1) ::close(fd);
  }
}

TEST_F(GpuBufferManagerTest, SuccessivePoolsUseDifferentBlockIds) {
  g3_test::Stream stream;
  GpuBufferManager first({1, 1024, 0});
  GpuBufferManager second({1, 1024, 0});
  ASSERT_TRUE(first.initialise());
  EXPECT_FALSE(second.initialise());
  auto first_block = first.acquire_for_publish(stream.driver());
  ASSERT_TRUE(first_block);
  const auto first_descriptor = first_block->prepare_publish(stream.driver());
  ASSERT_TRUE(first_descriptor);
  ASSERT_EQ(cudaStreamSynchronize(stream.value), cudaSuccess);
  ASSERT_TRUE(first.try_close());
  ASSERT_TRUE(second.initialise());
  auto second_block = second.acquire_for_publish(stream.driver());
  ASSERT_TRUE(second_block);
  const auto second_descriptor = second_block->prepare_publish(stream.driver());
  ASSERT_TRUE(second_descriptor);
  EXPECT_NE(first_descriptor.value().block_id, second_descriptor.value().block_id);
  ASSERT_EQ(cudaStreamSynchronize(stream.value), cudaSuccess);
  EXPECT_TRUE(second.try_close());
}

TEST_F(GpuBufferManagerTest, ReservationsUseDistinctBlocksAndExhaustThePool) {
  g3_test::Stream stream;
  GpuBufferManager manager({2, 1024, 0});
  ASSERT_TRUE(manager.initialise());
  auto first = manager.acquire_for_publish(stream.driver());
  auto second = manager.acquire_for_publish(stream.driver());
  ASSERT_TRUE(first);
  ASSERT_TRUE(second);
  EXPECT_NE(first->device_ptr(), second->device_ptr());
  EXPECT_FALSE(manager.acquire_for_publish(stream.driver()));
  first->cancel();
  second->cancel();
  ASSERT_EQ(cudaStreamSynchronize(stream.value), cudaSuccess);
  EXPECT_TRUE(manager.try_close());
}

TEST_F(GpuBufferManagerTest, CancelReturnsTheSameBlockToThePool) {
  g3_test::Stream stream;
  GpuBufferManager manager({2, 1024, 0});
  ASSERT_TRUE(manager.initialise());
  auto first = manager.acquire_for_publish(stream.driver());
  auto second = manager.acquire_for_publish(stream.driver());
  ASSERT_TRUE(first);
  ASSERT_TRUE(second);
  // Keep the first writer reserved, so reuse cannot depend on a grace timeout.
  void* second_ptr = second->device_ptr();
  ASSERT_NE(second_ptr, nullptr);
  second->cancel();
  ASSERT_EQ(cudaStreamSynchronize(stream.value), cudaSuccess);
  auto reused = manager.acquire_for_publish(stream.driver());
  ASSERT_TRUE(reused);
  EXPECT_EQ(reused->device_ptr(), second_ptr);
  ASSERT_TRUE(reused->prepare_publish(stream.driver()));
  first->cancel();
  ASSERT_EQ(cudaStreamSynchronize(stream.value), cudaSuccess);
  EXPECT_TRUE(manager.try_close());
}

TEST_F(GpuBufferManagerTest, CompletedPublicationWaitsForGraceBeforeReuse) {
  g3_test::Stream stream;
  GpuBufferManager manager({1, 1024, 0});
  ASSERT_TRUE(manager.initialise());
  auto block = manager.acquire_for_publish(stream.driver());
  ASSERT_TRUE(block);
  void* pointer = block->device_ptr();
  struct FrozenClock {
    FrozenClock() { clock_us = 1000000; }
    ~FrozenClock() { clock_us = 0; }
  } clock;
  const auto descriptor = block->prepare_publish(stream.driver());
  ASSERT_TRUE(descriptor);
  ASSERT_EQ(cudaStreamSynchronize(stream.value), cudaSuccess);
  auto mapping = Metadata::attach(buffer_metadata::shm_name_for_block(
      descriptor.value().publisher_pid, descriptor.value().block_id));
  ASSERT_TRUE(mapping);
  ASSERT_EQ(Ref::current_refcount(mapping), 0u);
  // This also proves the pool's linked core observes the frozen clock.
  ASSERT_EQ(Ref::current_publish_timestamp_us(mapping), 1000000u);
  EXPECT_FALSE(manager.acquire_for_publish(stream.driver()));
  clock_us = 1099999;
  EXPECT_FALSE(manager.acquire_for_publish(stream.driver()));
  EXPECT_FALSE(manager.poisoned());
  clock_us = 1100000;
  auto reused = manager.acquire_for_publish(stream.driver());
  ASSERT_TRUE(reused);
  EXPECT_EQ(reused->device_ptr(), pointer);
  const auto next = reused->prepare_publish(stream.driver());
  ASSERT_TRUE(next);
  EXPECT_EQ(next.value().block_id, descriptor.value().block_id);
  EXPECT_NE(next.value().uid, descriptor.value().uid);
  ASSERT_EQ(cudaStreamSynchronize(stream.value), cudaSuccess);
  EXPECT_TRUE(manager.try_close());
}

TEST_F(GpuBufferManagerTest, SubscriberReferencePreventsReuseAfterCommit) {
  g3_test::Stream stream;
  GpuBufferManager manager({1, 1024, 0});
  ASSERT_TRUE(manager.initialise());
  auto block = manager.acquire_for_publish(stream.driver());
  ASSERT_TRUE(block);
  const auto descriptor = block->prepare_publish(stream.driver());
  ASSERT_TRUE(descriptor);
  auto mapping = Metadata::attach(buffer_metadata::shm_name_for_block(
      descriptor.value().publisher_pid, descriptor.value().block_id));
  ASSERT_TRUE(mapping);
  ASSERT_EQ(cudaStreamSynchronize(stream.value), cudaSuccess);
  // GPU completion is independently observed before expiring this fixture's grace.
  mapping->metadata()->publish_timestamp_us.store(1);
  {
    auto read_ref = Ref::acquire(mapping, descriptor.value().uid);
    ASSERT_TRUE(read_ref.valid());
    EXPECT_EQ(Ref::current_refcount(mapping), 1u);
    EXPECT_FALSE(manager.acquire_for_publish(stream.driver()));
    EXPECT_FALSE(manager.try_close());
    EXPECT_FALSE(manager.poisoned());
  }
  auto next = manager.acquire_for_publish(stream.driver());
  ASSERT_TRUE(next);
  const auto next_descriptor = next->prepare_publish(stream.driver());
  ASSERT_TRUE(next_descriptor);
  EXPECT_NE(next_descriptor.value().uid, descriptor.value().uid);
  ASSERT_EQ(cudaStreamSynchronize(stream.value), cudaSuccess);
  EXPECT_TRUE(manager.try_close());
}

}  // namespace ros2_cuda_ipc_core::publisher
