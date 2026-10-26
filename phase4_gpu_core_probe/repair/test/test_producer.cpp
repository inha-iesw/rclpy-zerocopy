#include <gtest/gtest.h>

#include <chrono>
#include <thread>

#include "ros2_cuda_ipc_core/publisher/gpu_buffer_manager.hpp"
#include "support.hpp"

namespace
{
using Manager = ros2_cuda_ipc_core::publisher::GpuBufferManager;

TEST(RepairedProducer, CancelDoesNotReissuePendingWriterPointer)
{
  ASSERT_EQ(cudaSetDevice(0), cudaSuccess);
  Manager manager({1, 64, 0});
  ASSERT_TRUE(manager.initialise());
  g3_test::Stream stream;
  auto block = manager.acquire_for_publish(stream.driver());
  ASSERT_TRUE(block);
  stream.gate();
  ASSERT_EQ(cudaMemsetAsync(block->device_ptr(), 0x11, 64, stream.value), cudaSuccess);
  block->cancel();
  ASSERT_EQ(cudaStreamQuery(stream.value), cudaErrorNotReady);
  EXPECT_FALSE(manager.acquire_for_publish(stream.driver()));
  EXPECT_FALSE(manager.try_close());
  EXPECT_FALSE(manager.poisoned());
  stream.release();
  ASSERT_EQ(cudaStreamSynchronize(stream.value), cudaSuccess);
  auto next = manager.acquire_for_publish(stream.driver());
  ASSERT_TRUE(next);
  next->cancel();
  ASSERT_EQ(cudaStreamSynchronize(stream.value), cudaSuccess);
  EXPECT_TRUE(manager.try_close());
}

TEST(RepairedProducer, GraceDoesNotOverridePendingReadyEvent)
{
  ASSERT_EQ(cudaSetDevice(0), cudaSuccess);
  Manager manager({1, 64, 0});
  ASSERT_TRUE(manager.initialise());
  g3_test::Stream stream;
  auto block = manager.acquire_for_publish(stream.driver());
  ASSERT_TRUE(block);
  stream.gate();
  ASSERT_EQ(cudaMemsetAsync(block->device_ptr(), 0x22, 64, stream.value), cudaSuccess);
  ASSERT_TRUE(block->prepare_publish(stream.driver()));
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  ASSERT_EQ(cudaStreamQuery(stream.value), cudaErrorNotReady);
  EXPECT_FALSE(manager.acquire_for_publish(stream.driver()));
  EXPECT_FALSE(manager.try_close());
  stream.release();
  ASSERT_EQ(cudaStreamSynchronize(stream.value), cudaSuccess);
  auto next = manager.acquire_for_publish(stream.driver());
  ASSERT_TRUE(next);
  next->cancel();
  ASSERT_EQ(cudaStreamSynchronize(stream.value), cudaSuccess);
  EXPECT_TRUE(manager.try_close());
}

TEST(RepairedProducer, ActiveReservationMakesCloseBusy)
{
  ASSERT_EQ(cudaSetDevice(0), cudaSuccess);
  Manager manager({1, 64, 0});
  ASSERT_TRUE(manager.initialise());
  g3_test::Stream stream;
  auto block = manager.acquire_for_publish(stream.driver());
  ASSERT_TRUE(block);
  EXPECT_FALSE(manager.try_close());
  ASSERT_NE(block->device_ptr(), nullptr);
  block->cancel();
  ASSERT_EQ(cudaStreamSynchronize(stream.value), cudaSuccess);
  EXPECT_TRUE(manager.try_close());
  EXPECT_FALSE(manager.is_initialised());
}

TEST(RepairedProducer, OneProcessCannotAllocateAnotherPoolWhileFirstIsLive)
{
  ASSERT_EQ(cudaSetDevice(0), cudaSuccess);
  Manager first({1, 64, 0});
  ASSERT_TRUE(first.initialise());
  Manager second({1, 64, 0});
  EXPECT_FALSE(second.initialise());
  EXPECT_TRUE(first.try_close());
  EXPECT_TRUE(second.initialise());
  EXPECT_TRUE(second.try_close());
}

TEST(RepairedProducer, MismatchedPrepareStreamLeavesReservationCancellable)
{
  ASSERT_EQ(cudaSetDevice(0), cudaSuccess);
  Manager manager({1, 64, 0});
  ASSERT_TRUE(manager.initialise());
  g3_test::Stream owner;
  g3_test::Stream other;
  auto block = manager.acquire_for_publish(owner.driver());
  ASSERT_TRUE(block);
  EXPECT_FALSE(block->prepare_publish(other.driver()));
  EXPECT_FALSE(manager.poisoned());
  EXPECT_TRUE(block->valid());
  EXPECT_NE(block->device_ptr(), nullptr);
  block->cancel();
  ASSERT_EQ(cudaStreamSynchronize(owner.value), cudaSuccess);
  EXPECT_TRUE(manager.try_close());
}

}  // namespace
