#include <cuda_runtime_api.h>
#include <gtest/gtest.h>

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <stdexcept>
#include <thread>

#include "ros2_cuda_ipc_core/publisher/gpu_buffer_manager.hpp"

namespace
{
using Manager = ros2_cuda_ipc_core::publisher::GpuBufferManager;

void check(cudaError_t error)
{
  if (error != cudaSuccess) {
    throw std::runtime_error(cudaGetErrorString(error));
  }
}

// Hold a producer stream before its queued device write, without changing the
// candidate. Release on unwind and bound the callback wait to prevent a hang.
class PendingProducer
{
public:
  PendingProducer() {check(cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking));}

  ~PendingProducer()
  {
    finish();
    (void)cudaStreamDestroy(stream_);
  }

  void enqueue(void * pointer)
  {
    check(cudaLaunchHostFunc(stream_, wait, this));
    check(cudaMemsetAsync(pointer, 0x11, 64, stream_));
    std::unique_lock<std::mutex> lock(mutex_);
    if (!condition_.wait_for(lock, std::chrono::seconds(3), [this] {return entered_;})) {
      throw std::runtime_error("producer callback did not start");
    }
  }

  cudaStream_t stream() const {return stream_;}

  void finish() noexcept
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      released_ = true;
    }
    condition_.notify_all();
    (void)cudaStreamSynchronize(stream_);
  }

private:
  static void CUDART_CB wait(void * opaque)
  {
    auto & self = *static_cast<PendingProducer *>(opaque);
    std::unique_lock<std::mutex> lock(self.mutex_);
    self.entered_ = true;
    self.condition_.notify_all();
    self.condition_.wait_for(
      lock, std::chrono::seconds(10), [&self] {return self.released_;});
  }

  cudaStream_t stream_ = nullptr;
  std::mutex mutex_;
  std::condition_variable condition_;
  bool entered_ = false;
  bool released_ = false;
};

TEST(CandidateRejection, CancelReissuesPointerWhileProducerWriteIsPending)
{
  ASSERT_EQ(cudaSetDevice(0), cudaSuccess);
  Manager manager({1, 64, 0});
  ASSERT_TRUE(manager.initialise());
  PendingProducer producer;
  auto first = manager.acquire_for_publish();
  ASSERT_TRUE(first);
  void * original = first->device_ptr();
  producer.enqueue(original);
  ASSERT_EQ(cudaStreamQuery(producer.stream()), cudaErrorNotReady);

  first->cancel();
  auto second = manager.acquire_for_publish();
  ASSERT_TRUE(second) << "Candidate behavior changed: reassess the adoption verdict";
  EXPECT_EQ(second->device_ptr(), original);
  EXPECT_EQ(cudaStreamQuery(producer.stream()), cudaErrorNotReady);
  RecordProperty("hazard", "cancel_reuses_pointer_before_producer_completion");
  second->cancel();
  producer.finish();
}

TEST(CandidateRejection, PreparedPointerIsReissuedBeforeReadyAfterGrace)
{
  ASSERT_EQ(cudaSetDevice(0), cudaSuccess);
  Manager manager({1, 64, 0});
  ASSERT_TRUE(manager.initialise());
  PendingProducer producer;
  auto first = manager.acquire_for_publish();
  ASSERT_TRUE(first);
  void * original = first->device_ptr();
  producer.enqueue(original);
  auto descriptor = first->prepare_publish(reinterpret_cast<CUstream>(producer.stream()));
  ASSERT_TRUE(descriptor);
  EXPECT_FALSE(manager.acquire_for_publish());

  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  ASSERT_EQ(cudaStreamQuery(producer.stream()), cudaErrorNotReady);
  auto second = manager.acquire_for_publish();
  ASSERT_TRUE(second) << "Candidate behavior changed: reassess the adoption verdict";
  EXPECT_EQ(second->device_ptr(), original);
  EXPECT_EQ(cudaStreamQuery(producer.stream()), cudaErrorNotReady);
  RecordProperty("hazard", "prepared_reuse_while_ready_event_is_pending");
  second->cancel();
  producer.finish();
}

TEST(CandidateControl, CancelAfterProducerCompletionAllowsReuse)
{
  ASSERT_EQ(cudaSetDevice(0), cudaSuccess);
  Manager manager({1, 64, 0});
  ASSERT_TRUE(manager.initialise());
  PendingProducer producer;
  auto first = manager.acquire_for_publish();
  ASSERT_TRUE(first);
  void * original = first->device_ptr();
  producer.enqueue(original);
  producer.finish();
  ASSERT_EQ(cudaStreamQuery(producer.stream()), cudaSuccess);
  first->cancel();
  auto second = manager.acquire_for_publish();
  ASSERT_TRUE(second);
  EXPECT_EQ(second->device_ptr(), original);
}
}  // namespace
