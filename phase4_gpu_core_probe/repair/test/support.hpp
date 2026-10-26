#pragma once

#include <cuda.h>
#include <cuda_runtime_api.h>

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <stdexcept>

namespace g3_test
{
inline void check(cudaError_t result)
{
  if (result != cudaSuccess) {
    throw std::runtime_error(cudaGetErrorString(result));
  }
}

// Release a pending stream before destruction, including assertion unwind.
class Stream
{
public:
  Stream() {check(cudaStreamCreateWithFlags(&value, cudaStreamNonBlocking));}
  ~Stream()
  {
    release();
    (void)cudaStreamSynchronize(value);
    (void)cudaStreamDestroy(value);
  }

  void gate()
  {
    check(cudaLaunchHostFunc(value, wait, this));
    std::unique_lock<std::mutex> lock(mutex_);
    if (!condition_.wait_for(lock, std::chrono::seconds(3), [this] {return entered_;})) {
      throw std::runtime_error("CUDA gate did not start");
    }
  }

  void release()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      released_ = true;
    }
    condition_.notify_all();
  }

  CUstream driver() const {return reinterpret_cast<CUstream>(value);}
  cudaStream_t value = nullptr;

private:
  static void CUDART_CB wait(void * opaque)
  {
    auto & self = *static_cast<Stream *>(opaque);
    std::unique_lock<std::mutex> lock(self.mutex_);
    self.entered_ = true;
    self.condition_.notify_all();
    self.condition_.wait_for(
      lock, std::chrono::seconds(10), [&self] {return self.released_;});
  }

  std::mutex mutex_;
  std::condition_variable condition_;
  bool entered_ = false;
  bool released_ = false;
};
}  // namespace g3_test
